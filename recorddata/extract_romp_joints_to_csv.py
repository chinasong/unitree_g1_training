import os
import numpy as np
import pandas as pd
from tqdm import tqdm
from scipy.interpolate import interp1d

ROMP_OUTPUT_DIR = "/home/ubuntu/unitree_g1_training/externals/ROMP/output"
OUTPUT_CSV = "romp_output_g1.csv"
FRAME_RATE = 30
INTERP_RATIO = 6
OUTPUT_RATE = FRAME_RATE * INTERP_RATIO
SMOOTH_WINDOW = 5

G1_JOINT_MAPPING = {
    0: "L_LEG_HIP_PITCH", 1: "L_LEG_HIP_ROLL", 2: "L_LEG_HIP_YAW",
    3: "L_LEG_KNEE", 4: "L_LEG_ANKLE_PITCH", 5: "L_LEG_ANKLE_ROLL",
    6: "R_LEG_HIP_PITCH", 7: "R_LEG_HIP_ROLL", 8: "R_LEG_HIP_YAW",
    9: "R_LEG_KNEE", 10: "R_LEG_ANKLE_PITCH", 11: "R_LEG_ANKLE_ROLL",
    12: "WAIST_YAW",
    15: "L_SHOULDER_PITCH", 16: "L_SHOULDER_ROLL", 17: "L_SHOULDER_YAW",
    18: "L_ELBOW", 19: "L_WRIST_ROLL", 20: "L_WRIST_PITCH", 21: "L_WRIST_YAW",
    22: "R_SHOULDER_PITCH", 23: "R_SHOULDER_ROLL", 24: "R_SHOULDER_YAW",
    25: "R_ELBOW", 26: "R_WRIST_ROLL", 27: "R_WRIST_PITCH", 28: "R_WRIST_YAW"
}

columns = ["time"]
for i in range(29):
    name = G1_JOINT_MAPPING.get(i, f"joint_{i}")
    columns += [f"{name}_q", f"{name}_dq", f"{name}_tau"]

pose_list = []
npz_files = sorted(f for f in os.listdir(ROMP_OUTPUT_DIR) if f.endswith(".npz"))
print(f"📂 Found {len(npz_files)} npz frames")

for fname in tqdm(npz_files):
    fpath = os.path.join(ROMP_OUTPUT_DIR, fname)
    try:
        data = np.load(fpath, allow_pickle=True)["results"].item()
        global_orient = data.get("global_orient", np.zeros((1, 3))).reshape(-1)
        body_pose = data.get("body_pose", np.zeros((1, 69))).reshape(-1)
        full_pose = np.concatenate([global_orient, body_pose])
        q_frame = []
        for joint_id in range(29):
            angle_idx = joint_id * 3
            q = full_pose[angle_idx] if angle_idx < len(full_pose) else 0.0
            q_frame.append(q)
        pose_list.append(q_frame)
    except Exception as e:
        print(f"⚠️ {fname} failed: {e}")

pose_array = np.array(pose_list)
original_times = np.arange(len(pose_array)) / FRAME_RATE
interp_times = np.linspace(0, original_times[-1], len(pose_array) * INTERP_RATIO)

interp_q = []
for j in range(pose_array.shape[1]):
    interp_func = interp1d(original_times, pose_array[:, j], kind='cubic', fill_value="extrapolate")
    smooth_q = pd.Series(interp_func(interp_times)).rolling(window=SMOOTH_WINDOW, min_periods=1, center=True).mean()
    interp_q.append(smooth_q.values)
interp_q = np.array(interp_q).T

dq_array = np.gradient(interp_q, 1.0 / OUTPUT_RATE, axis=0)
tau_array = np.zeros_like(dq_array)

rows = []
for i, t in enumerate(interp_times):
    q = interp_q[i]
    dq = dq_array[i]
    tau = tau_array[i]
    row = [t] + [v for triplet in zip(q, dq, tau) for v in triplet]
    rows.append(row)

df = pd.DataFrame(rows, columns=columns)
df.to_csv(OUTPUT_CSV, index=False)
print(f"✅ CSV saved to {OUTPUT_CSV}, total frames: {len(rows)}")