import os
import numpy as np
import pandas as pd
from tqdm import tqdm

ROMP_OUTPUT_DIR = "../externals/ROMP/output"  # 修改为你的路径
OUTPUT_CSV = "romp_output_g1.csv"
FRAME_RATE = 30  # 假设视频是 30fps

# G1 joint mapping
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

# CSV columns
columns = ["time"]
for i in range(29):
    name = G1_JOINT_MAPPING.get(i, f"joint_{i}")
    columns += [f"{name}_q", f"{name}_dq", f"{name}_tau"]

rows = []
npz_files = sorted(f for f in os.listdir(ROMP_OUTPUT_DIR) if f.endswith(".npz"))

print(f"Found {len(npz_files)} frames.")
last_q_frame = None

for idx, fname in enumerate(tqdm(npz_files)):
    path = os.path.join(ROMP_OUTPUT_DIR, fname)
    try:
        data = np.load(path, allow_pickle=True)['results'].item()
        global_orient = data['global_orient'].reshape(-1)  # (3,)
        body_pose = data['body_pose'].reshape(-1)  # (69,)
        full_pose = np.concatenate([global_orient, body_pose])  # (72,) = 24 joints * 3

        q_frame = []
        for joint_id in range(29):
            if joint_id < 24:
                q = full_pose[joint_id * 3]  # 只取 1 个轴
            else:
                q = 0.0  # ROMP 不输出额外关节
            q_frame.append(q)

        # 估算 dq（速度） 和 tau（力矩）
        if last_q_frame is not None:
            dq_frame = [(q - last_q) * FRAME_RATE for q, last_q in zip(q_frame, last_q_frame)]
        else:
            dq_frame = [0.0] * 29

        tau_frame = [0.0] * 29  # 暂时置零

        row = [idx / FRAME_RATE] + \
              [val for triple in zip(q_frame, dq_frame, tau_frame) for val in triple]
        rows.append(row)
        last_q_frame = q_frame

    except Exception as e:
        print(f"Failed to process {fname}: {e}")
        continue

# 写入CSV
df = pd.DataFrame(rows, columns=columns)
df.to_csv(OUTPUT_CSV, index=False)
print(f"✅ Saved G1 CSV to: {OUTPUT_CSV}")