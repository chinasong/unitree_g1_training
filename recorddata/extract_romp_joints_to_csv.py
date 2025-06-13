import os
import numpy as np
import pandas as pd
from tqdm import tqdm
from scipy.interpolate import interp1d

ROMP_OUTPUT_DIR = "../externals/ROMP/output"  # 修改为你的ROMP输出路径
OUTPUT_CSV = "romp_output_g1.csv"
FRAME_RATE = 30  # 原始视频帧率
INTERP_RATIO = 6  # 插值倍数，插值后为 180fps
OUTPUT_RATE = FRAME_RATE * INTERP_RATIO

# G1 关节映射表
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

# CSV 表头
columns = ["time"]
for i in range(29):
    name = G1_JOINT_MAPPING.get(i, f"joint_{i}")
    columns += [f"{name}_q", f"{name}_dq", f"{name}_tau"]

# 解析 ROMP 输出的 pose 数据
npz_files = sorted(f for f in os.listdir(ROMP_OUTPUT_DIR) if f.endswith(".npz"))
print(f"Found {len(npz_files)} frames.")
pose_list = []

for idx, fname in enumerate(tqdm(npz_files)):
    path = os.path.join(ROMP_OUTPUT_DIR, fname)
    try:
        data = np.load(path, allow_pickle=True)['results'].item()
        global_orient = data['global_orient'].reshape(-1)  # (3,)
        body_pose = data['body_pose'].reshape(-1)  # (69,)
        full_pose = np.concatenate([global_orient, body_pose])  # (72,)

        q_frame = []
        for joint_id in range(29):
            if joint_id < 24:
                q = full_pose[joint_id * 3]  # 只取1轴角度
            else:
                q = 0.0
            q_frame.append(q)

        pose_list.append(q_frame)
    except Exception as e:
        print(f"Failed to process {fname}: {e}")

# 时间轴插值准备
pose_array = np.array(pose_list)
original_times = np.arange(len(pose_array)) / FRAME_RATE
interp_times = np.linspace(0, original_times[-1], len(pose_array) * INTERP_RATIO)

# 插值每个关节角度
interp_q = []
for j in range(pose_array.shape[1]):
    f = interp1d(original_times, pose_array[:, j], kind='linear')
    interp_q.append(f(interp_times))
interp_q = np.array(interp_q).T  # shape: (N, 29)

# 估算 dq
dq_array = np.gradient(interp_q, 1.0 / OUTPUT_RATE, axis=0)
tau_array = np.zeros_like(dq_array)

# 合并数据行
rows = []
for i, t in enumerate(interp_times):
    q_vals = interp_q[i]
    dq_vals = dq_array[i]
    tau_vals = tau_array[i]
    row = [t] + [val for triple in zip(q_vals, dq_vals, tau_vals) for val in triple]
    rows.append(row)

# 写入 CSV
df = pd.DataFrame(rows, columns=columns)
df.to_csv(OUTPUT_CSV, index=False)
print(f"✅ Interpolated G1 motion CSV saved to: {OUTPUT_CSV}")