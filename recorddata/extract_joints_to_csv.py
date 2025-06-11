import os
import numpy as np
import pandas as pd
from tqdm import tqdm

# ROMP输出目录
input_folder = "../externals/ROMP/output"
# 输出CSV路径
output_csv_path = "romp_output_g1.csv"

# G1关节编号映射
JOINT_MAPPING = {
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

# 每个关节：q, dq, tau
columns = ['time']
for idx in range(29):
    name = JOINT_MAPPING.get(idx, f"joint_{idx}")
    columns.extend([f"{name}_q", f"{name}_dq", f"{name}_tau"])

# 读取所有 npz 文件
npz_files = sorted([f for f in os.listdir(input_folder) if f.endswith(".npz")])
print(f"Total npz files found: {len(npz_files)}")

pose_history = []  # 保存每一帧的 full_pose（用于计算速度和加速度）
rows = []

for i, file in enumerate(tqdm(npz_files)):
    file_path = os.path.join(input_folder, file)
    print(f"\nProcessing {file_path} ...")
    data = np.load(file_path, allow_pickle=True)

    if 'results' not in data:
        print(f"Skipped {file} (no 'results')")
        continue

    results = data['results'].item()
    keys = list(results.keys())
    print("Available keys in results:", keys)

    try:
        pose = results['body_pose'].reshape(-1)  # (1, 69)
        root_orient = results['global_orient'].reshape(-1)  # (1, 3)
        full_pose = np.concatenate([root_orient, pose])  # (72,)
        pose_history.append(full_pose)

        # 计算时间
        frame_time = i * 0.033  # 假设 30 FPS
        row = [frame_time]

        for j in range(29):
            if j < 24:
                q = full_pose[j * 3]
            else:
                q = 0.0

            # 速度 dq
            if i > 0 and j < 24:
                q_prev = pose_history[i - 1][j * 3]
                dq = (q - q_prev) / 0.033
            else:
                dq = 0.0

            # 加速度（简化为差分速度），估算 tau
            if i > 1 and j < 24:
                q_prev2 = pose_history[i - 2][j * 3]
                dq_prev = (q_prev - q_prev2) / 0.033
                tau = (dq - dq_prev) / 0.033
            else:
                tau = 0.0

            row.extend([q, dq, tau])
        rows.append(row)

    except Exception as e:
        print(f"Failed to process {file}: {e}")
        continue

# 保存CSV
df = pd.DataFrame(rows, columns=columns)
df.to_csv(output_csv_path, index=False)
print(f"\n✅ CSV saved to {output_csv_path}")