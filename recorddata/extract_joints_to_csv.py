import os
import numpy as np
import pandas as pd
from tqdm import tqdm

# ROMP输出目录
input_folder = "../externals/ROMP/output"
# 输出CSV路径
output_csv_path = "romp_output_g1.csv"

# G1关节编号对应映射
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

# 构建列名
columns = ['time']
for idx in range(29):  # 0-28
    name = JOINT_MAPPING.get(idx, f"joint_{idx}")
    columns.extend([f"{name}_q", f"{name}_dq", f"{name}_tau"])

rows = []

npz_files = sorted([f for f in os.listdir(input_folder) if f.endswith(".npz")])
print(f"Total npz files found: {len(npz_files)}")

for i, file in enumerate(tqdm(npz_files)):
    path = os.path.join(input_folder, file)
    print(f"\nProcessing {path} ...")

    try:
        data = np.load(path)
        print("Keys in npz:", list(data.keys()))

        # 尝试提取pose数据
        body_pose = data['body_pose']
        root_orient = data['global_orient']

        print("body_pose shape:", body_pose.shape)
        print("global_orient shape:", root_orient.shape)

        pose = body_pose.reshape(-1)
        root_orient = root_orient.reshape(-1)
        full_pose = np.concatenate([root_orient, pose])  # 24*3 = 72

        frame_row = [i * 0.033]

        for joint_id in range(29):
            if joint_id * 3 + 2 < len(full_pose):
                q = full_pose[joint_id * 3]
            else:
                q = 0.0
            dq = 0.0
            tau = 0.0
            frame_row.extend([q, dq, tau])

        rows.append(frame_row)

    except Exception as e:
        print(f"Failed to process {file}: {e}")
        continue

# 保存CSV
df = pd.DataFrame(rows, columns=columns)
df.to_csv(output_csv_path, index=False)
print(f"\nCSV saved to {output_csv_path}")