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

# 生成最终CSV列名（每个关节q, dq, tau）
columns = ['time']
for idx in range(29):  # 0-28
    name = JOINT_MAPPING.get(idx, f"joint_{idx}")
    columns.extend([f"{name}_q", f"{name}_dq", f"{name}_tau"])

# 收集数据
rows = []

# 遍历 npz 文件
npz_files = sorted([f for f in os.listdir(input_folder) if f.endswith(".npz")])
for i, file in enumerate(tqdm(npz_files)):
    data = np.load(os.path.join(input_folder, file))
    pose = data['params']['body_pose'].reshape(-1)  # 23 * 3
    root_orient = data['params']['global_orient'].reshape(-1)  # 3
    full_pose = np.concatenate([root_orient, pose])  # 24 * 3

    frame_row = [i * 0.033]  # 假设帧率30fps，每帧约0.033秒

    for joint_id in range(29):
        if joint_id < len(full_pose) // 3:
            q = full_pose[joint_id * 3]
        else:
            q = 0.0  # 如果ROMP没有提供该关节，填0
        dq = 0.0
        tau = 0.0
        frame_row.extend([q, dq, tau])

    rows.append(frame_row)

# 保存为CSV
df = pd.DataFrame(rows, columns=columns)
df.to_csv(output_csv_path, index=False)
print(f"CSV saved to {output_csv_path}")