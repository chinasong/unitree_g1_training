import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import imageio
import os
from tqdm import tqdm

# 路径设定
CSV_PATH = "romp_output_g1.csv"
GIF_PATH = "g1_motion.gif"
FRAME_DIR = "gif_frames"
os.makedirs(FRAME_DIR, exist_ok=True)

# 骨架连线（近似 G1 结构）
JOINT_CONNECTIONS = [
    ("L_SHOULDER_PITCH", "L_ELBOW"),
    ("L_ELBOW", "L_WRIST_ROLL"),
    ("R_SHOULDER_PITCH", "R_ELBOW"),
    ("R_ELBOW", "R_WRIST_ROLL"),
    ("L_LEG_HIP_PITCH", "L_LEG_KNEE"),
    ("L_LEG_KNEE", "L_LEG_ANKLE_PITCH"),
    ("R_LEG_HIP_PITCH", "R_LEG_KNEE"),
    ("R_LEG_KNEE", "R_LEG_ANKLE_PITCH"),
    ("WAIST_YAW", "L_SHOULDER_PITCH"),
    ("WAIST_YAW", "R_SHOULDER_PITCH"),
    ("WAIST_YAW", "L_LEG_HIP_PITCH"),
    ("WAIST_YAW", "R_LEG_HIP_PITCH")
]

# 读取角度数据
df = pd.read_csv(CSV_PATH)
angle_columns = [col for col in df.columns if col.endswith("_q")]
angles = df[angle_columns].values
joint_names = [col.replace("_q", "") for col in angle_columns]
joint_name_to_index = {name: i for i, name in enumerate(joint_names)}

# 简化2D可视化位置
def compute_joint_positions(joint_angles):
    coords = []
    for i, angle in enumerate(joint_angles):
        x = i % 10 + np.cos(angle)
        y = i // 10 + np.sin(angle)
        coords.append((x, y))
    return np.array(coords)

# 生成帧图
image_paths = []
for i, q in tqdm(enumerate(angles), total=len(angles)):
    coords = compute_joint_positions(q)
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-5, 15)
    ax.set_ylim(-2, 10)
    ax.axis('off')

    for j1, j2 in JOINT_CONNECTIONS:
        if j1 in joint_name_to_index and j2 in joint_name_to_index:
            i1, i2 = joint_name_to_index[j1], joint_name_to_index[j2]
            x1, y1 = coords[i1]
            x2, y2 = coords[i2]
            ax.plot([x1, x2], [y1, y2], 'k-', lw=2)

    xs, ys = coords[:, 0], coords[:, 1]
    ax.scatter(xs, ys, c='blue', s=20)

    frame_path = os.path.join(FRAME_DIR, f"frame_{i:04d}.png")
    plt.savefig(frame_path)
    plt.close(fig)
    image_paths.append(frame_path)

# 合成为GIF
frames = [imageio.imread(p) for p in image_paths]
imageio.mimsave(GIF_PATH, frames, fps=20)
print(f"✅ GIF 动画保存为: {GIF_PATH}")