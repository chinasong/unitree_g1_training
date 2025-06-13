import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

csv_path = "romp_output_g1.csv"  # 替换为你的文件路径
df = pd.read_csv(csv_path)

# 提取关节名称
joint_names = [col[:-2] for col in df.columns if col.endswith("_q")]
joint_names = sorted(set(joint_names))
time = df["time"].values
joint_angles = np.array([df[f"{j}_q"].values for j in joint_names])

# 创建动画
fig, ax = plt.subplots(figsize=(12, 6))
bars = ax.bar(joint_names, joint_angles[:, 0])
ax.set_ylim(-2.5, 2.5)
ax.set_ylabel("Joint Angle (rad)")
ax.set_title("G1 Robot Joint Angles Over Time")
plt.xticks(rotation=90)

def update(frame):
    for bar, height in zip(bars, joint_angles[:, frame]):
        bar.set_height(height)
    ax.set_title(f"G1 Robot Joint Angles (Time: {time[frame]:.2f}s)")
    return bars

ani = animation.FuncAnimation(fig, update, frames=len(time), interval=50, blit=False)
ani.save("g1_joint_angles_animation.gif", writer="pillow", fps=20)