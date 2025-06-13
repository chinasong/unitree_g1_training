import os
import numpy as np
import pandas as pd
from tqdm import tqdm
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

ROMP_OUTPUT_DIR = "/home/ubuntu/unitree_g1_training/externals/ROMP/output"
OUTPUT_CSV = "romp_output_g1.csv"
FRAME_RATE = 30
INTERP_RATIO = 6
OUTPUT_RATE = FRAME_RATE * INTERP_RATIO

# G1 的全部 29 个关节（含顺序）
G1_JOINT_LIST = [
    "L_LEG_HIP_PITCH", "L_LEG_HIP_ROLL", "L_LEG_HIP_YAW",
    "L_LEG_KNEE", "L_LEG_ANKLE_PITCH", "L_LEG_ANKLE_ROLL",
    "R_LEG_HIP_PITCH", "R_LEG_HIP_ROLL", "R_LEG_HIP_YAW",
    "R_LEG_KNEE", "R_LEG_ANKLE_PITCH", "R_LEG_ANKLE_ROLL",
    "WAIST_YAW",
    "joint_13", "joint_14",
    "L_SHOULDER_PITCH", "L_SHOULDER_ROLL", "L_SHOULDER_YAW",
    "L_ELBOW", "L_WRIST_ROLL", "L_WRIST_PITCH", "L_WRIST_YAW",
    "R_SHOULDER_PITCH", "R_SHOULDER_ROLL", "R_SHOULDER_YAW",
    "R_ELBOW", "R_WRIST_ROLL", "R_WRIST_PITCH", "R_WRIST_YAW"
]

# 构建 CSV 表头
columns = ["time"]
for joint in G1_JOINT_LIST:
    columns += [f"{joint}_q", f"{joint}_dq", f"{joint}_tau"]

# 加载 pose 数据
pose_list = []
npz_files = sorted(f for f in os.listdir(ROMP_OUTPUT_DIR) if f.endswith(".npz"))
print(f"📂 共找到 {len(npz_files)} 个 ROMP 输出帧")

for fname in tqdm(npz_files):
    fpath = os.path.join(ROMP_OUTPUT_DIR, fname)
    try:
        data = np.load(fpath, allow_pickle=True)["results"].item()
        global_orient = data.get("global_orient", np.zeros((1, 3))).reshape(-1)
        body_pose = data.get("body_pose", np.zeros((1, 69))).reshape(-1)
        full_pose = np.concatenate([global_orient, body_pose])  # (72,)
        # 取每个关节的 Z 轴旋转（索引 % 3 == 2）
        q_frame = [full_pose[i] if i < 72 else 0.0 for i in range(2, 72, 3)]
        # 如果不够 29 个关节，填充 0
        q_frame += [0.0] * (29 - len(q_frame))
        pose_list.append(q_frame[:29])
    except Exception as e:
        print(f"⚠️ 失败: {fname} — {e}")

if not pose_list:
    print("❌ 没有成功加载任何 ROMP pose 数据.")
    exit(1)

pose_array = np.array(pose_list)
original_times = np.arange(len(pose_array)) / FRAME_RATE
interp_times = np.linspace(0, original_times[-1], len(pose_array) * INTERP_RATIO)

# 插值
interp_q = []
# 滑动平均函数
def smooth_signal(data, window_size=7):
    return np.convolve(data, np.ones(window_size) / window_size, mode='same')

for j in range(pose_array.shape[1]):
    interp_q[:, j] = smooth_signal(interp_q[:, j], window_size=7)

# 重新计算速度
dq_array = np.gradient(interp_q, 1.0 / OUTPUT_RATE, axis=0)

# 速度估算（差分除以 dt）
dq_array = np.gradient(interp_q, 1.0 / OUTPUT_RATE, axis=0)
tau_array = np.zeros_like(dq_array)

# 写入 CSV
rows = []
for i, t in enumerate(interp_times):
    q = interp_q[i]
    dq = dq_array[i]
    tau = tau_array[i]
    row = [t] + [v for triplet in zip(q, dq, tau) for v in triplet]
    rows.append(row)

df = pd.DataFrame(rows, columns=columns)
df.to_csv(OUTPUT_CSV, index=False)
print(f"✅ CSV 文件生成完成: {OUTPUT_CSV}，共 {len(df)} 行")

# 可视化关键关节
for joint in ["L_ELBOW", "R_ELBOW", "L_SHOULDER_PITCH", "R_SHOULDER_PITCH"]:
    if f"{joint}_q" in df.columns:
        plt.figure(figsize=(10, 3))
        plt.plot(df["time"], df[f"{joint}_q"], label=f"{joint} 角度")
        plt.title(f"{joint} 角度变化")
        plt.xlabel("时间（秒）")
        plt.ylabel("角度（弧度）")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(f"{joint}_plot.png")
        plt.show()