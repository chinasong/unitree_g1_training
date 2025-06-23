import os
import numpy as np
import pandas as pd
from tqdm import tqdm
from scipy.interpolate import interp1d

ROMP_OUTPUT_DIR = "externals/ROMP/output"  # 替换为你的 ROMP 输出路径
OUTPUT_CSV = "romp_output_g1_upperbody.csv"
FRAME_RATE = 30
INTERP_RATIO = 6
OUTPUT_RATE = FRAME_RATE * INTERP_RATIO

# G1 上半身 16 个关节名称（13~28）
UPPER_JOINTS = [
    "WAIST_YAW",
    "joint_13", "joint_14",  # 占位，可改名
    "L_SHOULDER_PITCH", "L_SHOULDER_ROLL", "L_SHOULDER_YAW",
    "L_ELBOW", "L_WRIST_ROLL", "L_WRIST_PITCH", "L_WRIST_YAW",
    "R_SHOULDER_PITCH", "R_SHOULDER_ROLL", "R_SHOULDER_YAW",
    "R_ELBOW", "R_WRIST_ROLL", "R_WRIST_PITCH", "R_WRIST_YAW"
]

# 构建 CSV 表头
columns = ["time"]
for joint in UPPER_JOINTS:
    columns += [f"{joint}_q", f"{joint}_dq", f"{joint}_tau"]

# 加载 ROMP pose 数据
pose_list = []
npz_files = sorted(f for f in os.listdir(ROMP_OUTPUT_DIR) if f.endswith(".npz"))
print(f"📂 共找到 {len(npz_files)} 个 ROMP 输出帧")

for fname in tqdm(npz_files):
    fpath = os.path.join(ROMP_OUTPUT_DIR, fname)
    try:
        data = np.load(fpath, allow_pickle=True)["results"].item()
        global_orient = data.get("global_orient", np.zeros((1, 3))).reshape(-1)
        body_pose = data.get("body_pose", np.zeros((1, 69))).reshape(-1)
        full_pose = np.concatenate([global_orient, body_pose])  # shape (72,)

        # 取 Z 轴（索引 % 3 == 2），一共 24 个关节 * 3 = 72，取其中 Z轴
        z_rot = [full_pose[i] for i in range(2, 72, 3)]
        # 对应 G1 的关节索引是 13~28（共16个）
        upperbody_q = z_rot[13:29]
        pose_list.append(upperbody_q)
    except Exception as e:
        print(f"⚠️ 失败: {fname} — {e}")

pose_array = np.array(pose_list)
original_times = np.arange(len(pose_array)) / FRAME_RATE
interp_times = np.linspace(0, original_times[-1], len(pose_array) * INTERP_RATIO)

# 插值 + 平滑
interp_q = []
for j in range(pose_array.shape[1]):
    try:
        f = interp1d(original_times, pose_array[:, j], kind='cubic', fill_value="extrapolate")
        smoothed = np.convolve(f(interp_times), np.ones(7) / 7, mode="same")
        interp_q.append(smoothed)
    except Exception as e:
        print(f"⚠️ 插值失败: 关节 {j} — {e}")
        interp_q.append(np.zeros_like(interp_times))

interp_q = np.array(interp_q).T
dq_array = np.gradient(interp_q, 1.0 / OUTPUT_RATE, axis=0)
tau_array = np.zeros_like(dq_array)

# 写入 CSV
rows = []
for i, t in enumerate(interp_times):
    row = [t] + [v for triplet in zip(interp_q[i], dq_array[i], tau_array[i]) for v in triplet]
    rows.append(row) 

df = pd.DataFrame(rows, columns=columns)
df.to_csv(OUTPUT_CSV, index=False)
print(f"✅ 上半身 G1 CSV 文件生成完成: {OUTPUT_CSV}，共 {len(df)} 行")