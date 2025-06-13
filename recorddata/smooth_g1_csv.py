import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
from datetime import datetime

# === 输入输出路径 ===
INPUT_CSV = "romp_output_g1.csv"
OUTPUT_CSV = f"romp_output_g1_interp_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# === 插值与平滑参数 ===
INTERP_FACTOR = 4      # 插值倍数（每帧变为 INTERP_FACTOR 帧）
SMOOTH_WINDOW = 9      # 滑动窗口大小（必须为奇数）
SMOOTH_POLY = 3        # 多项式阶数（savgol_filter用）

# === 加载数据 ===
df = pd.read_csv(INPUT_CSV)
time = df["time"].values
joint_names = sorted(set(c.rsplit("_", 1)[0] for c in df.columns if "_q" in c))

# === 插值时间轴 ===
interp_time = np.linspace(time[0], time[-1], len(time) * INTERP_FACTOR)

# === 插值和平滑处理每个关节 ===
rows = []
for joint in joint_names:
    q = df[f"{joint}_q"].values
    # 插值
    interp_fn = interp1d(time, q, kind='cubic')
    interp_q = interp_fn(interp_time)

    # 平滑
    if len(interp_q) >= SMOOTH_WINDOW:
        smooth_q = savgol_filter(interp_q, window_length=SMOOTH_WINDOW, polyorder=SMOOTH_POLY)
    else:
        smooth_q = interp_q

    # 计算速度（dq）+ 初始化力矩（tau=0）
    dq = np.gradient(smooth_q, np.mean(np.diff(interp_time)))
    tau = np.zeros_like(smooth_q)

    # 存储为列
    df_joint = pd.DataFrame({
        "time": interp_time,
        f"{joint}_q": smooth_q,
        f"{joint}_dq": dq,
        f"{joint}_tau": tau
    })

    rows.append(df_joint)

# === 合并为输出 CSV ===
final_df = rows[0][["time"]].copy()
for df_joint in rows:
    for col in df_joint.columns:
        if col != "time":
            final_df[col] = df_joint[col]

# === 保存 ===
final_df.to_csv(OUTPUT_CSV, index=False)
print(f"✅ 导出成功：{OUTPUT_CSV}，共 {len(final_df)} 帧")