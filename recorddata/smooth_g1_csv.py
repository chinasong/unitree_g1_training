import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import datetime
import os

INPUT_CSV = "romp_output_g1.csv"
INTERP_RATIO = 6  # 每帧插值倍数
FRAME_RATE = 30
OUTPUT_RATE = FRAME_RATE * INTERP_RATIO
WINDOW_SIZE = 7  # 滑动平均窗口

def smooth_signal(signal, window_size=7):
    window = np.ones(window_size) / window_size
    return np.convolve(signal, window, mode='same')

# 加载原始 CSV
df = pd.read_csv(INPUT_CSV)
time = df["time"].values
q_columns = [col for col in df.columns if col.endswith("_q")]

# 构建插值时间序列
interp_time = np.linspace(time[0], time[-1], len(time) * INTERP_RATIO)

# 处理每个关节角度
interp_q = {}
for col in q_columns:
    interp_func = interp1d(time, df[col].values, kind='cubic', fill_value="extrapolate")
    interp_vals = interp_func(interp_time)
    smoothed = smooth_signal(interp_vals, window_size=WINDOW_SIZE)
    interp_q[col] = smoothed

# 计算速度 dq
dq = {
    col.replace("_q", "_dq"): np.gradient(vals, 1.0 / OUTPUT_RATE)
    for col, vals in interp_q.items()
}

# tau 全部设为 0
tau = {
    col.replace("_q", "_tau"): np.zeros_like(vals)
    for col, vals in interp_q.items()
}

# 构建输出 DataFrame
output = pd.DataFrame({"time": interp_time})
for col in q_columns:
    output[col] = interp_q[col]
    output[col.replace("_q", "_dq")] = dq[col.replace("_q", "_dq")]
    output[col.replace("_q", "_tau")] = tau[col.replace("_q", "_tau")]

# 保存输出
now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
output_name = f"romp_output_g1_interp_{now}.csv"
output.to_csv(output_name, index=False)
print(f"✅ 插值+平滑后的数据已保存为 {output_name}，共 {len(output)} 帧")