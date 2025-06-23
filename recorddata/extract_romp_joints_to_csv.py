import os
import numpy as np
import pandas as pd
from tqdm import tqdm
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

# ==================== 配置选项 ====================
# 选择平滑级别: "conservative", "moderate", "aggressive"
SMOOTH_LEVEL = "conservative"  # 推荐使用保守模式获得最平滑的动作

# 根据平滑级别调整参数
if SMOOTH_LEVEL == "conservative":
    INTERP_RATIO = 2  # 更低的插值倍数
    SMOOTH_WINDOW = 31  # 更大的平滑窗口
    VELOCITY_SCALE = 0.7  # 速度缩放因子
elif SMOOTH_LEVEL == "moderate":
    INTERP_RATIO = 3
    SMOOTH_WINDOW = 21
    VELOCITY_SCALE = 0.85
else:  # aggressive
    INTERP_RATIO = 4
    SMOOTH_WINDOW = 15
    VELOCITY_SCALE = 1.0

ROMP_OUTPUT_DIR = "/home/ubuntu/unitree_g1_training/externals/ROMP/output"
OUTPUT_CSV = f"romp_output_g1_{SMOOTH_LEVEL}.csv"
FRAME_RATE = 30
OUTPUT_RATE = FRAME_RATE * INTERP_RATIO

print(f"🎛️  平滑级别: {SMOOTH_LEVEL}")
print(f"📊 配置参数: 插值倍数={INTERP_RATIO}, 平滑窗口={SMOOTH_WINDOW}, 速度缩放={VELOCITY_SCALE}")

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

# 关节速度限制 (rad/s) - 根据G1机器人实际能力调整
JOINT_VELOCITY_LIMITS = {
    "L_LEG_HIP_PITCH": 2.0, "L_LEG_HIP_ROLL": 2.0, "L_LEG_HIP_YAW": 2.0,
    "L_LEG_KNEE": 2.0, "L_LEG_ANKLE_PITCH": 2.0, "L_LEG_ANKLE_ROLL": 2.0,
    "R_LEG_HIP_PITCH": 2.0, "R_LEG_HIP_ROLL": 2.0, "R_LEG_HIP_YAW": 2.0,
    "R_LEG_KNEE": 2.0, "R_LEG_ANKLE_PITCH": 2.0, "R_LEG_ANKLE_ROLL": 2.0,
    "WAIST_YAW": 1.5,
    "joint_13": 1.0, "joint_14": 1.0,
    "L_SHOULDER_PITCH": 1.5, "L_SHOULDER_ROLL": 1.5, "L_SHOULDER_YAW": 1.5,
    "L_ELBOW": 1.5, "L_WRIST_ROLL": 1.0, "L_WRIST_PITCH": 1.0, "L_WRIST_YAW": 1.0,
    "R_SHOULDER_PITCH": 1.5, "R_SHOULDER_ROLL": 1.5, "R_SHOULDER_YAW": 1.5,
    "R_ELBOW": 1.5, "R_WRIST_ROLL": 1.0, "R_WRIST_PITCH": 1.0, "R_WRIST_YAW": 1.0
}

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

print(f"🔄 插值配置: {FRAME_RATE} FPS → {OUTPUT_RATE} FPS (倍数: {INTERP_RATIO})")

# 插值每个关节
interp_q = []
for j in range(pose_array.shape[1]):
    try:
        # 使用更平滑的插值方法
        interp_func = interp1d(original_times, pose_array[:, j], kind='cubic', 
                              fill_value="extrapolate", bounds_error=False)
        interp_q.append(interp_func(interp_times))
    except Exception as e:
        print(f"⚠️ 插值失败: 关节 {j} — {e}")
        interp_q.append(np.zeros_like(interp_times))  # fallback

# 转换为 NumPy 数组并转置为 (N_interp, 29)
interp_q = np.array(interp_q).T

# 增强平滑处理
def smooth_signal_advanced(data, window_size=SMOOTH_WINDOW, polyorder=3):
    """使用Savitzky-Golay滤波器进行高级平滑"""
    if len(data) < window_size:
        window_size = len(data) - 1 if len(data) > 1 else 1
    if window_size % 2 == 0:
        window_size -= 1
    if window_size < 3:
        return data
    
    try:
        return savgol_filter(data, window_size, polyorder)
    except:
        # fallback to simple moving average
        return np.convolve(data, np.ones(window_size) / window_size, mode='same')

def limit_velocity(positions, velocities, dt, max_velocities):
    """限制关节速度"""
    limited_velocities = np.copy(velocities)
    for i, max_vel in enumerate(max_velocities):
        # 应用速度缩放因子
        max_vel *= VELOCITY_SCALE
        # 限制速度大小
        vel_magnitude = np.abs(limited_velocities[:, i])
        scale_factor = np.minimum(1.0, max_vel / (vel_magnitude + 1e-8))
        limited_velocities[:, i] *= scale_factor
    
    # 根据限制后的速度重新计算位置
    limited_positions = np.copy(positions)
    for i in range(1, len(limited_positions)):
        limited_positions[i] = limited_positions[i-1] + limited_velocities[i] * dt
    
    return limited_positions, limited_velocities

print("🔄 应用高级平滑处理...")
# 对每个关节进行多级平滑
for j in range(interp_q.shape[1]):
    # 第一级：Savitzky-Golay平滑
    interp_q[:, j] = smooth_signal_advanced(interp_q[:, j], window_size=SMOOTH_WINDOW, polyorder=3)
    
    # 第二级：中值滤波去除异常值
    from scipy.signal import medfilt
    interp_q[:, j] = medfilt(interp_q[:, j], kernel_size=5)

# 计算速度
print("🔄 计算关节速度...")
dq_array = np.gradient(interp_q, 1.0 / OUTPUT_RATE, axis=0)

# 对速度也进行平滑
for j in range(dq_array.shape[1]):
    dq_array[:, j] = smooth_signal_advanced(dq_array[:, j], window_size=max(7, SMOOTH_WINDOW//3), polyorder=2)

# 应用速度限制
print("🔄 应用速度限制...")
max_velocities = [JOINT_VELOCITY_LIMITS[joint] for joint in G1_JOINT_LIST]
interp_q, dq_array = limit_velocity(interp_q, dq_array, 1.0 / OUTPUT_RATE, max_velocities)

# 计算加速度并平滑
print("🔄 计算并平滑加速度...")
ddq_array = np.gradient(dq_array, 1.0 / OUTPUT_RATE, axis=0)
for j in range(ddq_array.shape[1]):
    ddq_array[:, j] = smooth_signal_advanced(ddq_array[:, j], window_size=max(5, SMOOTH_WINDOW//4), polyorder=2)

# 简单的力矩计算（可以根据需要调整）
tau_array = np.zeros_like(dq_array)

# 写入 CSV
print("💾 写入CSV文件...")
rows = []
for i, t in enumerate(interp_times):
    q = interp_q[i]
    dq = dq_array[i]
    tau = tau_array[i]
    row = [t] + [v for triplet in zip(q, dq, tau) for v in triplet]
    rows.append(row)

df = pd.DataFrame(rows, columns=columns)
df.to_csv(OUTPUT_CSV, index=False)
print(f"✅ 平滑CSV文件生成完成: {OUTPUT_CSV}")
print(f"📊 数据统计:")
print(f"   - 总帧数: {len(df)}")
print(f"   - 时间跨度: {interp_times[-1]:.2f} 秒")
print(f"   - 输出频率: {OUTPUT_RATE} Hz")
print(f"   - 平均速度: {np.mean(np.abs(dq_array)):.3f} rad/s")
print(f"   - 最大速度: {np.max(np.abs(dq_array)):.3f} rad/s")

# 可选：保存速度限制后的数据用于分析
df_analysis = pd.DataFrame({
    'time': interp_times,
    'max_velocity': np.max(np.abs(dq_array), axis=1),
    'mean_velocity': np.mean(np.abs(dq_array), axis=1),
    'max_acceleration': np.max(np.abs(ddq_array), axis=1)
})
df_analysis.to_csv("velocity_analysis.csv", index=False)
print("📈 速度分析数据已保存到 velocity_analysis.csv")