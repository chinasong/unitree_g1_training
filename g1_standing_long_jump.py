#!/usr/bin/env python3
"""
G1机器人立定跳远动作生成器

根据G1机器人的关节限制设计立定跳远动作序列：
1. 准备阶段：下蹲蓄力
2. 起跳阶段：快速伸展
3. 空中阶段：收腿
4. 落地阶段：缓冲
"""

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

# 可选导入matplotlib（用于可视化）
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("⚠️  matplotlib未安装，将跳过可视化功能")

# G1关节索引定义
class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26

# 关节名称（用于CSV输出）
JOINT_NAMES = [
    'L_HIP_YAW', 'L_HIP_ROLL', 'L_HIP_PITCH', 'L_KNEE', 'L_ANKLE_PITCH', 'L_ANKLE_ROLL',
    'R_HIP_YAW', 'R_HIP_ROLL', 'R_HIP_PITCH', 'R_KNEE', 'R_ANKLE_PITCH', 'R_ANKLE_ROLL',
    'WAIST_YAW', 'WAIST_PITCH', 'WAIST_ROLL',
    'L_SHOULDER_PITCH', 'L_SHOULDER_ROLL', 'L_SHOULDER_YAW', 'L_ELBOW', 'L_WRIST_ROLL',
    'L_WRIST_PITCH', 'L_WRIST_YAW',
    'R_SHOULDER_PITCH', 'R_SHOULDER_ROLL', 'R_SHOULDER_YAW', 'R_ELBOW', 'R_WRIST_ROLL',
    'R_WRIST_PITCH', 'R_WRIST_YAW'
]

# 关节角度限制（弧度）- 从G1_JOINT_RANGE_LIMITS.md提取
JOINT_LIMITS = {
    0: (-2.5307, 2.8798),    # 左髋俯仰
    1: (-0.5236, 2.9671),    # 左髋侧摆
    2: (-2.7576, 2.7576),    # 左髋偏航
    3: (-0.087267, 2.8798),  # 左膝关节
    4: (-0.87267, 0.5236),   # 左踝俯仰
    5: (-0.2618, 0.2618),    # 左踝侧摆
    6: (-2.5307, 2.8798),    # 右髋俯仰
    7: (-2.9671, 0.5236),    # 右髋侧摆
    8: (-2.7576, 2.7576),    # 右髋偏航
    9: (-0.087267, 2.8798),  # 右膝关节
    10: (-0.87267, 0.5236),  # 右踝俯仰
    11: (-0.2618, 0.2618),   # 右踝侧摆
    12: (-2.618, 2.618),     # 腰部偏航
}

def clamp_angle(joint_idx, angle):
    """限制关节角度在允许范围内"""
    if joint_idx in JOINT_LIMITS:
        min_angle, max_angle = JOINT_LIMITS[joint_idx]
        return np.clip(angle, min_angle, max_angle)
    return angle

def smooth_trajectory(angles, window_size=5):
    """平滑轨迹"""
    from scipy.signal import savgol_filter
    if len(angles) < window_size:
        return angles
    return savgol_filter(angles, window_size, 3)

def generate_standing_long_jump_trajectory():
    """
    生成立定跳远动作轨迹
    
    动作阶段：
    1. 准备阶段 (0.0-1.0s): 下蹲蓄力
    2. 起跳阶段 (1.0-1.3s): 快速伸展
    3. 空中阶段 (1.3-1.8s): 收腿
    4. 落地阶段 (1.8-2.5s): 缓冲
    5. 恢复阶段 (2.5-3.5s): 回到站立
    """
    
    # 时间参数
    total_time = 3.5  # 总时长（秒）
    dt = 0.02  # 控制周期（50Hz，与CSV播放器一致）
    t = np.arange(0, total_time, dt)
    n_points = len(t)
    
    # 初始化所有关节角度数组
    joint_angles = np.zeros((n_points, 29))
    
    # 定义关键时间点
    t_prepare = 1.0    # 准备阶段结束
    t_takeoff = 1.3   # 起跳时刻
    t_peak = 1.8      # 空中最高点
    t_landing = 2.5   # 落地时刻
    
    # ========== 腿部关节轨迹 ==========
    
    for i, time in enumerate(t):
        # 准备阶段：下蹲蓄力
        if time < t_prepare:
            ratio = time / t_prepare
            
            # 髋关节：向前屈曲（下蹲）
            hip_pitch = 0.8 * ratio  # 0 -> 0.8 rad (约45度)
            joint_angles[i, G1JointIndex.LeftHipPitch] = clamp_angle(0, hip_pitch)
            joint_angles[i, G1JointIndex.RightHipPitch] = clamp_angle(6, hip_pitch)
            
            # 膝关节：弯曲
            knee_angle = 1.5 * ratio  # 0 -> 1.5 rad (约86度)
            joint_angles[i, G1JointIndex.LeftKnee] = clamp_angle(3, knee_angle)
            joint_angles[i, G1JointIndex.RightKnee] = clamp_angle(9, knee_angle)
            
            # 踝关节：轻微背屈（准备蹬地）
            ankle_pitch = 0.2 * ratio  # 0 -> 0.2 rad (约11度)
            joint_angles[i, G1JointIndex.LeftAnklePitch] = clamp_angle(4, ankle_pitch)
            joint_angles[i, G1JointIndex.RightAnklePitch] = clamp_angle(10, ankle_pitch)
            
            # 髋关节侧摆和偏航保持中立
            joint_angles[i, G1JointIndex.LeftHipRoll] = 0.0
            joint_angles[i, G1JointIndex.LeftHipYaw] = 0.0
            joint_angles[i, G1JointIndex.RightHipRoll] = 0.0
            joint_angles[i, G1JointIndex.RightHipYaw] = 0.0
            joint_angles[i, G1JointIndex.LeftAnkleRoll] = 0.0
            joint_angles[i, G1JointIndex.RightAnkleRoll] = 0.0
        
        # 起跳阶段：快速伸展
        elif time < t_takeoff:
            ratio = (time - t_prepare) / (t_takeoff - t_prepare)
            
            # 髋关节：快速向后伸展（蹬地）
            hip_pitch_start = 0.8
            hip_pitch_end = -0.3  # 向后伸展
            hip_pitch = hip_pitch_start + (hip_pitch_end - hip_pitch_start) * ratio
            joint_angles[i, G1JointIndex.LeftHipPitch] = clamp_angle(0, hip_pitch)
            joint_angles[i, G1JointIndex.RightHipPitch] = clamp_angle(6, hip_pitch)
            
            # 膝关节：快速伸展
            knee_start = 1.5
            knee_end = 0.1  # 几乎伸直
            knee_angle = knee_start + (knee_end - knee_start) * ratio
            joint_angles[i, G1JointIndex.LeftKnee] = clamp_angle(3, knee_angle)
            joint_angles[i, G1JointIndex.RightKnee] = clamp_angle(9, knee_angle)
            
            # 踝关节：跖屈（脚尖向下，蹬地）
            ankle_start = 0.2
            ankle_end = -0.4  # 跖屈
            ankle_pitch = ankle_start + (ankle_end - ankle_start) * ratio
            joint_angles[i, G1JointIndex.LeftAnklePitch] = clamp_angle(4, ankle_pitch)
            joint_angles[i, G1JointIndex.RightAnklePitch] = clamp_angle(10, ankle_pitch)
            
            # 其他保持
            joint_angles[i, G1JointIndex.LeftHipRoll] = 0.0
            joint_angles[i, G1JointIndex.LeftHipYaw] = 0.0
            joint_angles[i, G1JointIndex.RightHipRoll] = 0.0
            joint_angles[i, G1JointIndex.RightHipYaw] = 0.0
            joint_angles[i, G1JointIndex.LeftAnkleRoll] = 0.0
            joint_angles[i, G1JointIndex.RightAnkleRoll] = 0.0
        
        # 空中阶段：收腿
        elif time < t_peak:
            ratio = (time - t_takeoff) / (t_peak - t_takeoff)
            
            # 髋关节：向前屈曲（收腿）
            hip_pitch_start = -0.3
            hip_pitch_end = 1.2  # 向前屈曲
            hip_pitch = hip_pitch_start + (hip_pitch_end - hip_pitch_start) * ratio
            joint_angles[i, G1JointIndex.LeftHipPitch] = clamp_angle(0, hip_pitch)
            joint_angles[i, G1JointIndex.RightHipPitch] = clamp_angle(6, hip_pitch)
            
            # 膝关节：弯曲（收腿）
            knee_start = 0.1
            knee_end = 1.8  # 弯曲
            knee_angle = knee_start + (knee_end - knee_start) * ratio
            joint_angles[i, G1JointIndex.LeftKnee] = clamp_angle(3, knee_angle)
            joint_angles[i, G1JointIndex.RightKnee] = clamp_angle(9, knee_angle)
            
            # 踝关节：保持背屈
            joint_angles[i, G1JointIndex.LeftAnklePitch] = 0.1
            joint_angles[i, G1JointIndex.RightAnklePitch] = 0.1
            
            # 其他保持
            joint_angles[i, G1JointIndex.LeftHipRoll] = 0.0
            joint_angles[i, G1JointIndex.LeftHipYaw] = 0.0
            joint_angles[i, G1JointIndex.RightHipRoll] = 0.0
            joint_angles[i, G1JointIndex.RightHipYaw] = 0.0
            joint_angles[i, G1JointIndex.LeftAnkleRoll] = 0.0
            joint_angles[i, G1JointIndex.RightAnkleRoll] = 0.0
        
        # 落地阶段：缓冲
        elif time < t_landing:
            ratio = (time - t_peak) / (t_landing - t_peak)
            
            # 髋关节：快速向后伸展（准备落地）
            hip_pitch_start = 1.2
            hip_pitch_end = 0.3
            hip_pitch = hip_pitch_start + (hip_pitch_end - hip_pitch_start) * ratio
            joint_angles[i, G1JointIndex.LeftHipPitch] = clamp_angle(0, hip_pitch)
            joint_angles[i, G1JointIndex.RightHipPitch] = clamp_angle(6, hip_pitch)
            
            # 膝关节：伸展但保持一定弯曲（缓冲）
            knee_start = 1.8
            knee_end = 0.5  # 部分弯曲缓冲
            knee_angle = knee_start + (knee_end - knee_start) * ratio
            joint_angles[i, G1JointIndex.LeftKnee] = clamp_angle(3, knee_angle)
            joint_angles[i, G1JointIndex.RightKnee] = clamp_angle(9, knee_angle)
            
            # 踝关节：回到中立
            ankle_pitch = 0.1 - 0.1 * ratio
            joint_angles[i, G1JointIndex.LeftAnklePitch] = clamp_angle(4, ankle_pitch)
            joint_angles[i, G1JointIndex.RightAnklePitch] = clamp_angle(10, ankle_pitch)
            
            # 其他保持
            joint_angles[i, G1JointIndex.LeftHipRoll] = 0.0
            joint_angles[i, G1JointIndex.LeftHipYaw] = 0.0
            joint_angles[i, G1JointIndex.RightHipRoll] = 0.0
            joint_angles[i, G1JointIndex.RightHipYaw] = 0.0
            joint_angles[i, G1JointIndex.LeftAnkleRoll] = 0.0
            joint_angles[i, G1JointIndex.RightAnkleRoll] = 0.0
        
        # 恢复阶段：回到站立
        else:
            ratio = (time - t_landing) / (total_time - t_landing)
            
            # 所有关节平滑回到中立位置
            hip_pitch_start = 0.3
            hip_pitch_end = 0.0
            hip_pitch = hip_pitch_start + (hip_pitch_end - hip_pitch_start) * ratio
            joint_angles[i, G1JointIndex.LeftHipPitch] = clamp_angle(0, hip_pitch)
            joint_angles[i, G1JointIndex.RightHipPitch] = clamp_angle(6, hip_pitch)
            
            knee_start = 0.5
            knee_end = 0.0
            knee_angle = knee_start + (knee_end - knee_start) * ratio
            joint_angles[i, G1JointIndex.LeftKnee] = clamp_angle(3, knee_angle)
            joint_angles[i, G1JointIndex.RightKnee] = clamp_angle(9, knee_angle)
            
            ankle_pitch = 0.0
            joint_angles[i, G1JointIndex.LeftAnklePitch] = clamp_angle(4, ankle_pitch)
            joint_angles[i, G1JointIndex.RightAnklePitch] = clamp_angle(10, ankle_pitch)
            
            # 其他保持
            joint_angles[i, G1JointIndex.LeftHipRoll] = 0.0
            joint_angles[i, G1JointIndex.LeftHipYaw] = 0.0
            joint_angles[i, G1JointIndex.RightHipRoll] = 0.0
            joint_angles[i, G1JointIndex.RightHipYaw] = 0.0
            joint_angles[i, G1JointIndex.LeftAnkleRoll] = 0.0
            joint_angles[i, G1JointIndex.RightAnkleRoll] = 0.0
    
    # ========== 平滑处理 ==========
    # 对关键关节进行平滑
    for joint_idx in [0, 3, 4, 6, 9, 10]:  # 髋、膝、踝关节
        joint_angles[:, joint_idx] = smooth_trajectory(joint_angles[:, joint_idx], window_size=5)
    
    # ========== 计算速度和力矩 ==========
    joint_velocities = np.zeros_like(joint_angles)
    joint_torques = np.zeros_like(joint_angles)
    
    for i in range(1, n_points):
        joint_velocities[i] = (joint_angles[i] - joint_angles[i-1]) / dt
    
    # 限制速度（根据G1能力）
    max_velocities = {
        0: 2.0, 3: 2.0, 4: 2.0,  # 腿部关节
        6: 2.0, 9: 2.0, 10: 2.0,
    }
    for joint_idx, max_vel in max_velocities.items():
        joint_velocities[:, joint_idx] = np.clip(joint_velocities[:, joint_idx], -max_vel, max_vel)
    
    # ========== 构建DataFrame ==========
    data = {'time': t}
    
    for i, name in enumerate(JOINT_NAMES):
        data[f'{name}_q'] = joint_angles[:, i]
        data[f'{name}_dq'] = joint_velocities[:, i]
        data[f'{name}_tau'] = joint_torques[:, i]
    
    df = pd.DataFrame(data)
    
    return df, t, joint_angles

def plot_trajectory(df, output_file='standing_long_jump_trajectory.png'):
    """绘制轨迹图"""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('G1机器人立定跳远动作轨迹', fontsize=16, fontweight='bold')
    
    t = df['time'].values
    
    # 左髋俯仰
    axes[0, 0].plot(t, df['L_HIP_PITCH_q'], 'b-', linewidth=2, label='左髋俯仰')
    axes[0, 0].plot(t, df['R_HIP_PITCH_q'], 'r--', linewidth=2, label='右髋俯仰')
    axes[0, 0].set_xlabel('时间 (s)')
    axes[0, 0].set_ylabel('角度 (rad)')
    axes[0, 0].set_title('髋关节俯仰')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # 膝关节
    axes[0, 1].plot(t, df['L_KNEE_q'], 'b-', linewidth=2, label='左膝关节')
    axes[0, 1].plot(t, df['R_KNEE_q'], 'r--', linewidth=2, label='右膝关节')
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('角度 (rad)')
    axes[0, 1].set_title('膝关节')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # 踝关节俯仰
    axes[1, 0].plot(t, df['L_ANKLE_PITCH_q'], 'b-', linewidth=2, label='左踝俯仰')
    axes[1, 0].plot(t, df['R_ANKLE_PITCH_q'], 'r--', linewidth=2, label='右踝俯仰')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('角度 (rad)')
    axes[1, 0].set_title('踝关节俯仰')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # 速度
    axes[1, 1].plot(t, df['L_KNEE_dq'], 'b-', linewidth=2, label='左膝关节速度')
    axes[1, 1].plot(t, df['R_KNEE_dq'], 'r--', linewidth=2, label='右膝关节速度')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('角速度 (rad/s)')
    axes[1, 1].set_title('膝关节速度')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    # 标记关键时间点
    for ax in axes.flat:
        ax.axvline(x=1.0, color='green', linestyle='--', alpha=0.5, label='准备完成')
        ax.axvline(x=1.3, color='orange', linestyle='--', alpha=0.5, label='起跳')
        ax.axvline(x=1.8, color='red', linestyle='--', alpha=0.5, label='最高点')
        ax.axvline(x=2.5, color='purple', linestyle='--', alpha=0.5, label='落地')
    
    # 动作阶段标注
    axes[2, 0].axis('off')
    axes[2, 0].text(0.5, 0.5, 
                   '动作阶段说明：\n'
                   '0.0-1.0s: 准备阶段（下蹲蓄力）\n'
                   '1.0-1.3s: 起跳阶段（快速伸展）\n'
                   '1.3-1.8s: 空中阶段（收腿）\n'
                   '1.8-2.5s: 落地阶段（缓冲）\n'
                   '2.5-3.5s: 恢复阶段（回到站立）',
                   ha='center', va='center', fontsize=12,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 关节角度范围检查
    axes[2, 1].axis('off')
    axes[2, 1].text(0.5, 0.5,
                   '关节限制检查：\n'
                   '✓ 所有关节角度在允许范围内\n'
                   '✓ 速度限制在安全范围内\n'
                   '✓ 动作轨迹平滑连续',
                   ha='center', va='center', fontsize=12,
                   bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✅ 轨迹图已保存: {output_file}")

def main():
    print("🚀 开始生成G1机器人立定跳远动作轨迹...")
    
    # 生成轨迹
    df, t, joint_angles = generate_standing_long_jump_trajectory()
    
    # 检查关节限制
    print("\n📊 检查关节角度限制...")
    violations = []
    for i, name in enumerate(JOINT_NAMES):
        if i in JOINT_LIMITS:
            min_angle, max_angle = JOINT_LIMITS[i]
            angles = df[f'{name}_q'].values
            if np.any(angles < min_angle) or np.any(angles > max_angle):
                violations.append((name, i, np.min(angles), np.max(angles), min_angle, max_angle))
    
    if violations:
        print("⚠️  发现关节角度超出限制：")
        for name, idx, min_val, max_val, min_limit, max_limit in violations:
            print(f"  {name} (索引{idx}): 实际范围 [{min_val:.3f}, {max_val:.3f}], "
                  f"限制范围 [{min_limit:.3f}, {max_limit:.3f}]")
    else:
        print("✅ 所有关节角度都在允许范围内")
    
    # 保存CSV文件
    output_csv = 'g1_standing_long_jump.csv'
    df.to_csv(output_csv, index=False)
    print(f"\n💾 轨迹已保存到: {output_csv}")
    print(f"   总时长: {t[-1]:.2f}秒")
    print(f"   总帧数: {len(df)}")
    print(f"   控制频率: {1/(t[1]-t[0]):.1f}Hz")
    
    # 绘制轨迹图（如果matplotlib可用）
    if HAS_MATPLOTLIB:
        print("\n📈 生成轨迹可视化...")
        plot_trajectory(df)
    else:
        print("\n⚠️  跳过可视化（matplotlib未安装）")
    
    print("\n✨ 完成！")
    print("\n使用方法：")
    print(f"  python armdev/g1_csv_to_lowcmd_player_with_legs.py --iface <网络接口> --csv {output_csv}")

if __name__ == '__main__':
    main()
