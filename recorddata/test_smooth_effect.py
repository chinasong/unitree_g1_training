#!/usr/bin/env python3
"""
测试不同平滑级别的效果
"""

import subprocess
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def run_extraction(smooth_level):
    """运行数据提取程序"""
    print(f"\n🔄 测试 {smooth_level} 平滑级别...")
    
    # 修改配置文件中的平滑级别
    with open('extract_romp_joints_to_csv.py', 'r') as f:
        content = f.read()
    
    # 替换平滑级别
    content = content.replace(f'SMOOTH_LEVEL = "conservative"', f'SMOOTH_LEVEL = "{smooth_level}"')
    content = content.replace(f'SMOOTH_LEVEL = "moderate"', f'SMOOTH_LEVEL = "{smooth_level}"')
    content = content.replace(f'SMOOTH_LEVEL = "aggressive"', f'SMOOTH_LEVEL = "{smooth_level}"')
    
    with open('extract_romp_joints_to_csv.py', 'w') as f:
        f.write(content)
    
    # 运行程序
    try:
        result = subprocess.run(['python', 'extract_romp_joints_to_csv.py'], 
                              capture_output=True, text=True, timeout=300)
        if result.returncode == 0:
            print(f"✅ {smooth_level} 级别处理完成")
            return True
        else:
            print(f"❌ {smooth_level} 级别处理失败: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print(f"⏰ {smooth_level} 级别处理超时")
        return False

def analyze_results():
    """分析不同级别的结果"""
    levels = ["conservative", "moderate", "aggressive"]
    results = {}
    
    for level in levels:
        csv_file = f"romp_output_g1_{level}.csv"
        if os.path.exists(csv_file):
            df = pd.read_csv(csv_file)
            
            # 计算统计信息
            dq_columns = [col for col in df.columns if col.endswith('_dq')]
            velocities = df[dq_columns].values
            
            results[level] = {
                'max_velocity': np.max(np.abs(velocities)),
                'mean_velocity': np.mean(np.abs(velocities)),
                'std_velocity': np.std(np.abs(velocities)),
                'total_frames': len(df),
                'duration': df['time'].iloc[-1] - df['time'].iloc[0]
            }
    
    return results

def plot_comparison(results):
    """绘制比较图表"""
    if not results:
        print("❌ 没有数据可以比较")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('不同平滑级别的效果比较', fontsize=16)
    
    levels = list(results.keys())
    metrics = ['max_velocity', 'mean_velocity', 'std_velocity', 'duration']
    titles = ['最大速度 (rad/s)', '平均速度 (rad/s)', '速度标准差 (rad/s)', '持续时间 (s)']
    
    for i, (metric, title) in enumerate(zip(metrics, titles)):
        ax = axes[i//2, i%2]
        values = [results[level][metric] for level in levels]
        bars = ax.bar(levels, values, color=['#2E8B57', '#FF8C00', '#DC143C'])
        ax.set_title(title)
        ax.set_ylabel('数值')
        
        # 在柱状图上添加数值标签
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                   f'{value:.3f}', ha='center', va='bottom')
    
    plt.tight_layout()
    plt.savefig('smooth_comparison.png', dpi=300, bbox_inches='tight')
    print("📊 比较图表已保存为 smooth_comparison.png")
    plt.show()

def main():
    print("🧪 开始测试不同平滑级别的效果...")
    
    # 测试不同级别
    levels = ["conservative", "moderate", "aggressive"]
    success_count = 0
    
    for level in levels:
        if run_extraction(level):
            success_count += 1
    
    print(f"\n📈 成功处理 {success_count}/{len(levels)} 个级别")
    
    # 分析结果
    if success_count > 0:
        results = analyze_results()
        print("\n📊 结果分析:")
        for level, stats in results.items():
            print(f"\n{level.upper()} 级别:")
            print(f"  最大速度: {stats['max_velocity']:.3f} rad/s")
            print(f"  平均速度: {stats['mean_velocity']:.3f} rad/s")
            print(f"  速度标准差: {stats['std_velocity']:.3f} rad/s")
            print(f"  总帧数: {stats['total_frames']}")
            print(f"  持续时间: {stats['duration']:.2f} 秒")
        
        # 绘制比较图表
        plot_comparison(results)
        
        # 推荐最佳级别
        if len(results) > 1:
            best_level = min(results.keys(), key=lambda x: results[x]['max_velocity'])
            print(f"\n🎯 推荐使用 {best_level} 级别 (最低最大速度)")
    else:
        print("❌ 没有成功处理任何级别")

if __name__ == "__main__":
    main() 