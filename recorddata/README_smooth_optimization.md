# G1机器人动作平滑优化

## 问题描述
原始的ROMP数据转换程序生成的G1机器人动作过快且不够平滑，导致机器人运动不自然。

## 优化方案

### 1. 多级平滑处理
- **Savitzky-Golay滤波器**：保持信号特征的同时去除高频噪声
- **中值滤波**：去除异常值
- **速度平滑**：对关节角速度进行额外平滑
- **加速度平滑**：确保加速度变化平滑

### 2. 速度限制
- 为每个关节设置合理的速度上限
- 腿部关节：2.0 rad/s
- 腰部关节：1.5 rad/s  
- 手臂关节：1.0-1.5 rad/s
- 手腕关节：1.0 rad/s

### 3. 可配置的平滑级别

#### Conservative（保守模式）- 推荐
- 插值倍数：2x (30FPS → 60FPS)
- 平滑窗口：31点
- 速度缩放：70%
- 特点：最平滑，动作最慢，适合精细控制

#### Moderate（中等模式）
- 插值倍数：3x (30FPS → 90FPS)
- 平滑窗口：21点
- 速度缩放：85%
- 特点：平衡平滑性和响应性

#### Aggressive（激进模式）
- 插值倍数：4x (30FPS → 120FPS)
- 平滑窗口：15点
- 速度缩放：100%
- 特点：响应最快，但平滑性较差

## 使用方法

### 1. 基本使用
```bash
# 修改配置（在文件开头）
SMOOTH_LEVEL = "conservative"  # 选择平滑级别

# 运行程序
python extract_romp_joints_to_csv.py
```

### 2. 批量测试
```bash
# 测试所有平滑级别并比较效果
python test_smooth_effect.py
```

### 3. 输出文件
- `romp_output_g1_conservative.csv`：保守模式输出
- `romp_output_g1_moderate.csv`：中等模式输出
- `romp_output_g1_aggressive.csv`：激进模式输出
- `velocity_analysis.csv`：速度分析数据
- `smooth_comparison.png`：比较图表

## 技术细节

### 平滑算法
```python
# Savitzky-Golay滤波器
def smooth_signal_advanced(data, window_size=31, polyorder=3):
    return savgol_filter(data, window_size, polyorder)
```

### 速度限制
```python
# 限制关节速度并重新计算位置
def limit_velocity(positions, velocities, dt, max_velocities):
    # 应用速度缩放
    # 限制速度大小
    # 重新计算位置
```

### 多级处理流程
1. **数据加载**：从ROMP npz文件加载姿态数据
2. **插值**：使用三次样条插值提高采样率
3. **位置平滑**：Savitzky-Golay + 中值滤波
4. **速度计算**：数值微分计算角速度
5. **速度平滑**：对速度信号进行平滑
6. **速度限制**：应用关节速度限制
7. **加速度计算**：计算并平滑加速度
8. **输出**：生成CSV文件

## 性能指标

程序会输出以下统计信息：
- 总帧数和时间跨度
- 输出频率
- 平均速度和最大速度
- 各关节的速度分布

## 故障排除

### 常见问题
1. **动作仍然过快**：尝试使用更保守的平滑级别
2. **动作过于缓慢**：尝试使用更激进的平滑级别
3. **特定关节抖动**：检查该关节的速度限制设置

### 调试建议
1. 查看 `velocity_analysis.csv` 分析速度分布
2. 使用 `test_smooth_effect.py` 比较不同级别
3. 检查ROMP输入数据的质量

## 参数调优

### 关键参数
- `INTERP_RATIO`：插值倍数，影响输出频率
- `SMOOTH_WINDOW`：平滑窗口大小，影响平滑程度
- `VELOCITY_SCALE`：速度缩放因子，影响整体速度
- `JOINT_VELOCITY_LIMITS`：各关节速度限制

### 调优建议
1. 从保守模式开始
2. 根据实际效果逐步调整
3. 注意观察机器人的稳定性
4. 考虑安全因素，不要设置过高的速度限制 