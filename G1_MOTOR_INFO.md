# G1机器人电机信息完整文档

## 概述

G1机器人共有 **29个电机**，分为以下几个部分：
- **腿部**：12个电机（左右腿各6个）
- **腰部**：3个电机（但23DOF版本中腰部Roll和Pitch被锁定）
- **手臂**：14个电机（左右手臂各7个，但23DOF版本中手腕Pitch和Yaw无效）

## 电机索引定义

### G1JointIndex 类定义

```python
class G1JointIndex:
    # 左腿 (0-5)
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4          # 与LeftAnklePitch相同索引
    LeftAnkleRoll = 5
    LeftAnkleA = 5          # 与LeftAnkleRoll相同索引
    
    # 右腿 (6-11)
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10        # 与RightAnklePitch相同索引
    RightAnkleRoll = 11
    RightAnkleA = 11        # 与RightAnkleRoll相同索引
    
    # 腰部 (12-14)
    WaistYaw = 12
    WaistRoll = 13          # 注意：23DOF/29DOF版本中腰部被锁定，无效
    WaistA = 13             # 与WaistRoll相同索引
    WaistPitch = 14         # 注意：23DOF/29DOF版本中腰部被锁定，无效
    WaistB = 14             # 与WaistPitch相同索引
    
    # 左手臂 (15-21)
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20     # 注意：23DOF版本中无效
    LeftWristYaw = 21       # 注意：23DOF版本中无效
    
    # 右手臂 (22-28)
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27    # 注意：23DOF版本中无效
    RightWristYaw = 28      # 注意：23DOF版本中无效
```

## 详细电机规格表

### 1. 左腿电机 (索引 0-5)

| 索引 | 关节名称 | 英文名称 | 位置范围 (rad) | 力矩范围 (Nm) | Kp | Kd | 速度限制 (rad/s) | 轴方向 |
|------|---------|---------|---------------|--------------|----|----|-----------------|--------|
| 0 | 左髋关节俯仰 | LeftHipPitch | -2.5307 ~ 2.8798 | -88 ~ 88 | 60 | 1 | 2.0 | Y轴 |
| 1 | 左髋关节侧摆 | LeftHipRoll | -0.5236 ~ 2.9671 | -88 ~ 88 | 60 | 1 | 2.0 | X轴 |
| 2 | 左髋关节偏航 | LeftHipYaw | -2.7576 ~ 2.7576 | -88 ~ 88 | 60 | 1 | 2.0 | Z轴 |
| 3 | 左膝关节 | LeftKnee | -0.087267 ~ 2.8798 | -139 ~ 139 | 100 | 2 | 2.0 | Y轴 |
| 4 | 左踝关节俯仰 | LeftAnklePitch | -0.87267 ~ 0.5236 | -50 ~ 50 | 40 | 1 | 2.0 | Y轴 |
| 5 | 左踝关节侧摆 | LeftAnkleRoll | -0.2618 ~ 0.2618 | -50 ~ 50 | 40 | 1 | 2.0 | X轴 |

### 2. 右腿电机 (索引 6-11)

| 索引 | 关节名称 | 英文名称 | 位置范围 (rad) | 力矩范围 (Nm) | Kp | Kd | 速度限制 (rad/s) | 轴方向 |
|------|---------|---------|---------------|--------------|----|----|-----------------|--------|
| 6 | 右髋关节俯仰 | RightHipPitch | -2.5307 ~ 2.8798 | -88 ~ 88 | 60 | 1 | 2.0 | Y轴 |
| 7 | 右髋关节侧摆 | RightHipRoll | -2.9671 ~ 0.5236 | -88 ~ 88 | 60 | 1 | 2.0 | X轴 |
| 8 | 右髋关节偏航 | RightHipYaw | -2.7576 ~ 2.7576 | -88 ~ 88 | 60 | 1 | 2.0 | Z轴 |
| 9 | 右膝关节 | RightKnee | -0.087267 ~ 2.8798 | -139 ~ 139 | 100 | 2 | 2.0 | Y轴 |
| 10 | 右踝关节俯仰 | RightAnklePitch | -0.87267 ~ 0.5236 | -50 ~ 50 | 40 | 1 | 2.0 | Y轴 |
| 11 | 右踝关节侧摆 | RightAnkleRoll | -0.2618 ~ 0.2618 | -50 ~ 50 | 40 | 1 | 2.0 | X轴 |

### 3. 腰部电机 (索引 12-14)

| 索引 | 关节名称 | 英文名称 | 位置范围 (rad) | 力矩范围 (Nm) | Kp | Kd | 速度限制 (rad/s) | 轴方向 | 备注 |
|------|---------|---------|---------------|--------------|----|----|-----------------|--------|------|
| 12 | 腰部偏航 | WaistYaw | -2.618 ~ 2.618 | -88 ~ 88 | 60 | 1 | 1.5 | Z轴 | 有效 |
| 13 | 腰部侧摆 | WaistRoll | - | - | 40 | 1 | 1.0 | X轴 | **23DOF版本中锁定，无效** |
| 14 | 腰部俯仰 | WaistPitch | - | - | 40 | 1 | 1.0 | Y轴 | **23DOF版本中锁定，无效** |

### 4. 左手臂电机 (索引 15-21)

| 索引 | 关节名称 | 英文名称 | 位置范围 (rad) | 力矩范围 (Nm) | Kp | Kd | 速度限制 (rad/s) | 轴方向 | 备注 |
|------|---------|---------|---------------|--------------|----|----|-----------------|--------|------|
| 15 | 左肩俯仰 | LeftShoulderPitch | -3.0892 ~ 2.6704 | -25 ~ 25 | 40 | 1 | 1.5 | Y轴 | 有效 |
| 16 | 左肩侧摆 | LeftShoulderRoll | -1.5882 ~ 2.2515 | -25 ~ 25 | 40 | 1 | 1.5 | X轴 | 有效 |
| 17 | 左肩偏航 | LeftShoulderYaw | -2.618 ~ 2.618 | -25 ~ 25 | 40 | 1 | 1.5 | Z轴 | 有效 |
| 18 | 左肘关节 | LeftElbow | -1.0472 ~ 2.0944 | -25 ~ 25 | 40 | 1 | 1.5 | Y轴 | 有效 |
| 19 | 左腕侧摆 | LeftWristRoll | -1.97222 ~ 1.97222 | -25 ~ 25 | 40 | 1 | 1.0 | X轴 | 有效 |
| 20 | 左腕俯仰 | LeftWristPitch | - | - | 40 | 1 | 1.0 | Y轴 | **23DOF版本中无效** |
| 21 | 左腕偏航 | LeftWristYaw | - | - | 40 | 1 | 1.0 | Z轴 | **23DOF版本中无效** |

### 5. 右手臂电机 (索引 22-28)

| 索引 | 关节名称 | 英文名称 | 位置范围 (rad) | 力矩范围 (Nm) | Kp | Kd | 速度限制 (rad/s) | 轴方向 | 备注 |
|------|---------|---------|---------------|--------------|----|----|-----------------|--------|------|
| 22 | 右肩俯仰 | RightShoulderPitch | -3.0892 ~ 2.6704 | -25 ~ 25 | 40 | 1 | 1.5 | Y轴 | 有效 |
| 23 | 右肩侧摆 | RightShoulderRoll | -2.2515 ~ 1.5882 | -25 ~ 25 | 40 | 1 | 1.5 | X轴 | 有效 |
| 24 | 右肩偏航 | RightShoulderYaw | -2.618 ~ 2.618 | -25 ~ 25 | 40 | 1 | 1.5 | Z轴 | 有效 |
| 25 | 右肘关节 | RightElbow | -1.0472 ~ 2.0944 | -25 ~ 25 | 40 | 1 | 1.5 | Y轴 | 有效 |
| 26 | 右腕侧摆 | RightWristRoll | -1.97222 ~ 1.97222 | -25 ~ 25 | 40 | 1 | 1.0 | X轴 | 有效 |
| 27 | 右腕俯仰 | RightWristPitch | - | - | 40 | 1 | 1.0 | Y轴 | **23DOF版本中无效** |
| 28 | 右腕偏航 | RightWristYaw | - | - | 40 | 1 | 1.0 | Z轴 | **23DOF版本中无效** |

## 控制参数配置

### 默认PD控制参数

```python
# 位置控制增益 (Kp)
Kp = [
    60, 60, 60, 100, 40, 40,      # 左腿
    60, 60, 60, 100, 40, 40,      # 右腿
    60, 40, 40,                   # 腰部
    40, 40, 40, 40, 40, 40, 40,  # 左手臂
    40, 40, 40, 40, 40, 40, 40   # 右手臂
]

# 速度控制增益 (Kd)
Kd = [
    1, 1, 1, 2, 1, 1,     # 左腿
    1, 1, 1, 2, 1, 1,     # 右腿
    1, 1, 1,              # 腰部
    1, 1, 1, 1, 1, 1, 1,  # 左手臂
    1, 1, 1, 1, 1, 1, 1   # 右手臂
]
```

### 特殊控制模式

G1机器人支持两种控制模式：

1. **PR模式 (Mode.PR = 0)**: 串行控制俯仰/侧摆关节
   - 用于控制踝关节的Pitch和Roll
   - 使用 `LeftAnklePitch` 和 `LeftAnkleRoll` 索引

2. **AB模式 (Mode.AB = 1)**: 并行控制A/B关节
   - 用于控制踝关节的A和B电机
   - 使用 `LeftAnkleA` 和 `LeftAnkleB` 索引（与Pitch/Roll共享索引）

## 关节名称映射

### CSV文件中的关节名称

```python
JOINT_NAMES = [
    'L_HIP_YAW', 'L_HIP_ROLL', 'L_HIP_PITCH', 'L_KNEE', 
    'L_ANKLE_PITCH', 'L_ANKLE_ROLL',
    'R_HIP_YAW', 'R_HIP_ROLL', 'R_HIP_PITCH', 'R_KNEE', 
    'R_ANKLE_PITCH', 'R_ANKLE_ROLL',
    'WAIST_YAW', 'WAIST_PITCH', 'WAIST_ROLL',
    'L_SHOULDER_PITCH', 'L_SHOULDER_ROLL', 'L_SHOULDER_YAW', 
    'L_ELBOW', 'L_WRIST_ROLL', 'L_WRIST_PITCH', 'L_WRIST_YAW',
    'R_SHOULDER_PITCH', 'R_SHOULDER_ROLL', 'R_SHOULDER_YAW', 
    'R_ELBOW', 'R_WRIST_ROLL', 'R_WRIST_PITCH', 'R_WRIST_YAW'
]
```

## 速度限制建议

根据实际应用场景，建议的关节速度限制：

```python
JOINT_VELOCITY_LIMITS = {
    # 腿部关节
    "L_LEG_HIP_PITCH": 2.0, "L_LEG_HIP_ROLL": 2.0, "L_LEG_HIP_YAW": 2.0,
    "L_LEG_KNEE": 2.0, "L_LEG_ANKLE_PITCH": 2.0, "L_LEG_ANKLE_ROLL": 2.0,
    "R_LEG_HIP_PITCH": 2.0, "R_LEG_HIP_ROLL": 2.0, "R_LEG_HIP_YAW": 2.0,
    "R_LEG_KNEE": 2.0, "R_LEG_ANKLE_PITCH": 2.0, "R_LEG_ANKLE_ROLL": 2.0,
    
    # 腰部关节
    "WAIST_YAW": 1.5,
    
    # 手臂关节
    "L_SHOULDER_PITCH": 1.5, "L_SHOULDER_ROLL": 1.5, "L_SHOULDER_YAW": 1.5,
    "L_ELBOW": 1.5, "L_WRIST_ROLL": 1.0,
    "R_SHOULDER_PITCH": 1.5, "R_SHOULDER_ROLL": 1.5, "R_SHOULDER_YAW": 1.5,
    "R_ELBOW": 1.5, "R_WRIST_ROLL": 1.0
}
```

## 重要注意事项

1. **23DOF vs 29DOF**:
   - G1有23DOF和29DOF两种配置
   - 23DOF版本中，腰部Roll和Pitch被锁定（索引13、14无效）
   - 23DOF版本中，手腕Pitch和Yaw无效（索引20、21、27、28无效）

2. **踝关节控制**:
   - 踝关节支持PR模式和AB模式两种控制方式
   - PR模式：使用Pitch/Roll索引（4、5、10、11）
   - AB模式：使用A/B索引（与Pitch/Roll共享索引）

3. **膝关节特殊处理**:
   - 膝关节（索引3、9）具有更大的力矩范围（-139 ~ 139 Nm）
   - 建议使用更高的Kp值（100）和Kd值（2）

4. **控制频率**:
   - 建议控制频率：500Hz（控制周期 0.002秒）
   - 低级别控制通过 `rt/lowcmd` 和 `rt/lowstate` 通道通信

5. **安全限制**:
   - 所有位置命令应在关节范围内
   - 力矩命令不应超过力矩限制
   - 建议在控制前检查关节状态

## 参考代码

### Python SDK使用示例

```python
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber

# 创建控制命令
low_cmd = LowCmd_()

# 设置电机控制参数
motor_idx = G1JointIndex.LeftHipPitch  # 示例：左髋关节俯仰
low_cmd.motor_cmd[motor_idx].mode = 1  # 1:启用, 0:禁用
low_cmd.motor_cmd[motor_idx].q = 0.0   # 目标位置 (rad)
low_cmd.motor_cmd[motor_idx].dq = 0.0  # 目标速度 (rad/s)
low_cmd.motor_cmd[motor_idx].tau = 0.0 # 前馈力矩 (Nm)
low_cmd.motor_cmd[motor_idx].kp = 60.0 # 位置增益
low_cmd.motor_cmd[motor_idx].kd = 1.0  # 速度增益

# 设置控制模式
low_cmd.mode_pr = Mode.PR  # 或 Mode.AB
```

## 数据来源

- 电机规格信息来自：`armdev/g1_23dof.xml` (MuJoCo模型文件)
- 关节索引定义来自：`legdev/g1_leg.py`, `g1dev/g1_dev_data.py`
- 控制参数来自：`legdev/g1_leg.py`
- 速度限制来自：`recorddata/extract_romp_joints_to_csv.py`

---

**最后更新**: 2025-12-10
**版本**: G1 23DOF/29DOF
