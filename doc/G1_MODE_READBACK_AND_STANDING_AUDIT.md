# G1 模式回读与站稳证据核查

2026-10-21，北京时间；G1 EDU 23 自由度，Orin。全程只读，不切换模式、不发送动作，不修改 Quantumult X。

## 已查明的范围

- 重启后绑定 boot ID `b33a7dc5-2617-44cd-b353-041bab1c204f`，重新核验四个官方进程的 PID、启动时刻与 argv。旧启动周期的记录继续保留为历史证据。
- 两个独立的 60 秒对照分别在控制台断开、连接时调用三个固定官方 getter，各 60/60 次成功；被动监听没有发现这些请求缺少官方响应。这将排查重点收敛到控制台接收边界，不能单凭此结果指定唯一的线程或队列根因。
- 原控制台 61 次状态快照中，CheckMode 有 18 次因 3104 撤回，GetFsmMode 有 11 次；快照数量不是 RPC 失败率。
- 将三个 getter 放入独立 DDS 进程，串行请求并按 request ID/API ID 关联响应。保留原始接收时间，超时、断流或子进程异常立即撤回当前值；不延长 TTL，不把上一次成功当成当前成功。界面显示实际请求计数。

## 站稳与承重

当前实现只学习并展示证据缺口，`already_standing=null`、`execution_allowed=false`，不选择起身路线。关节角度、IMU 小倾角、低速度、RGB 或 FSM 编号均不能单独证明站稳。

新启动周期发现 136 个 DDS publication。`rt/odommodestate` 的声明类型为 Go 系列 SportModeState，197 个样本中的四路 foot_force 均为零；尚未确认它们对应本机 G1 左右脚的有效传感器，不能解释为零承重。其他有限范围发现未找到足底专用主题，不等于接口不存在。固定只读 GetBalanceMode / GetSwingHeight / GetStandHeight 仍返回 7301，保持未知。

基础档案分别保留 Python SDK 起身转换和 C++ StandUp 的版本差异，仅作接口文档知识；尚未核定本机固件的适用路线、慢速参数、持续时间或轨迹。SetSpeedMode / SetStandHeight 不当作已经证实的缓慢起身控制。

需继续取得起始姿势、外部支撑、双脚接触与承重、重心和支撑区域关系、官方控制转换的实机适用性证据，才可能评估具体路线。目前没有“不摔倒”的已验证动作范围。

## 阻尼的含义

阻尼主要抵抗关节运动，姿势保持还涉及位置刚度、目标与力矩，平衡站立还涉及全身协调和支撑反馈。保持阻尼模式不能作为主动撑住身体的保证。官方 SDK 将 Damp、ZeroTorque、Start 分开；这些封装名称和编号不证明本机电机实际出力。

参考：[官方 G1 Loco 封装](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/g1/loco/g1_loco_client.py)、[官方低层示例中分开的刚度与阻尼字段](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/example/g1/low_level/g1_ankle_swing_example.cpp)。示例数值不作为实机参数。

## 产物与验证

基础档案 v15 保留模式传输对照、旧控制台快照、新启动周期支撑核查和官方进程身份，共 31 份安全来源。253 个未知实机电机配置字段继续为 null，七类安全条件均未核定。

相关测试 134 项通过，前端构建通过。发布与三分钟实测结果见 [验证记录](G1_MODE_READBACK_AND_STANDING_VERIFICATION.json)。实测窗口不能外推为长期可用性或运动安全保证。

## 已部署版本的连续实测

发布 `8c0a84d0fb97`，2026-10-21 14:17:32–14:20:32（北京时间 UTC+8）。181 次状态采样均取得实时相机与三个有效模式值。实际请求计数：GetFsmId 355/355、GetFsmMode 60/60、CheckMode 59/59，共 474/474 成功，超时与其他错误均为 0；37/37 次 JPEG 成功且哈希均不同，身体反馈最大延迟 60 ms。

当前 FSM 为 0、CheckMode 为 form=0/name=ai，编号交叉核对一致；双膝约 160.55°/159.07°。具体官方能力是否执行、世界姿态、双脚接触与承重仍未知，不能由稳定回读推导已站稳。

449 个发布文件 SHA256 一致，四个官方进程的启动周期/PID/启动时刻/参数一致；控制台以只读模式监听 Orin 回环地址，模式与相机使用独立接收子进程。验证期间未切换模式或执行动作，未修改 Quantumult X。
