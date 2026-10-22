# G1 当前姿势、承重与官方能力只读核查

核查日期：2026-10-21，北京时间。设备为用户确认的 G1 EDU 23 自由度、无手、Orin。历史证据绑定同一启动 ID `7a6791dc-f512-4b45-aca3-44fbde855e25`；不沿用为下一次启动后的当前状态。

## 本次可确认与未知

| 问题 | 取得的证据 | 可下的结论 |
|---|---|---|
| 当前关节形态 | 同期相机与 LowState，双膝约 161° / 160°；主 IMU 横滚约 7.8°、俯仰约 -3.5° | 双膝显著屈曲；不能确定站、坐、卧或是否有外部支撑 |
| 现场画面 | 瓷砖地面、带轮椅子、人员腿脚；机器人足底接触面未完整进入视野 | 不能从人员脚部或画面下沿局部机器人部件推定机器人双脚接地 |
| 躯干朝向 | 官方 G1 示例使用的 `rt/secondary_imu`，消息类型匹配 HG IMU，20 秒接收 384 条 | 增加另一组原始朝向；未核定安装外参、传感器健康，不据此融合世界姿势 |
| 骨盆、躯干位姿 | 两个 `nav_msgs::msg::dds_::Odometry_` 话题各接收 384 条；frame=odom，child 分别 pelvis / torso | 获得里程计估计；原点与地面关系未知，z≈0.314 / 0.367 米不解释为离地高度，全零协方差不解释为无误差 |
| 双脚是否承重 | HG LowState / SportModeState 无对应足底载荷字段；实际 GO 类型 `rt/odommodestate` 的四路 foot_force 全为 0 | 四路字段在这台双足 G1 的有效性、单位及左右映射未知；左、右接触与载荷均保存 null |
| 是否执行官方能力 | GetFsmId=0、GetFsmMode=0、HG fsm_id=0 / fsm_mode=0 / task_id=4 / task_time=0，CheckMode={form:"0",name:"ai"} | 保存服务原始回读。不能把 FSM 映射、task_id 或 ai 标识当作某项能力实际执行中、完成或空闲的证明 |
| 平衡模式与高度 | GetBalanceMode / GetSwingHeight / GetStandHeight 都返回 7301 | 官方定义为 `LocoState not available.`；数据继续未知，不切换模式获取 |

取样计数是有界 reader 轮询收到的数量，不能当作真实发布频率。DDS 发现的 138 个发布端点、167 个订阅端点不是控制板数量。`physical_publisher_verified` 均为 false，接收时效不代表传感器健康。

## 已核查的其他只读接口

| 接口 | 本次结果 | 控制台处理 |
|---|---|---|
| rt/secondary_imu | 类型匹配并持续收到 | 持续被动订阅，1.5 秒过期撤回 |
| rt/state_estimator/odom_pelvis、odom_torso | 类型匹配并持续收到 | 持续被动订阅，1.5 秒过期撤回；展示原始位姿、速度与协方差 |
| GetFsmMode / 7002 | 成功，data=0 | 仅调用官方专用 getter，空参数、无 lease，3 秒查询、6 秒过期撤回 |
| rt/state_estimator/fusion_odom、fusion_odom_torso | 20 秒窗口内无消息 | 保存历史缺失证据，不补零、不推定接口永久不可用 |
| rt/arm/action/state、rt/selftest | 20 秒窗口内无消息 | 既有被动订阅保留；无消息不等于空闲、无故障 |
| rt/odommodestate | 先按发现的实际 GO 类型匹配，再被动接收 | 保留核查记录；四路足力语义未核定，不进入实时左右脚载荷 |
| GetPhase / 7006 | 官方已弃用 | 未调用 |
| GetMimicMotion / 7100 | 与动作 API 复用且有控制模式前提 | 未调用，不为补证据而改变机器人模式 |

官方 `PressSensorState` 出现在灵巧手 HandState 定义中，不能据此声称 G1 双脚有可读压力传感器。本次公开协议和有界 DDS 发现未补齐足底负载接口，不代表硬件或私有协议绝对不存在。

## 部署后持续验证

Orin 自有只读控制台发布为 `4f0c4a2b492b`，基础模型 v14 / `a7befbcdb87d…`；442 个发布文件的 SHA256 全部匹配。四个已记录的宇树官方进程启动身份不变。

北京时间 **2026-10-21 13:05:34–13:06:34**，连接就绪后独立测量 60 秒：

- 61 / 61 次状态采样相机均为 live，官方取图增量 **300 / 300 成功、0 超时**；13 张 JPEG 全部可解码且哈希不同。相机状态帧龄最大 171 ms，JPEG 响应帧龄另存原始回执；身体反馈帧龄最大 71 ms。
- 躯干 IMU、骨盆和躯干里程计均在 61 / 61 次采样时新鲜。最终双膝 161.031° / 160.015°，主 IMU 横滚 7.855° / 俯仰 -3.512°；站/坐/卧、足底接触和载荷仍未知。
- `CheckMode` 51 / 61 次采样可用、10 次采样记录 3104；`GetFsmMode` 31 / 61 次采样可用、30 次采样记录 3104。**这是状态快照计数，同一失败可被多次采样，不能当作 RPC 失败次数或成功率。** 两接口间歇超时的根因尚未确认，失败立即撤回；最终快照两者均未知。HG 模式/任务流持续新鲜，最终 fsm_id=0 / fsm_mode=0 / task_id=4 / task_time=0；仍不证明具体能力正在执行或空闲。
- 最后画面可见地砖、带轮椅脚及画面下沿局部部件，机器人足底与承重点不在完整视野内。相机请求时间与本机接收时间不等于曝光时间，身体与画面没有硬件同步保证。
- 观察器暂停，导航 idle、控制权 owner=null；动作许可 false、7 类安全条件未核定、253 个实机配置字段仍为空。未执行动作，未改 Quantumult X。

服务重启后、只读连接尚未恢复时启动的第一份采样包含 waiting / 空身体值，旧汇总脚本也因 None 比较失败。这份初始化窗口留档，不计入上述稳定窗口；随后确认只读连接和相机 live，再独立完成上述测量。

验证：G1 测试 **122 项通过**（1 条 Starlette/httpx 弃用提示）；前端构建通过（既有三维 chunk 大小提示）。Mac 浏览器已核对实时角度、左右脚未知、模式失败撤回和新增原始通道，620 px 宽度可阅读。截图与文件指纹见 `G1_POSTURE_SUPPORT_VERIFICATION.json`。这些结果只覆盖本次窗口，不保证后续长期稳定，也不构成防摔认证。

## 持久基础模型与实时状态的区别

基础模型 v14 持久保存接口定义、实机历史回执、错误码语义、排除项及 SHA256；`posture_interface_audit.current=false`。身体实时值随消息更新，断流自动撤回，不能让历史角度冒充当前姿势。控制台新增左、右脚接触与载荷未知显示、主/躯干 IMU 区分和里程计原始反馈。

本次新增实机电机配置值 **0 项**；253 个 actual_config 字段仍为 null。7 类安全证据继续不完整，已核定动作范围为 0。安静、速度接近零、画面稳定或观察期间未摔倒都不生成防摔安全边界。

## 来源与验证

原始回执在 `g1_console/assets/safety_evidence_sources/*20261021_v14.json`，统一来源清单在 `g1_console/assets/g1_safety_evidence_audit.json`。生成基础模型时校验全部来源 SHA256，测试比较重新生成结果。

- [官方 G1 躯干 IMU 示例定义](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/example/g1/low_level/g1_ankle_swing_example.cpp)：只读源码，未执行示例。
- [官方 G1 专用 getter](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/g1/loco/g1_loco_client.hpp)。
- [官方 G1 错误码 7301](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/g1/loco/g1_loco_error.hpp)。

部署、持续采样、界面及测试结果另存 `G1_POSTURE_SUPPORT_VERIFICATION.json`。所有读取均未发布 LowCmd、未请求控制权、未调用运动 setter；只更新本项目自有只读服务。Quantumult X 和宇树官方服务不改动。
