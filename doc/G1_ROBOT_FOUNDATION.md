# G1 身体基础档案

2026-10-21，北京时间（UTC+8）。适用设备：用户确认的 G1 EDU、23 自由度、无手、Jetson Orin。当前连接仅用于读取。

**机器人已有的结构和参数定义是持久基础知识，不由 Codex 每次重新猜测。实时关节反馈与静态档案分开，当前状态随反馈刷新。** 这里的“基础模型”是有来源、可查询的机器人描述与条件规则，不是训练新的神经网络或运动策略。

当前 v21 增加用户提供的官方 App 截图证据：型号 G1 EDU+、软件 v1.5.6、硬件 v1.0。该软件字段尚未证明对应运动固件；后者继续为 `null`，23 DOF 配置不因 EDU+ 标签改变。截图 IMU 与通信质量作为历史观察单独保存，253 个实机电机配置字段保持未知。详见 [App 截图核对](G1_APP_EVIDENCE_REVIEW.md)；v20 的 [104 版本诊断](G1_OTA_VERSION_DIAGNOSIS.md) 保留。

## 持久保存与访问

- `g1_console/assets/g1_motor_knowledge.json`：23 电机档案、全部 10 项 HG MotorState 字段、7 项 MotorCmd 字段、PR/AB 坐标定义、已核实的 SDK 状态映射、未知项、官方文件 commit 和 SHA256。
- `g1_console/assets/g1_23.json`：身体关节树、官方编号映射、3D 网格关系。
- `GET /api/body` 的 `motor_knowledge`：启动即可查询的同一份基础档案，即使没有连接机器人也可加载。
- `GET /api/state` 的 `telemetry` 与 `assessment`：实时原始数据、模型角度比较、当前能力条件。断流不延长有效期；过期结论变为待核实。
- Mac 观察输入中的 `foundation`、`assessment` 和能力 `compatibility`：自动附带给每次 Codex 观察，无需再次搜索或推断静态参数。报告证据保存 `foundation_id`，可追溯使用的版本。

档案随项目和 Orin 版本化发布目录保存，重启控制台、Mac worker 或切换 Codex 会话不会丢失。`foundation_id` 由档案内容确定；只有资料或已核实配置发生变化才产生新版本。滚动速度窗口仅在内存保存约 12 秒，断流或重连清空，不作为永久身体参数。

## 参数的证据边界

| 类别 | 已保存内容 | 使用边界 |
|---|---|---|
| 用户设备配置 | EDU、23 DOF、无手、Orin | 用户确认；另以新鲜 mode_machine=4 和 23 关节编号核对官方机型文档 |
| 旧 RL Gym 参考模型 | 角度、URDF 速度和 effort、MuJoCo 力矩范围、关节轴与原点、父子连杆、质量和惯量 | 参考模型不代表本机持续安全值；URDF 总质量不代表称量结果 |
| 官方示例 | 每个电机的 Kp、Kd | 仅读取示例源码，没有运行；不等于出厂或当前官方控制器增益 |
| 实时电机反馈 | mode、q、dq、ddq、tau_est、temperature[2]、vol、sensor[2]、motorstate、reserve[4] | 保留类型、单位和原值；未知字段语义不补猜 |
| 对应机型的官方文档 | mode 4 = g1_23dof_rev_1_0；髋俯仰减速比 14.3、髋侧摆 22.5；23 关节对应 URDF 限值 | 每次观察核对适用性；文档值不是逐电机标定，模型限值不是连续/峰值规格 |
| 实机身份与配置 | 当前增益、连续/峰值力矩、电流上限、减速器标定、零位、型号、序号、驱动固件和热限制 | 尚未取得，actual_config 保存为 null，界面显示未知 |
| 其他身体反馈 | IMU、tick、mode_pr、mode_machine、version、FSM 及各自时效 | LowState.version 保留原始数组，不冒充完整固件版本；LowState 不含电池字段，v12 另接 HG BmsState 原始消息，电量和健康待核定 |

官方 G1 SDK 将 `temperature[0]` 定义为外壳、`temperature[1]` 定义为绕组温度。辅助函数中的默认 85/120°C 可由调用方覆盖，不是从实机读取的固件阈值，也不作为健康判断。`motorstate=0` 不等于已证明健康。`sensor` 和 `reserve` 没有解码位表，不解释为特定传感器。`tau_est` 是估算值，`ddq=0` 也不证明固件支持有效加速度测量。

## 当前状态与官方能力如何比较

`assessment` 为确定性只读规则，输出 `matched / violated / unknown`，并保留依据。

1. 23 个关节逐一比较参考模型角度，保留越界幅度。踝关节必须确认 `mode_pr=0` 的 PR 坐标；AB 或未知坐标不直接比较，也不作为该踝的 3D 角度。微小越界仍显示偏差，但不诊断电机损坏。
2. 官方 SDK 的 `ZeroTorque()` 对应 FSM 0，`Start()` 对应 500。这里只解释 SDK 映射；FSM 0 不等于每个估算力矩都为零。FSM 回读过期 1.5 秒后不作为当前条件。
3. 对实机 `GetActionList` 明确返回的 FSM / mode_machine 列表逐值比较。没有声明条件的条目标为待核实；只满足声明条件不证明完整前提已满足。
4. 本项目速度控制要求 FSM 500，这一规则标为项目门槛，不扩张为全部官方动作的统一规则。
5. 稳定站立、不摔倒、电机健康、足底接触和承重缺乏足够证据，持续保留待核实。低关节速度、小 IMU 倾角或参考 3D 形态都不能证明这些条件。

所有比较的 `execution_allowed` 均为 false。Orin 强制只读、无控制权；此模块不执行动作、不修改增益、不发布 LowCmd，也不切换官方控制模式。

## 2026-10-21 参数核查增补

基础档案 v10 保存对应机型文档、温度通道、参数混用排除项和带时间的 Orin / 其他控制板检查。新资料固定 commit 和 SHA256。`board_audit` 保存主板 / BMS 原码、被动命令载荷和安装 XML；`interface_audit` 保存官方协议核查、6 个服务 API 版本、28 条软件服务记录及包版本查询 3203。它们均为历史证据，不填实机增益、驱动固件或标定，不判定当前健康；253 个实际电机配置字段仍为 null。Go2/B2 配置读取协议的 G1 适用性未知，不能因为版本相同就调用。机型或关节配置不符、断流时，文档适用性不显示为符合。详见 [电机参数核查](G1_MOTOR_PARAMETER_AUDIT.md)。

## 官方来源与更新

- [HG MotorState 字段](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/idl/unitree_hg/msg/dds_/_MotorState_.py)
- [G1 低层示例：坐标和示例增益](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/example/g1/low_level/g1_low_level_example.py)
- [官方 G1 状态映射](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/g1/loco/g1_loco_client.py)
- [官方 G1 23 DOF URDF](https://github.com/unitreerobotics/unitree_rl_gym/blob/276801e46c5d433564f24658bac64f254b7d2d4b/resources/robots/g1_description/g1_23dof.urdf)

仅当权威资料或硬件配置变化时更新。生成器通过 XML / AST 读取锁定文件，不导入或执行控制示例：

```bash
.venv/bin/python scripts/build_g1_motor_knowledge.py
CYCLONEDDS_HOME="$PWD/.venv/opt/cyclonedds" .venv/bin/python -m pytest tests/test_g1_foundation.py tests/test_g1_insight.py -q
```

先核对来源版本及实机适用性，再更新档案并测试、构建、发布。无法公开读取的工厂配置需要本机专属资料或官方支持补齐，不能用探索电机极限来补全。

`github_review` 保存 2026-10-21 10:15 北京时间的公开 issue 核查：14 组查询取得 198 个去重命中，14 个相关 issue 的 47 条评论已完整读取。8 个核心仓库均未启用 Discussions；不外推到整个组织。来源身份和适用范围分别保存，官方仓库内的社区回复不能升级为本机配置；未找到可核实的 G1 配置 / 标定只读导出协议。保留检索分页、元数据与正文哈希并校验快照 SHA256，不保存或执行评论中的控制代码。

`service_audit` 与 `official_capabilities` 新增本机服务启动/日志证据、76 个官方方法与重载的完整参数、跨 SDK 差异及 7 次只读回读。未知单位/范围和实参保持空值，不新增动作许可。详见 [服务与能力参数核查](G1_SERVICE_CAPABILITY_AUDIT.md)。

`service_audit.configuration_analysis` 保存两处非明文配置的后续离线分析：MegaFlow config.bin 已核定为 AES-256-CBC JSON 组件清单；master_service.json 仅核定 FMX 容器标记，内部未知。未取得电机配置或标定，不将声明插件升级为可执行能力。详见 [配置格式核查](G1_OPAQUE_CONFIGURATION_AUDIT.md)。

`service_audit.fmx_analysis` 补充同一实机 FMX 文件的结构、5 个受保护入口、7 个 ELF 的有界搜索和 16 个离线假设测试。完整载荷仍未知，253 个实机电机字段继续留空。详见 [FMX 后续核查](G1_FMX_ANALYSIS.md)。

`service_audit.fmx_loader_review` 新增本机 master_service_pc4 1.0.0.2 的模块清单声明、文件大小和内容指纹关联，以及扩展目录检索、OTA 缓存版本差异和官方索引核查。没有取得完整 Mixer/Gen 或电机参数；模块声明不升级为电机固件版本。详见 [Mixer / Gen 与版本匹配核查](G1_FMX_LOADER_SEARCH.md)。

`service_audit.large_library_review` 补齐此前超过上限的 libcryptopp.a / librealsense2.so.2.50.0 静态扫描，绑定实机指纹、时间、master 关联与正向对照。未取得完整 Mixer/Gen 或 FMX 载荷，新增实参 0 项；通用密码算法与相机标定不进入 actual_config。详见 [两份大库核查](G1_LARGE_LIBRARY_AUDIT.md)。

基础档案 v11 增加 `service_audit.package_cache_review` 与 `state_safety`：补扫官方安装/OTA 缓存 51 文件、5 ELF、4 软件包，发现目录版本与已安装补丁清单一致的 1.0.0.6 UTPK；内部仍未知，新增电机实参 0。保存 7 个官方保护辅助函数的默认值、单位和非实机阈值属性，以及项目拦截线、缺失证据和空的逐动作范围。实时 `assessment.state_learning` 分开模式、具体动作未知、关节形态、世界支撑未知和 12 秒变化；每 30 秒只记录带证据的历史状态，不自动扩张安全范围。真实运动在最终分发处锁定，关闭只读或人工勾选也不放行。详见 [状态、安全与 OTA 核查](G1_STATE_SAFETY_AND_OTA_AUDIT.md)。

基础档案 v12 增加 `safety_evidence_audit`：保存用户重接 Type-C 后的 D435i 枚举、官方成功取图、同期身体／场景历史观察，以及固定版本 HG 四字段消息定义。实时解释器新增 HG 模式／任务、BMS、主板、动作状态、自检和只读 `CheckMode`，分别标记接收新鲜度，七类安全条件分别显示已取得与缺失证据。BMS 原值不推定百分比或健康，task_id=4 不推定具体动作。相机已接通但复测仍间歇 `SDK_3104`，过期 RGB 自动撤回；历史画面不随当前状态更新。新增电机实参仍为 0、253 字段留空、逐动作安全范围为空。详见 [七类安全证据与相机只读核查](G1_SAFETY_EVIDENCE_AUDIT.md)。

基础档案 v13 增加 `safety_evidence_audit.camera_transport`：保存 SDK_3104 的旧接收器基线、被动官方响应对照、同进程轮询失败记录、独立进程成功对照与三分钟集成复测。实测窗口内 899/899 次取图成功，Mac 37/37 张 JPEG 成功且哈希不同，181 次状态采样均为 live；这些是带时间戳的历史测量，不是长期可用性或动作安全保证。新的同期图像与原始关节、IMU、HG 任务、BMS、主板和 CheckMode 绑定到 run/tick 与 SHA256。能确认双膝约 161°/160°屈曲，不能确认全身站/坐/卧、承重或具体能力正在执行。253 个实机电机配置字段继续为空，7 类安全条件仍未核定。详见 [SDK_3104 相机稳定性核查](G1_CAMERA_STABILITY_AUDIT.md)。

基础档案 v14 增加 `safety_evidence_audit.posture_interface_audit`：补查躯干 IMU、骨盆/躯干里程计、专用 GetFsmMode 及其他接触/活动证据。当前控制台持续展示新增原始反馈，分别标记时效；里程计 z 不当作离地高度、四路全零 foot_force 不映射为双脚承重、7301 按官方“运动状态数据不可用”保留未知。基础档案只保存带时间的历史核查，当前站/坐/卧、左右接触/载荷和具体能力活动未确认，实机电机配置仍无新增值。详见 [当前姿势、承重与能力核查](G1_POSTURE_SUPPORT_AUDIT.md)。

基础档案 v15 增加 `safety_evidence_audit.mode_readback_audit` 与 `standing_knowledge`：绑定重启后的官方进程身份，保存模式请求/响应对照及新一轮足底证据核查；站立转换只记录固定版本官方文档，不作为本机执行许可。实时三个模式 getter 改用独立 DDS 进程，显示实际 RPC 计数与原始接收时效。站稳、承重、慢速起身参数仍未知，253 个实机电机配置字段继续为空。详见 [模式回读与站稳证据核查](G1_MODE_READBACK_AND_STANDING_AUDIT.md)。

基础档案 v16 增加 `standing_knowledge.support_audit`：保存用户确认的坐地、双脚贴地、人员支撑及数字视图相符，明确为有入档时间的人工历史证据；实时传感器的支撑判断仍未知。新增六次固定官方只读查询、同期 61 次身体采样和手册版本核对。GetBalanceMode / GetStandHeight 的 7301 不填成 0；手册名义过渡时间不当作实机慢速参数。形成五阶段条件方案，未选择起身转换、未核定独立站稳、未执行动作，253 个电机实参继续为空。详见 [坐地起点与站稳条件方案](G1_SEATED_STANDING_REVIEW.md)。

基础档案 v17 将当前人工支撑记录更正为“靠柜，非人员托住”，保留 v16 原文、来源哈希和更正关系。坐地、双脚贴地及数字形态相符沿用原记录时间，不冒充本次重新确认。新增 Orin 安装包清单和 20 秒被动 DDS 核查：PC4 模块版本不当作运动固件，LowState 的 `[0, 0]` 不解码为有效版本，MainBoardState 没有版本字段，四路全零足力没有经核实的 G1 双脚映射。左右脚与柜体载荷、独立站稳和 253 个实际电机配置字段保持未知。条件方案增加柜体接触与多处支撑核查，不选择或执行动作。详见 [v17 靠柜更正与条件方案](G1_SEATED_STANDING_REVIEW.md)。

基础档案 v18 增加 `standing_knowledge.support_audit.applicability_review`：保存六类官方能力对靠柜坐地起点的适用性核对、左右脚/坐地处/柜体四处接触的证据和缺口，以及每阶段的完成证据。历史 OTA 1.5.3.8 目录标签不当作当前运动固件；六个状态通道有限窗口无消息不推定无活动。起身包装未公开速度/时长，速度命令 duration 与手臂混合权重不当作全身慢速起身参数。四处载荷、站稳判据及 253 个电机实参继续为空。详见 [v18 只读适用性核查](G1_SEATED_STANDING_REVIEW.md)。
