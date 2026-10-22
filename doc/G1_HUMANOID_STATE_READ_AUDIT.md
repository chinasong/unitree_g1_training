# humanoid 双链路：运动固件与 7301 只读核查

2026-10-21，北京时间（UTC+8）。发布 **`953b51275852`**，基础档案 **v26**。Mac 入口：[G1 控制台](http://127.0.0.1:18765/)。

**确认 `.161` 的两条官方 HG 状态流与 7301 错误应答在同一窗口共存；实际运动固件版本和 7301 内部触发条件仍未知。** 软件新增双链路证据、官方读取入口及适用条件面板。本轮未执行动作、切换模式/官方服务、申请控制权或写入机器人配置；未操作 Quantumult X。

## 实机采样与结论

北京时间 **22:11:45–22:12:01**，16 秒被动 RTPS 头部观察、DDS 内置发现、HG 状态订阅，以及两项固定官方 getter。原始证据：[状态与应答](../g1_console/assets/safety_evidence_sources/humanoid_state_20261021_v30.json)。

| 读取项 | 实际结果 | 解释边界 |
| --- | --- | --- |
| GetFsmId / sport 7001 | code 0，`{"data":0}`，5.55 ms | 公开 SDK ZeroTorque 编号；不测量全部实际电机力矩 |
| GetBalanceMode / sport 7003 | code 7301，数据为空，5.50 ms | 收到错误应答，不是 3104 超时；模式值保持未知 |
| `rt/sportmodestate` | 1559 个有效 HG 帧；接收窗口 0.011–16.014 s | `fsm_id=0, fsm_mode=0, task_id=4, task_time=0.0` |
| `rt/lf/sportmodestate` | 240 个有效 HG 帧；接收窗口 0.031–15.963 s | 原始字段相同；task_id=4 的本机语义未核定 |

两条状态流的接收时间均覆盖 7003 请求与错误回包。**因此，本窗口不是完全缺少 HG 状态广播，也不是 3104 超时。** 但这不证明服务内部 LocoState 缓存、门控或版本兼容正常，不能确定哪个内部条件触发 7301。帧计数不证明没有丢包；本地接收时间不是传感器时间。

本轮另外两次探索采样也各调用同样的 7001 / 7003 固定 getter；三次合计 **6 次查询，均未出现 3104**。这 6 次与发布后控制台后台轮询分别计数，不代表长期没有超时。

### 两个参与者分别保留

通过请求标识/API → 应答 publication handle → 发布者 GUID → 参与者 GUID → 被动 RTPS 源 IP 配对，实际 sport 应答来自 `.161`。状态发布者采用另一参与者 GUID：

| 链路 | 参与者 GUID | DDS 自报信息 |
| --- | --- | --- |
| sport RPC 应答 | `01102018-14ee-e682-8024-7c60000001c1` | Unitree / humanoid / PID 3396 |
| 两条 HG 状态广播 | `01102018-55dd-5073-eeea-0b4b000001c1` | Unitree / humanoid / PID 3396 |

两者 GUID 不同，不能合并成同一个 DDS 参与者。相同的自报进程名/PID 不是远程可执行文件、物理控制板或固件版本的核验。上一轮仅配对 sport 应答参与者；本轮进一步找到状态发布参与者，不能由前者没有状态主题推出整台节点不发布 HG 状态。

HG SportModeState 的官方结构只有上述四个模式/任务字段，**不含运动固件、脚底接触、双脚载荷或柜体支撑力**。

## 官方只读入口与固件缺口

| 入口 | 软件覆盖与本机结果 | 当前条件 |
| --- | --- | --- |
| GetFsmId / GetFsmMode，7001 / 7002 | 已接入；最终状态均成功返回 0 | 模式编号不能独立证明姿态或平衡 |
| GetBalanceMode / Swing / StandHeight / Phase，7003–7006 | 已接入；此前 7301，本轮重查 7003 仍 7301 | 错误不替换成数值 0 |
| ServiceList / CheckMode | 已接入；此前 ai_sport 开启，最终 CheckMode 为 `ai` | 服务开启/模式名不能证明内部平衡就绪 |
| GetPkgVersion，robot_state 1006 | 此前当前应答端返回 3203；本轮未重复调用 | 不支持应答不能推导实际包版本 |
| GetMimicMotion，sport 7100，固定 FSM 550 / API 1 | 找到官方 getter，**未调用** | 官方 C++ 要求稳定状态（例如站立）；当前起点、支撑和承重未核定，不开放任意 FSM API |
| basic_softlimit / basic_clearoip | 仅 DDS 发现，**未调用** | 未找到官方 G1 读取契约、参数与副作用说明，不猜测私有命令 |

官方 [错误定义](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_error.hpp)将 7301 定义为 **LocoState not available**，未提供服务端判定代码。官方 [G1 客户端](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_client.hpp)给出 GetMimicMotion 的稳定状态要求；[API 编号](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_api.hpp)不能代替实机支持证明。

Orin 七个目录根的有界文本/日志尾部检查：**202 项文件、41 项文本、1,096,978 字节**，没有匹配的数值版本候选，无读取错误；两处 OTA 备份深层目录因深度上限跳过。未解码不透明文件、未检查 `.161` 文件系统，不能称作穷尽所有安装包或缓存。证据：[文件搜索](../g1_console/assets/safety_evidence_sources/humanoid_files_20261021_v30.json)。

`device_motion_firmware`、`exact_7301_predicate`、`balance_ready` 继续为空。App v1.5.6、PC4 模块版本、BMS 版本、LowState version `[0,0]`、SDK 协议版本均不代填运动固件。

## 23 自由度起身、姿态保持与平衡

官方 23 DOF ≥8.4.2.222 手册（官方 PDF 未随公开仓库发布，请从宇树官网获取）与 23 DOF >8.6.0.0 手册（官方 PDF 未随公开仓库发布，请从宇树官网获取）存在不同起身路线；本机运动固件未知，不能选择其中一条作为已适用路径。

| 功能 | 官方入口 | 本机尚缺的条件 |
| --- | --- | --- |
| 站立预备 / 姿态保持 | C++ StandUp → FSM 4 | 椅坐预备不等于靠柜坐地；本机保持、退出和“锁定站立”映射未核定 |
| 蹲姿 / 蹲站转换 | C++ Squat → FSM 2；Python 双向转换 → FSM 706 | 不能混同；当前坐地不等于允许的蹲姿，转换方向/承重未知 |
| 躺卧 / 自适应起身 | Python Lie2StandUp → FSM 702；新版手册自适应起身 | 新版与 702 映射未知；靠柜坐地不满足已核对的躺卧起点 |
| 行走控制 | Start → FSM 500 | 不是任意坐姿起身入口；交接和独立站稳未核定 |
| 站立平衡 / 持续踏步 | API 7102，balance 0 / 1 | 7003 仍 7301；持续踏步可能移动双脚 |
| 阻尼 / 零力矩 | FSM 1 / 0 | 不等于主动托举或独立平衡 |

最近人工描述为坐地、双脚贴地、靠柜，属于历史观察，本轮未重新确认接触。**双脚、坐地处、柜体各自承重，以及慢速起身速度/时长和防摔范围仍未知。** 不由摄像头、IMU、关节角、里程计或 FSM 代算载荷；全部 9 项研究模式继续 `device_applicable=null, execution_allowed=false`。

软件 StopMove、阻尼保护与独立硬件断电急停不同；软件停止不保证打断所有起身动作。实体急停、失联响应和停止时延仍需本机证据，不能宣称软件完全替代实体安全通道。

## 软件与验证

- 新增两项只读审计脚本和来源校验器。仅固定 getter；被动 RTPS 不保留载荷，不执行官方二进制，不申请 lease。
- 软件遥控器新增“humanoid 两条状态链路 · 官方读取入口覆盖”，展示采样时间、两个来源、两条 HG 流、固定查询和六类入口。历史证据与实时身体状态分别展示。
- 基础档案 v26，ID `b70ee76711b5e9d1f995614b6fbd16ed21ac6282c092a16ab74ed90a1e154df2`。**253 项实机电机配置字段仍为空**，未用示例值/默认值补齐。
- 后端 **267 项测试通过**，前端 **7 项测试通过**，TypeScript/Vite 构建通过；保留既有 httpx 弃用与 BodyView chunk 大小提示。测试范围为 `pytest tests -q`，不宣称整个旧训练项目测试通过。
- 发布 **485 个文件 SHA256 全匹配**。仅更新本项目控制台，官方 master、OTA、视频、MegaFlow 四个进程的 PID/argv/启动时钟与本轮部署前基线相同。
- 控制台保持人工遥控功能已启用（`G1_READ_ONLY=0, G1_MANUAL_REMOTE=1`），**本轮无控制者、未授权运动、自主导航 idle**；不能称整个服务强制只读。动作目录未扩权，`full_remote_equivalence=false`。
- Mac 浏览器已验证新面板，保存截图。没有点击动作、模式切换、控制权、停止或授权按钮。

**22:21:19 最终只读状态**：23 关节在线，遥测帧龄 4 ms，FSM 0；7001 / 7002 / CheckMode 最新值分别为 0 / 0 / ai。各自最近 60 秒为 **119/119、20/20、20/20 成功**，无超时/错误。相机最近 60 秒 **300/300**，本次运行 **943/943**，无超时，帧龄 251 ms。这些仅证明采样窗口内读取链路，不证明独立站稳或长期健康。

发布后运行 ID `03174dd00ad5`，命令历史为空；事件只有相机连接、连接、状态观察，无动作接受、控制权取得、运动授权、导航启动或软件停止。历史日志保留一行既有格式异常，没有改写历史。

下一步需要宇树提供与 `.161` humanoid 构建匹配的只读版本接口、7301 服务端判定及 23 DOF 起身/姿态保持/平衡切换说明。证据不足前不通过切换模式或试动作来探测条件。实际运动、慢速起身和实体遥控全功能等效均未验证。

Mac 控制台核查面板（截图 / 照片未公开，仅本地存档）

验收：[部署前基线](G1_HUMANOID_PREDEPLOY.json)、[Orin 发布与无动作核验](G1_HUMANOID_ORIN.json)、[最终身体状态](G1_HUMANOID_FINAL_STATE.json)、[验收汇总](G1_HUMANOID_VERIFICATION.json)。
