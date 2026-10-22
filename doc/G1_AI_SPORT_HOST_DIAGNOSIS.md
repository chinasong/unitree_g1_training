# ai_sport 运动节点、7301 与软件遥控器核查

2026-10-21，北京时间。发布 **`1ff38c7cee16`** 已部署至 Orin，Mac 入口为 [G1 软件遥控器](http://127.0.0.1:18765/)。

**新确认了运动接口应答的网络发送节点及 ai_sport 的开启状态；实际运动固件版本、7301 的内部触发条件仍未知。** 本轮仅使用固定只读查询、被动 RTPS 头部和本机进程信息，没有用切换模式或服务试错。没有形成适用于靠柜坐地姿态的已核定起身序列；软件遥控器尚未完全替代实体遥控器。

## 本机 ai_sport 与应答来源

北京时间 **21:08:47–21:09:11**，进行 24 秒被动观察和 10 项固定查询：sport API 1、7001–7006，motion_switcher 1001，robot_state 1003、1006。

- 通过请求 ID/API、应答 publication handle、DDS 发现样本中的 publication instance handle 与 participant GUID，再匹配被动 RTPS 头部中的 GUID 前缀和源 IP，10 项应答均映射到观测到的网络发送节点 **`192.168.123.161`**。每项查询的 1 秒窗口各有一份配对应答。
- 捕获 **240 个有效 RTPS 头部**，无捕获错误；仅保留 20 字节头部派生字段，不保存网络中的应用负载。
- 这条链只证明网络应答来源，**不证明物理控制板、可执行文件身份或安装版本**。Orin 当前 eth0 为 `192.168.123.164`、wlan0 为 `<ORIN_IP>`；有界本机进程枚举没有找到名为 ai_sport/sport 的进程，不能排除重命名或不可读进程。
- robot_state 的 ServiceList 成功返回 **28 项**，字段只有 `name/status/protect`。`ai_sport.status=0, protect=0`。官方 [G1 RobotStateClient 文档](https://support.unitree.com/home/en/G1_developer/robot_state_client_interface)定义回读 **status 0 为开启、1 为关闭**，与 ServiceSwitch 的输入枚举不同；protect 数字含义仍未知。

原始证据：[10 项配对应答及网络头部](../g1_console/assets/safety_evidence_sources/sport_host_20261021_v28.json)、[官方契约归档](../g1_console/assets/safety_evidence_sources/sport_service_contract_20261021_v28.json)。

## 7301：已证条件与未证条件

| 项目 | 本机结果 | 判定范围 |
| --- | --- | --- |
| ServiceList / ai_sport | 开启，原值 0 | 服务开启不等于运动状态可读取 |
| sport 7001 / 7002 | code 0，均为 `{"data":0}` | FSM ID/模式的接口值；不是承重或站稳证据 |
| sport 7003–7006 | 均为 7301 | 平衡模式、摆腿高度、站立高度、相位没有有效值 |
| CheckMode | `form=0, name=ai` | 控制模块回读，不证明已执行某项官方动作 |
| sport API 1 | `1.0.0.0` | 服务协议版本，不能替代运动固件 |
| robot_state 1006 / GetPkgVersion | 3203 | 本接口不支持；未取得包版本 |

官方 [G1 错误定义](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_error.hpp)只把 7301 定义为 **LocoState not available**，没有给出服务端内部判定实现。FSM 0、ai_sport 开启与 7301 在本窗口同时出现，仅为相关性；尚未证明 FSM 0 是充分或必要条件，也不能断言启动 ai_sport 就能消除错误。内部状态未就绪、状态发布链路、版本兼容性等仍是待证假设。

本轮配对查询没有 3104；7301 是已收到的错误应答，与未收到应答不同。所有非零返回中的占位值被丢弃，有效数据保持未知，不回退成 0。

## 实际运动固件与 23 自由度适用性

**实际 ai_sport 运动固件仍为 null**。App 关于设备 v1.5.6、硬件 v1.0、PC4 的 master/patch/video 版本、SDK 协议版本及 OTA 缓存包标签均不代填。

官方 23 DOF 旧版手册（≥8.4.2.222）（官方 PDF 未随公开仓库发布，请从宇树官网获取）和新版手册（>8.6.0.0）（官方 PDF 未随公开仓库发布，请从宇树官网获取）的起身路线不同。固件未知，无法选择本机分支。九项模式的接口、前提、反馈和缺口已保留在控制台：

| 功能 | 已核对接口 | 本机适用结论 |
| --- | --- | --- |
| 站立预备 / 姿态保持 | C++ StandUp → FSM 4 | 靠柜坐地不等于官方椅坐预备，进入/退出与保持参数未知 |
| 蹲姿与蹲站转换 | C++ Squat → FSM 2；Python 双向转换 → FSM 706 | 两者不能混用；坐地不等于蹲姿，方向和支撑条件未核定 |
| 躺卧起身 / 新版自适应起身 | Python Lie2StandUp → FSM 702；新版手册⑩ | 未证明新版⑩与 702 等价，当前姿态不满足已核对的躺卧起点 |
| 进入行走控制 | Start → FSM 500 | 不是任意坐姿起身入口 |
| 站立平衡 / 持续踏步 | API 7102，balance 0/1 | GetBalanceMode 仍为 7301；踏步可能移动双脚，不能当位置锁定 |
| 阻尼 / 零力矩 | FSM 1 / 0 | 不构成主动托举或独立站稳证明 |

最近人工报告为坐地、双脚贴地、靠柜，属于历史人工观察。当前仍无脚底、坐地处和柜体各自承重的有效测量，不能从关节角、FSM 或画面推算成已证载荷。没有已核定的可调慢速起身参数，所有九项模式 `device_applicable=null, execution_allowed=false`。

StopMove 的零速度目标、无线遥控器阻尼保护和独立硬件断电急停的作用范围不同；本机硬件急停、时延、失联策略仍未核定。软件停止不能保证取消所有起身或动作过程。

## 软件与基础模型更新

- 新增结构化服务状态表，实时查询结果、时间和原始值可查看，ai_sport 单独标识；缺失或非法字段保持未知，不用默认值。
- 新增“ai_sport 服务与应答来源 · 7301 条件证据”面板，显示历史采样时间、网络来源、10 项原始查询与判定边界；历史证据不冒充实时状态，也不授予动作权限。
- 基础档案升至 **v24**，来源 SHA256 校验；**253 个未知实机电机配置字段仍为空**。`safety.execution_allowed=false`。
- 软件目录现有 **60 个条目、13 项只读查询**；这不是 60 种动作，也不是所有物理遥控功能已经等效。

ai_sport 与 7301 证据面板（截图 / 照片未公开，仅本地存档）

实机服务列表回读（截图 / 照片未公开，仅本地存档）

## 部署与验证

- **248 项 Python 测试、7 项 Node 测试通过**；TypeScript/Vite 构建通过。保留既有 httpx 弃用提示和 BodyView 大 chunk 警告。
- **482 个发布文件的 SHA256 全部匹配**；基础档案 ID 为 `742d9210f46f9a15c08f8011b1cf38d3074381af5ee0a1bddc576ee7e5e7ab51`。
- 仅重启本项目控制台并恢复只读订阅；四个官方服务的 PID、启动时钟和系统 boot ID 均未变化。控制台保留人工遥控部署模式，但本轮 **无控制者、未授权运动、自主执行关闭**，不把它宣称为全局强制只读。
- 新发布当前运行 ID `c599168b4781`。7 次固定只读功能查询加浏览器 ServiceList/GetBalanceMode 两次，共 **9 次**：3 次有效、5 次 7301、1 次 3203，**0 次 3104**。前述 10 次原始 DDS 查询发生在更新前，单独归档，不混入新运行次数。
- **21:21:17** 最后状态：23 关节在线、FSM 0、遥测帧龄见原始快照、无控制者、未授权运动；相机最近 60 秒 **300/300**，零超时/错误，总计 1133/1133 成功，帧龄 101 ms。只能证明订阅/取图链路，不能证明独立站稳。
- 本轮日志无 `command_accepted`、`control_acquired`、`manual_armed`、`navigation_started` 或 `stopped`；未执行动作或模式切换，未改 Quantumult X。历史事件文件有 1 行既有格式异常，未改写历史日志。

原始验收：[部署核查](G1_AI_SPORT_HOST_ORIN.json)、[七项功能查询](G1_AI_SPORT_HOST_QUERY_VERIFICATION.json)、[最后状态及九项历史](G1_AI_SPORT_HOST_FINAL_STATE.json)、[验收汇总](G1_AI_SPORT_HOST_VERIFICATION.json)。

## 剩余缺口

还需要与应答节点匹配的 **ai_sport 实际版本字段**及 **7301 服务端判定说明**。官方 issue [#43](https://github.com/unitreerobotics/unitree_sdk2_python/issues/43)曾提到 App 服务状态页的 ai_sport 组件版本，但那是其他机器的历史报告，不能抄作本机版本；已有“关于设备”截图也不能替代组件字段。没有该证据前不猜私有配置 API、不切换服务、不采用其他 29 DOF 机器的启动序列。

在固件、起点姿态、实际承重、模式交接与停止机制核定前，起身、姿态保持和平衡切换继续锁定，不能声称已经得到“缓慢站稳且不摔倒”的可执行方案。
