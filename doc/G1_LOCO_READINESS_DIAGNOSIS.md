# G1 7301、协议版本与起身条件复查

2026-10-21，北京时间。本轮发布 **`861f842d8d59`** 已部署至 Orin；Mac 入口为 [软件遥控器](http://127.0.0.1:18765/)。上一轮接收路径诊断保留在 [G1_RPC_STATE_DIAGNOSIS.md](G1_RPC_STATE_DIAGNOSIS.md)。

## 2026-10-22 补充：7301 可能是被动 FSM 下的预期拒答（待验证假设）

北京时间 2026-10-22 补充，只读整理已有证据，未发送任何查询或写入。

| 证据 | 内容 |
| --- | --- |
| 同码异名 | 锁定 SDK [h2_loco_api.py](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/h2/loco/h2_loco_api.py) 中 `LOCO_ERR_LOCOSTATE_NOT_AVAILABLE` 与 `LOCO_ERR_SCHEDULER_STATE_NOT_AVAILABLE` 同为 **7301**。G1 Python 客户端未列错误码；该别名是 H2 定义，用于 G1 属推断 |
| FSM 0 | 2026-10-21 v27 与 2026-10-22 v36（[installed_loco_sdk_20261022_v36.json](../g1_console/assets/safety_evidence_sources/installed_loco_sdk_20261022_v36.json)）均在 FSM 0 下得到 7003–7006 = 7301 |
| FSM 1 | 唯一阻尼测试后 [after_queries.json](evidence/damping_test_20261022/after_queries.json) 中 GetFsmId=1 时，GetBalanceMode、GetStandHeight 仍为 7301 |
| 未覆盖 | 项目从未在 FSM 2/3/4/500/702/706 等站立或运动模式下读取 7003–7006 |

**假设：**7003–7006 读取的是运动调度器（LocoState / Scheduler）的状态；零力矩（0）和阻尼（1）这类被动模式下调度器未运行，7301 是预期拒答，不是独立故障。若成立，“7301 消失”不能作为起身前置条件，否则形成“须先站立才能读到、读到才允许站立”的循环。

**可证伪实验（需用户决定，涉及实体动作）：**在吊架或有人扶持的保护条件下，由人用**实体遥控器**按官方手册让机器人进入站立 / 行走；本项目只做固定只读查询 7001–7006 并记录时间与原始应答，不取得控制权、不发写入。

- 站立 / 行走模式下 7003–7006 返回 0 且有实值 → 支持假设，7301 降级为“模式不适用”提示。
- 仍返回 7301 → 否定假设，7301 与运动固件 / 私有就绪条件相关的判断继续成立。

该假设不改变现有门控：起身、平衡、行走在本机适用条件核实前保持锁定。

### 对照脚本 `scripts/compare_g1_locostate.py`

在 Mac 上经 SSH 桥运行，只访问 `GET /api/session`、`GET /api/state` 和 `POST /api/command`（仅限 `get_fsm`、`get_fsm_mode`、`get_balance`、`get_swing_height`、`get_stand_height`、`get_phase`）。不取得控制权、不发送写入；只读查询在当前 `--damping-test` 范围内无需控制权。

```bash
.venv/bin/python scripts/compare_g1_locostate.py --duration 300 --interval 2 --note "保护方式 / 操作人 / 遥控器步骤"
```

- 每个采样点先读服务状态，再依次查询 6 项，最后再读一次状态；只有三处 FSM 一致的样本计入结论，模式切换途中的样本单列为不稳定。
- 7301 等非零返回一律记为未知值，不采用返回中的占位 0。
- 原始记录写入 `.g1-deploy/research/locostate-compare-*.jsonl`：首行为运行条件，末行为按 FSM 分组的统计与结论 `supports` / `refutes` / `inconclusive`。
- 实验流程：先启动脚本，在 FSM 0 下采几个基线样本；随后由人在保护条件下用实体遥控器按官方手册切换模式，每个模式保持至少 10 秒；结束按 Ctrl-C 或等时长结束。

定向测试 `tests/test_g1_locostate_compare.py` 使用真实控制台服务（`manual_remote + damping_test`）和录制型替身机器人，验证只发出上述 6 项查询、无控制者、7301 占位值不进入结果，以及三种结论判定。

**2026-10-22 16:54 试跑基线**（机器人静止，无人操作遥控器）：3 个稳定样本，FSM 0，7003–7006 均为 7301，结论 `inconclusive`；试跑后 run_id `a55319b05192` 不变、无控制者、未授权运动，服务命令记录只有只读查询。原始记录 `.g1-deploy/research/locostate-compare-20261022-165430.jsonl`。

## 本轮结论

**7301 仍存在，运动固件版本仍未知。** 本轮固定查询与配对应答已经进一步排除“没有收到服务回复”和采样窗口内的多份应答冲突；尚未确定 LocoState 不可用的内部触发条件。不能为读取成功而尝试切换模式、申请运动租约或重启官方服务。

新增运动、模式切换和设备状态服务的协议版本查询，实机与图形界面均已核验。软件目录现有 **60 个条目、13 项只读查询**，包含查询、通用与具名动作入口，不表示 60 种动作或全部遥控功能已经等效。

本轮没有取得控制权、授权运动、执行动作、切换模式或修改声光；Quantumult X 未操作。人工遥控服务开关仍开启，但保持无控制者、未授权运动；自主执行继续关闭。

## 7301 的应答证据

北京时间 **20:33:25–20:33:38**，直接进行 33 次固定只读查询：三项 API 协议版本、7001–7006、CheckMode 和 GetPkgVersion，各三次。7003–7006 共 12 次返回 7301；没有 3104。API 7001/7002 返回 0，CheckMode 返回 `form=0, name=ai`，不能据此推定平衡启用或机器人独立站稳。

北京时间 **20:34:37–20:34:54**，另对 7 个 sport 查询记录配对 request id、api id 和 DDS publication handle：

- 发现三个 sport 应答发布者；每次查询在各自 1 秒窗口内只收到同一个发布者的一份配对应答。
- 该发布者 GUID 为 `01102018-14ee-e682-8024-7c60000001c1`；GUID 不能证明物理控制板、进程归属或固件版本。
- 7003–7006 在 **0.86–1.35 ms** 内明确返回 7301；本窗口没有观察到多份应答冲突，不代表排除其他时段的冲突。
- `rt/servicestate`、`rt/multiplestate`、`rt/servicestateactivate` 的 8 秒被动读取均无样本。只证明本窗口没有取得这些 String 样本，不能证明整个系统没有服务版本信息。

官方 [G1 错误定义（固定提交）](https://github.com/unitreerobotics/unitree_sdk2/blob/69c04316d81ac244c314e730106e24787b3e61b8/include/unitree/robot/g1/loco/g1_loco_error.hpp)将 7301 定义为 LocoState 不可用。非零返回中的 `0`、`0.0`、`[]` 仍被丢弃；平衡模式、摆腿高度、站立高度和相位保持未知。

原始证据：[固定查询](../g1_console/assets/safety_evidence_sources/loco_readiness_20261021_v27.json)、[应答发布者](../g1_console/assets/safety_evidence_sources/loco_responders_20261021_v27.json)。第一次探测的 before/after 顶层 FSM 为空是取字段路径错误，不是实测 FSM 缺失；有效 FSM 证据来自配对的 7001 应答，后续脚本已修正路径。

## 协议版本不能替代运动固件

官方 [RPC 内部定义](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/rpc/internal.py)的 API 1 是服务协议版本。它返回纯文本，不能按 JSON 对象强行解析。新入口严格校验版本格式，错误回复不提供有效数据，也不回退旧接收路径。

| 服务 / 读取项 | 本机返回 | 能证明的范围 |
| --- | --- | --- |
| sport / API 1 | `1.0.0.0` | 运动服务协议版本 |
| motion_switcher / API 1 | `1.0.0.1` | 模式服务协议版本 |
| robot_state / API 1 | `1.0.0.2` | 设备状态服务协议版本 |
| robot_state / GetPkgVersion | 三次 `3203` | 当前服务不支持此 API；未取得包版本 |
| 运动主机固件 | **未知** | 没有可核验的实际版本值 |

App“关于设备”的 v1.5.6、PC4 模块安装版本、LowState `[0,0]` 和这些服务协议版本均未写成运动固件。GetPkgVersion 的 3203 与先前 OTA 工具超时单位错误造成的 104 是不同问题。运动固件字段及对应的 23 DOF 手册分支仍为空。

官方仓库的 [issue #170](https://github.com/unitreerobotics/unitree_sdk2_python/issues/170)报告了协议匹配但切换/速度应答与行为不一致的情况，环境是 **EDU+ 29 DOF / 软件 1.5.3**，并非本机结论。[PR #312](https://github.com/unitreerobotics/xr_teleoperate/pull/312)尚未合并，提议调整启动 FSM 序列；独立审阅指出自动启动、错误处理和验证问题。本轮没有采纳或执行该启动序列，也没有开放其 FSM。见 [社区线索归档](../g1_console/assets/safety_evidence_sources/loco_community_compatibility_20261021_v27.json)。

## 起身、姿态保持与平衡切换的适用边界

官方 23 DOF 旧版手册（≥8.4.2.222）（官方 PDF 未随公开仓库发布，请从宇树官网获取）与新版手册（>8.6.0.0）（官方 PDF 未随公开仓库发布，请从宇树官网获取）的起身路线不同；本机分支尚未匹配。接口参数、进入条件、反馈与缺口已在软件遥控器的九项模式表中保留：

| 路径 | 本轮适用结论 |
| --- | --- |
| StandUp / FSM 4，Squat / FSM 2，Sit / FSM 3 | 接口定义已核对，当前靠柜坐地的进入路线未知，继续锁定 |
| Squat2StandUp / StandUp2Squat / FSM 706 | 转换方向依赖状态；坐地不能当作蹲姿，继续锁定 |
| Lie2StandUp / FSM 702；新版自适应起身 | 未证明适用于当前姿态，也未匹配本机固件，继续锁定 |
| Start / FSM 500 | 行走控制入口，不证明能从当前坐姿起身 |
| BalanceStand / balance 0；ContinuousGait / balance 1 | 只读回读仍为 7301，不能推定已启用或试切换 |
| Damp / FSM 1；ZeroTorque / FSM 0 | 不能用作全身主动托举或站稳证明，继续锁定 |

最近人工报告为靠柜坐地、双脚贴地，属于有时间边界的人工观察；不等于实时测得的脚底、坐地处或柜体承重。手册要求的平整硬地、规定躺卧/蹲姿及椅坐辅助启动，不能直接套给靠柜坐地。约 5 秒动作描述不是已核定的可调起身速度；`SetVelocity.duration` 也不能替代起身减速参数。当前仍无法给出已核定的“缓慢站稳”可执行序列。

软件 StopMove 是零速度目标，不保证取消起身、手势或舞蹈，网络断开也不保证送达。遥控器阻尼保护是控制模式切换；本机独立硬件断电急停、保护时延与失联策略仍未知。详见上一轮报告的停止边界说明。

## 实际部署与验收

北京时间 **20:42:51–20:42:53**，发布 `861f842d8d59`，运行 ID `cb9ab226b9cf`，13 项入口各三次，共 **39 次只读查询**：

| 查询 | 次数 | 实际结果 |
| --- | ---: | --- |
| GetFsmId / GetFsmMode | 各 3 | code 0，值均为 0 |
| 平衡 / 摆腿高度 / 站立高度 / 相位 | 各 3 | 均为 7301，有效数据保持未知 |
| GetPkgVersion | 3 | 均为 3203，有效数据保持未知 |
| 三个服务的协议版本 | 各 3 | code 0，版本如上，`firmware_version=null` |
| ServiceList / GetActionList / GetVolume | 各 3 | code 0；28 个服务、20 个数字动作 ID 与 4 个命名条目、音量 41 |

39 次中 **零 3104**；15 次 SDK 拒绝在 HTTP 返回 502，历史记录保留 `read_unavailable` 和实际 SDK 码。浏览器又分别读取了三个协议版本，结果一致；当前运行合计 **42 次只读查询：27 次有效回读、15 次明确拒绝**。未执行动作来验证目录中的运动项。

- **239 项 Python 测试、5 项按住控制 Node 测试通过**；TypeScript/Vite 构建通过。既有 3D chunk 大小警告保留。
- **480 个发布文件 SHA256 全部一致**；基础档案 **v23**，253 个未知实机电机配置字段仍为空，`safety.execution_allowed=false`。
- 20:45:34 核查四个官方服务的 PID、启动时钟与系统 boot ID 均未变化；仅更新本项目控制台。旧控制台退出时记录 5 个 semaphore 清理警告，未宣称该既有清理问题已修复。
- 20:46:09 状态快照：23 关节在线、FSM 0、无控制者、未授权运动、导航 idle；相机最近 60 秒 **300/300**，零超时/错误。这只验证身体订阅和取图链路，不能证明承重或独立站稳。

验收：[查询原始记录](G1_LOCO_READINESS_QUERY_VERIFICATION.json)、[部署与进程核查](G1_LOCO_READINESS_ORIN.json)、[最后状态快照](G1_LOCO_READINESS_FINAL_STATE.json)、[验收汇总](G1_LOCO_READINESS_VERIFICATION.json)。

软件遥控器中的协议版本与固件边界（截图 / 照片未公开，仅本地存档）

## 剩余工作

仍需实际运动组件的版本证据及对应的官方接口/前置条件，不能以协议匹配或社区 29 DOF 启动序列代替。已有“关于设备”截图无需重复；若 App 的服务状态页提供 ai_sport 等组件版本，应保留字段名、组件名和版本原文。目前已经检查的只读 API 与状态主题没有提供该值。

在匹配固件、核实当前完整姿态和外部支撑、取得有效承重证据并确认停止机制之前，起身、姿态保持、平衡切换及未映射的新版功能继续锁定。软件遥控器尚未完全替代实体遥控器。
