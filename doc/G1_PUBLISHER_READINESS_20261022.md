# G1 运动板版本来源与 7301 就绪核查

2026-10-22，Asia/Shanghai。最终状态复核时间 **12:51:31**。

**版本公告、查询应答和运动状态流已配对到运动板 `192.168.123.161`；`humanoid` 实际运行构建和 7301 的服务端判定条件仍未知。** 软件窗口控制权、三类请求核对可用，机器人运动权限和实机动作尚未验证；不能宣称已完全替代遥控器。

## 本轮实机证据

采样 **12:38:55–12:44:25，共 330.213 秒**。通过每条样本的 `publication_handle` 配对 DDS 发布者，再通过参与者 GUID 前缀配对被动接收的 RTPS 源 IP。只保留 RTPS 头部统计，不保存原始网络载荷。进程名、PID 和主机名是 DDS 自报，IP 配对不是二进制签名。

| 证据 | 实际来源 | 本轮结果 | 能确认的范围 |
| --- | --- | --- | --- |
| `rt/servicestate` | `.161`，自报 `robot_state_service` PID 3113 | 12:40:35.826 收到 `ai_sport=8.7.5.212`，字段与服务名在同一对象中 | 确认公告来源及内容；没有确认 `humanoid` 的运行构建 |
| `rt/api/sport/response` | `.161`，自报 `humanoid` PID 3359 | 公告前后两轮 GetFsmId/GetFsmMode 均 code 0、值 0；GetBalanceMode/GetStandHeight 均 7301、值未知 | RPC 服务在应答；错误不是连接超时 |
| `rt/lf/sportmodestate` | `.161`，自报同一 `humanoid` PID，不同 DDS participant | **4,930 帧**；观测值始终为 FSM 0、mode 0、task_id 4、task_time 0 | 持续状态广播存在；task_id 4 的本机语义未知，不能解释为 StandUp 或站稳 |
| `motion_switcher` GetSilent | `.161`，自报 `motion_switcher_service` PID 3009 | 两轮均 code 0，`silent=false` | silent 标志为 false；不证明 SDK 已取得运动权限 |

共 **10 次只读 RPC，零 3104**，不含后续浏览器查询。所有请求 lease_id 为 0、参数为空；没有申请 SDK 运动 lease。公告载荷 SHA256 为 `c31dde1408172e046585859481d0ff67c2a874bacbb461f2c7ebbd813245b4c6`。

原始配对记录见 [service_publisher_20261022_v33.json](../g1_console/assets/safety_evidence_sources/service_publisher_20261022_v33.json)，机器可读验收见 [G1_PUBLISHER_READINESS_20261022.json](G1_PUBLISHER_READINESS_20261022.json)。两者分别记录采样时发布和最终部署，不混用发布 ID。

## 7301 与实际控制权

官方 [G1 错误定义，固定提交](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/g1/loco/g1_loco_error.hpp)将 7301 定义为 LocoState 不可用；该文件没有发布服务端就绪判定。**FSM 0、持续 HG 广播和 silent=false 同时存在，仍不足以使本机平衡与站立高度查询成功。** 这是本轮观测，不能反推所有固件都相同，或断言某一个模式切换必然修复。

本轮官方组织代码检索中的 LocoState 结果是错误常量、客户端或示例，没有在这些结果中发现服务端判定实现；这不是对全部闭源程序的结论。浏览器再次查询 GetBalanceMode 返回 **7301**，GetPkgVersion 返回 **3203（不支持此 API）**，没有取得固件版本。服务协议版本、App 软件版本和服务公告均不替代运行构建。

| 权限层 | 核验状态 |
| --- | --- |
| Orin SSH、遥测与只读 RPC 通路 | 已确认 |
| 本项目窗口独占、心跳、释放 | 已实际核验；最终 owner 为空 |
| 官方 SDK 运动控制权或运动 lease | 未申请、未确认 |
| 固件接受起身、姿态保持、平衡写入 | 未测试 |
| 独立站稳、承重分布、防摔边界 | 未确认 |

## 起身、姿态保持和平衡切换

以下是已核查 SDK 的请求映射与官方文档候选条件，**不是本机已批准的动作路线**。固定 Python SDK 提交为 `814556d15970dd2ecf1c9984e845ca02ab07e206`；C++、Python 和新固件命名的差异继续保留。

| 入口 | 已核查请求 | 本机尚缺证据 |
| --- | --- | --- |
| 站立预备 / 姿态保持 | C++ StandUp → SetFsmId：7101，`data=4` | 本机模式对应、力矩保护、进入及退出条件；不等于从靠柜坐地自动起身 |
| 躺卧起身候选 | Python Lie2Stand → 7101，`data=702` | 新手册 AdaptRise 与该接口的对应关系、规定躺姿、空间和恢复条件；当前靠柜坐地不是规定躺卧 |
| 蹲姿起立 | Python Squat2Stand / Stand2Squat → 7101，`data=706` | 双向切换的本机方向判定、当前状态、完成与中断条件；坐地不等于蹲姿 |
| 平衡模式 | C++ BalanceStand → SetBalanceMode：7102，`data=0` | 本机模式接受条件、已站立状态、支撑与退出条件；该接口不能用来证明自动起身 |

官方 23 DOF 遥控手册（2026-04-07 文件）（官方 PDF 未随公开仓库发布，请从宇树官网获取）分别列出阻尼、锁定站立、躺卧起立、坐姿、蹲姿起立和适应起身；椅坐流程包含人工扶正。手册没有证明当前靠柜坐地适用，也没有给出可确认的起身减速参数。不能把通用 duration、示例增益或当前 task_id 当作缺失条件。

继续核查了官方仓库中的社区 [PR #312](https://github.com/unitreerobotics/xr_teleoperate/pull/312)：本轮检查仍 open、未合并，head `1123c078e4902a6726b56aa7851c5b36cc1031ed`。它提出 FSM 1→4→501，报告版本是软件 1.5.3，没有对本机 23 DOF / ai_sport 8.7.5.212 的验证。相关 [29 DOF issue #170](https://github.com/unitreerobotics/unitree_sdk2_python/issues/170)的 10 月 2 日跟进报告 501 摇晃、200 踏步，没有提供可复用的成功轨迹。本轮未采用这些模式值、构造器切换或 lease 试探。

## 实现、部署与验证

- 新增有界只读发布者审计脚本与严格证据投影；拒绝发布句柄不匹配、来源不匹配、错误载荷冒充值、运动 lease 或权限升级记录。
- 基础档案保存带时间的历史来源证据，`current=false`；实际运动固件、7301 精确条件仍为空，**253 个未知实机电机配置字段仍为空**。
- Mac 界面增加“运动板发布来源核验”和四类模式适用条件；明确区分公告与运行构建、窗口控制权与机器人运动权限。
- Orin 发布 **`5e60ab48bcfa`**，**491 个文件 SHA256 一致**，run `d228a46f6764`，启用 `manual_remote=true, verify_only=true`。Mac 界面：<http://127.0.0.1:18766/>，SSH 转发：<http://127.0.0.1:18765/>。
- Python 定向 **103 项通过**；TypeScript / Vite 构建通过，保留既有 chunk 大小警告。没有据此宣称实机动作测试通过。
- 浏览器实际读取 silent=false、7301、3203，核验窗口获取、心跳与释放。12:48:12、12:48:20、12:48:29 分别完成蹲姿起立、姿态保持、平衡模式请求核对：参数有效，**三项 dispatched=false**，固件接受未知、物理结果未测试。
- 最终连接在线、23 关节反馈、遥测龄 22 ms；无窗口控制者、未授权运动、导航空闲。采样和部署前后官方 Orin 服务进程及系统 boot ID 未变化。

运动板发布来源核验界面（截图 / 照片未公开，仅本地存档）

## 剩余阻塞与可继续核验项

实际运动权限与三类动作仍未打通。下一步需要与本机匹配的官方运行构建读取途径、LocoState 就绪判定和 23 DOF 模式对应关系，才能选择合法起始姿态及退出/保护流程。目前不能把“查询成功”“进入某 FSM”或“脚底贴地”当作站稳证据。

已整理给宇树的具体问题，**仅存于本地，未对外发送**：

1. 哪个只读接口能读取当前 `humanoid` 运行构建，它与 `ai_sport=8.7.5.212` 公告如何对应？
2. 本机 FSM/mode=0、HG 状态持续发布且 silent=false 时，哪些内部条件导致 7003/7005 返回 7301？有无只读就绪字段？
3. 本机 23 DOF 的 FSM 4/702/706、7102 `data=0` 与新版 AdaptRise 的官方映射是什么？
4. 支持哪些起始姿态、外部支撑、完成回读、时长控制和中断保护？靠柜坐地是否有适用路径？

最近人工姿势信息为坐地、双脚贴地、靠柜；本轮没有独立测得双脚、坐地处及柜体载荷。未执行运动、模式或声光写入，未操作 Quantumult X，未修改系统配置，未重启机器人。
