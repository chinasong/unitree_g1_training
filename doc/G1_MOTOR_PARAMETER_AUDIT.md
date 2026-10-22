# G1 电机参数核查

2026-10-21，北京时间（UTC+8）。G1 EDU / 23 自由度 / 无手 / Orin。全程只读，未运行控制示例，未修改系统网络或 Quantumult X。

## 已确认的文档事实

| 项目 | 结果 | 证据与适用边界 |
|---|---|---|
| 当前机型代码 | 4；PR 坐标；23 关节稀疏编号匹配 | Mac 经现有 SSH 隧道读取 rt/lowstate；检查时反馈年龄 33 ms。每次观察重新核对，历史检查不冒充当前状态 |
| 官方机型对应关系 | `g1_23dof_rev_1_0` | 宇树 unitree_ros 的机型表明确绑定 mode_machine=4；用户确认的 23 DOF 配置也匹配 |
| 左右髋俯仰减速比 | 文档值 **14.3:1** | 对应机型的官方表；不是逐电机实测或序号核验 |
| 左右髋侧摆减速比 | 文档值 **22.5:1** | 同上；其余关节未由本版本表给出，不类推 |
| temperature[0] / [1] | SDK 定义：外壳 / 绕组温度 | G1 专用 SDK 保护辅助函数，未把函数默认阈值当成本机固件配置 |
| Orin 已安装软件包元数据 | master_service_pc4 1.0.0.2、unitree_patch_pc4 1.0.0.6、video_hub_pc4 1.0.1.1 | 读取 `/unitree/robot/pkg/version/version.json` 和模块清单；这是 PC4 软件包元数据，不是电机驱动固件，也不证明运行中二进制版本 |

## 对应模型与旧模型的差异

为避免版本混用，新增 23 个关节的对应版本 URDF 数据，旧 RL Gym 数据保留在独立字段。两份模型的关节角度范围相同；以下 6 个关节的 effort / velocity 不同。

| 关节（DDS 编号） | 旧 RL Gym effort / velocity | mode 4 对应 URDF effort / velocity |
|---|---|---|
| 左右髋侧摆（1、7） | 88 N·m / 32 rad/s | 139 N·m / 20 rad/s |
| 左右踝俯仰、踝侧摆（4、5、10、11） | 50 N·m / 37 rad/s | 35 N·m / 30 rad/s |

对应版本 URDF 总质量为 32.10685728 kg，旧 RL Gym 为 34.13385728 kg。二者都是模型质量，不是实机称量。3D 结构继续标为旧参考模型，没有假装已完整替换实机结构。上述 effort / velocity 不填入实机连续或峰值力矩，也不作为动作门槛。

## 已排除的混用

- 官方通用 `motor_info.md` 含多种电机型号，但没有逐项绑定本机 mode 4，且其髋俯仰项为 22.5，与对应版本的 14.3 不同。未用它填本机型号、减速比或额定力矩。
- SDK `motor_casing_overheat` / `motor_winding_overheat` 的默认 85/120°C 是调用者可覆盖的辅助函数参数，不是本机固件阈值、额定工作温度或降额曲线。
- 官方低层示例 Kp/Kd 仍只作示例展示，没有执行示例，也不用于填出厂或当前控制器增益。
- 软件包版本、LowState.version 原始 `[0,0]` 不解释为完整机器人或电机驱动固件版本。

## Orin 只读检查范围与未解决项

检查 `/unitree/etc`、`/unitree/robot`、`/unitree/module`、安装包副本和 `/home/unitree/unitree` 的配置文件名，再对 `/unitree`、`/opt/ota_package` 做深度不超过 8 的电机 / 关节 / 标定 / 零位 / 增益 / 限值文件名筛查，共检查 73 个文件名，未发现匹配项、未遇到目录权限错误、未达到数量上限。已安装官方模块目录只列出管理与视频模块。`master_service.json` 无法按 UTF-8 或 GB18030 解析为 JSON，没有尝试执行或解码厂商二进制来提取参数。

以上是首次检查范围；没有修改固件或发出电机探测命令。后续其他控制板核查见下节。

仍需本机专属资料或官方支持确认：当前/出厂 Kp/Kd，电机型号及序号，驱动固件，连续/峰值力矩与时长曲线，电流限制及转矩常数，其余关节减速比与效率，零位/编码器标定、摩擦和回差，固件热阈值与降额规则，motorstate 位表及 sensor 含义。所有 `actual_config` 仍为 null；不通过试转或探索极限补齐。

## 2026-10-21 其他控制板只读核查

本轮 DDS 观察于北京时间 **09:46:38（UTC+8）** 完成。使用已核验主机指纹的 `unitree@<ORIN_IP>` 登录 Orin，在其 `eth0` 上建立 Domain 0 的发现读取器与应用数据订阅。没有 DDS 应用写入、RPC 调用、控制权接管或标定操作。

| 检查对象 | 实际取得 | 仍不能据此确认 |
|---|---|---|
| Orin | eth0 `192.168.123.164/24`；设备树 `NVIDIA Orin NX Developer Kit`；Ubuntu 20.04.6；L4T R35 revision 3.1 | 不推定内存规格、完整整机型号或运动板固件 |
| 官方文档中的运动主机 PC1 | 邻居地址 `.123.161` 可达；TCP 22 返回 errno 111、拒绝连接；未尝试认证 | 未读取文件系统；不表示其他端口或私有协议不存在 |
| `rt/lf/mainboardstate` | 8 秒订阅窗口收到 **145** 个样本；保留四组原始数组 | 数组通道、单位、状态位及具体物理板身份未核定 |
| `rt/lf/bmsstate` | **145** 个样本；保留版本原码、soc/soh、cycle、电压、电流、温度等原字段 | 未将历史 soc 当当前电量；未猜测缩放、日期编码或把 BMS 版本当电机固件 |
| `rt/lowcmd` | **145** 个样本；最后一帧 CRC 正确、35 协议槽的 Kp/Kd 为零 | 发布者物理身份、驱动是否采纳未核定；零值不是实机增益，35 槽不是已装电机数 |
| `rt/lowstate` | **145** 个样本；最后一帧 CRC 正确，mode_machine=4、mode_pr=0、version=[0,0] | CRC 不是设备身份认证；version 原码不是完整固件版本 |
| `rt/selftest` | 发现发布声明，本次窗口 **0** 样本 | 不能据此认定无自检或健康正常 |
| 配置 / 标定服务 | 发现 config、basic_softlimit、basic_taumax、basic_demarcate 等声明 | 尚无匹配的公开只读方法协议；未猜测 API ID，未发请求 |

DDS participants、发布和订阅端点是软件实体，不是物理控制板数量，也不能凭 dex3 话题认定安装了灵巧手。未核定话题发布者对应的具体物理板；不更改用户确认的 23 DOF / 无手配置。

### Orin 安装目录模型

对 `/home/unitree/megaflow_sdk/ws` 做深度不超过 6 的文件名清查，排除虚拟环境、RoboJuDo 依赖树、缓存等，共 **350** 个文件名；12 个配置候选为 11 个 XML 模型 / 场景和 `bin/cos_config.json`。未复制含账号配置的文件内容。这是有边界的检查，不代表搜索了整个系统或所有控制板。

只读复制 `plugin/g1edu_motion/res/models/g1_description/g1_23dof_rev_1_0.xml`，SHA256 `8ca62fcccdca91a431ca04f1a42f9c2fda241fdd5e13411168dc82de00f978de`。有 23 个对应关节；四个踝关节的 `actuatorfrcrange` 为 ±50，而固定官方对应 URDF 的 effort 为 35。保存差异，没有选一个当实机规格。未证明当前控制器加载此文件，没有把随附模型当标定文件。

本轮新增可追溯的主板 / BMS 历史原码、协议来源和安装文件证据；**新增核定的逐电机配置值为 0**。其余参数仍需运动主机开放的文档化只读接口、本机专属标定导出或宇树对该整机序号的资料，不能由状态流或示例反推出。

## 2026-10-21 不登录运动主机的接口核查

本轮查询于北京时间 **09:58:03（UTC+8）** 完成。通过已核验的 Orin SSH 连接作桥接，在其 eth0 上调用官方 DDS/RPC 元数据 getter；**没有登录运动主机 PC1**。这不等于消除了 Orin 桥接所用的 SSH。

检查官方 G1 Basic Services、Robot State Client、Debugging Specification 正文，以及三个官方仓库固定 HEAD 的源码：SDK2 `d1abf001220d3e5f59e00a0c4aaa595e00089a4a`、Python SDK `814556d15970dd2ecf1c9984e845ca02ab07e206`、ROS2 `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`。源文件清单、检索范围、关键原文及 SHA256 随档案保存；范围不包括逆向二进制、私有 App 协议或所有未公开资料。

| 只读查询 | 实机结果 | 可确认的范围 |
|---|---|---|
| 通用 GetServerApiVersion，内部 API 1 | config / basic_demarcate / basic_softlimit / basic_taumax 均为 `1.0.0.1`；robot_type_service `1.0.0`；robot_state `1.0.0.2`，返回码均 0 | 服务 API 版本；不是电机固件，未核定服务所在物理主机 |
| RobotStateClient.ServiceList，1003 | 成功，28 条 name/status/protect 软件记录 | 不解释为物理板数量、健康判断或已装灵巧手 |
| RobotStateClient.GetPkgVersion，1006 | `3203`，官方定义为接口未实现 | package/module 版本保持 null |
| Go2 / B2 ConfigClient.Get(1002)、Meta(1004) | **未调用** | 公开源码要求具体 name，没有枚举方法；未找到 G1 复用说明、许可名称和标定 schema，不能仅凭版本相同套用 |
| G1 标定、软限位、力矩限制服务 | **未调用业务方法** | 版本号不提供读写语义；未找到匹配的 G1 公开只读业务协议 |

官方 G1 文档明确复用 B2 RobotStateClient，因此只调用其两个 getter。Python 客户端引用不存在的 `rpc.client_internal`；审计脚本仅适配这两个公开 getter 的固定封包，未修改厂商 SDK。这是审计脚本的兼容 workaround，应由厂商修复 SDK 导入；它不扩展业务接口权限。通用 API 1 的 SDK 服务端处理器直接返回版本，未进入标定或配置处理器。

**结果：8 次只读元数据 RPC，配置调用 / 标定调用 / 租约 / 动作写入均为 0，新增核定的电机配置值为 0。** DDS/ROS2 MotorState 不含电流上限、Kp/Kd、减速比、序号或零位标定。文档中保留的 q_raw 等字段不能反推零位。23 电机 × 11 个实际配置字段，共 **253 项仍为 null**。

在本轮公开资料范围内，尚不能核实一条适用于此 G1 固件、无需登录 PC1 的电机配置或标定读取路径；这不是证明接口不存在。继续补齐需要宇树提供适配本机固件的只读方法及 schema、获准读取的精确配置 name、逐电机序号映射、本机标定导出及其单位、坐标和完整性信息。不猜测私有 ID 或文件名试探。

## 2026-10-21 官方 GitHub issue / Discussions 核查

北京时间 **10:15:02（UTC+8）** 保存检索快照。宇树官方组织下 14 组公开 issue 查询包含 open / closed，取得 **198 个去重命中**，对命中标题及正文做相关性初筛；14 个相关 issue 的 **47 条评论完整读取**，并非人工审核全部命中及全部评论。`G1 config` 的 141 个命中已读取两页；首次 403 限流的查询已成功重试。8 个核心仓库的 `has_discussions` 均为 false，其中 SDK2 / ROS2 的 `has_issues` 也为 false；不能外推到组织所有仓库、私有工单或线下支持。

| 官方仓库中的帖子 | 经核实的内容 | 对本机的结论 |
|---|---|---|
| [RL Lab #42](https://github.com/unitreerobotics/unitree_rl_lab/issues/42#issuecomment-3415355301) | GitHub COLLABORATOR 指向 IsaacLab 动作缩放和 G1 29 DOF URDF joint limits | 模型 / 训练参数；没有本机限值或标定导出协议，不能替代 23 DOF 实参 |
| [Actuator #11](https://github.com/unitreerobotics/unitree_actuator_sdk/issues/11)、[#26](https://github.com/unitreerobotics/unitree_actuator_sdk/issues/26) | 独立电机编码器与置零问题；#11 有 COLLABORATOR 回复 | 未确认适用于 G1 机身，不采用其减速比、编码器类型或零位说法 |
| [Actuator #23](https://github.com/unitreerobotics/unitree_actuator_sdk/issues/23#issuecomment-2541414842) | GO-M8010-6 场景，COLLABORATOR 解释 sendRecv 是发控制后收状态 | 不是本机只读配置获取；未执行串口示例或标定模式 |
| [Dex1-1 #4](https://github.com/unitreerobotics/dex1_1_service/issues/4#issuecomment-5693917563) | COLLABORATOR 说明夹爪标定过程 | 是会改变标定的流程；本机无手，未执行，不能套用到机身 |
| [Python SDK #129](https://github.com/unitreerobotics/unitree_sdk2_python/issues/129) | ARM7 加灵巧手的过热提问，含提问者增益 | 其他配置的社区数值，没有只读配置导出方法，不入本机参数 |
| [RL Gym #95](https://github.com/unitreerobotics/unitree_rl_gym/issues/95) | 23 DOF 标题下的 22 维策略缩放 / PD 提问 | 训练配置，不是本机读取结果 |
| [Python SDK #28](https://github.com/unitreerobotics/unitree_sdk2_python/issues/28) | 反馈故障，社区建议切换模式和重启 | 不是标定读取；未运行切换或重启建议 |

官方组织的仓库不等于每条 issue 都由官方确认。`author_association=COLLABORATOR` 按 GitHub 返回值记录，不推定员工身份；协作者对模型限位、独立电机或夹爪的答复不扩大为本机参数授权。其余完整评论核查包含 SDK #72 / #162 / #170、XR #42、LeRobot #20 和 RL Gym #94，均未核实出本机配置或标定只读导出方法。

**本轮新增核定的逐电机实参为 0；253 项仍为 null。** 配置 / 标定读取支持保持未知，不写成“不支持”或“不存在”。GitHub 核查未增加参数探测 RPC；控制台重连后照常执行既有反馈及状态 getter。没有租约、配置读取试探、控制或标定操作。仍需厂商提供与本机固件及序号匹配的只读协议、配置名称与标定导出资料。

检索范围、分页、仓库功能开关、公开帖子及评论身份 / 更新时间 / 正文 SHA256 保存在 `github_audit_sources/review_20261021.json`；没有复制评论中的控制代码，哈希只标记抓取版本，不等同官方签名。归纳结论保存在 `g1_github_parameter_audit.json`，生成器校验来源 SHA256 和覆盖计数后收入基础模型 `github_review`。Mac 面板显示历史核查及原帖链接；观察提示明确社区材料不得补填实机参数。

## 保存与验证

- `g1_console/assets/g1_motor_knowledge.json`：基础档案 v5，包含对应版本、23 电机文档参数、控制板及非 SSH 接口历史核查和明确排除项；自动提供给每次 Mac 观察。
- `g1_console/assets/g1_github_parameter_audit.json`、`github_audit_sources/`：公开 issue 检索及评论核查证据，生成器校验 SHA256、分页及覆盖计数后收入 `github_review`，保持历史来源和实参分离。
- `doc/G1_GITHUB_PARAMETER_VERIFICATION.json`：本轮 72 项测试、前端构建、385 文件部署清单、实机只读回读与浏览器截图；Orin 已更新，观察 worker 在线且暂停。
- `g1_console/assets/g1_interface_parameter_audit.json`、`interface_audit_sources/`：此前官方资料、固定源码和带时间的 RPC 原始结果；生成器验证 SHA256 后收入 `interface_audit`，不转换为电机配置。
- `doc/G1_INTERFACE_PARAMETER_VERIFICATION.json`：此前 66 项测试、前端构建、383 文件部署清单、实机只读回读及浏览器验证；面板明确为历史核查。
- `scripts/audit_g1_interfaces.py`：固定服务版本查询及两个状态 getter，无任意 API ID / 配置 name 输入，无租约或控制入口。
- `g1_console/assets/g1_board_parameter_audit.json`、`board_audit_sources/`：原始被动读取、安装 XML 及 SHA256。生成器验证哈希后提取摘要；历史数据不冒充实时状态。
- `scripts/audit_g1_boards.py`：有限窗口核查，只建 DataReader，不建应用 DataWriter、不发 RPC。
- `g1_console/assets/g1_device_parameter_audit.json`：带时间的只读检查、配置文件 SHA256、非敏感软件模块版本与机型读数。
- `g1_console/assets/official_motor_sources/sources.json`：固定来源 commit、路径、SHA256；生成器校验原始文件后解析表格、URDF 和 SDK 函数。
- `doc/G1_BOARD_PARAMETER_VERIFICATION.json`：本轮 61 项测试、前端构建、365 文件部署清单和实机只读回读记录；相机当前返回 SDK_3102，如实记录未恢复。此前验证保留在 `doc/G1_MOTOR_PARAMETER_VERIFICATION.json`。
- 文档适用性要求新鲜反馈、mode 4 和完整 23 关节编号；机型改变、缺关节或断流时不显示符合。热阈值未知时，健康结论继续为待核实。所有条件比较不授予动作权限。

## 固定的官方来源

- [机型表与髋关节减速比](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description/README.md)
- [mode 4 对应 URDF](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description/g1_23dof_rev_1_0.urdf)
- [温度通道和可覆盖的辅助阈值](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/g1/common/terminations.hpp)
- [通用电机表：未套用到本机](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description/motor_info.md)
- [官方 PC1 / PC2 地址与 DDS 发现说明](https://github.com/unitreerobotics/xr_teleoperate/wiki/CycloneDDS-and-UnitreeSDK-%28en%29)
- [主板原始字段](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/idl/unitree_hg/msg/dds_/_MainBoardState_.py)
- [BMS 原始字段](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/idl/unitree_hg/msg/dds_/_BmsState_.py)
- [G1 基础服务文档](https://support.unitree.com/home/en/G1_developer/basic_services_interface)
- [G1 明确复用的 RobotStateClient](https://support.unitree.com/home/en/G1_developer/robot_state_client_interface)
- [RPC 服务端：API 版本处理](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/rpc/server.py)
- [Go2 ConfigClient：未确认适用于 G1](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/go2/config/config_client.hpp)
