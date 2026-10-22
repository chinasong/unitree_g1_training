# G1 Orin 只读控制台部署记录

身体与能力基线实测：**2026-10-20 23:17:35，北京时间 UTC+8**；相机部署实测：**2026-10-20 23:51:37，北京时间 UTC+8**。2026-10-21 增加 Mac Codex 自主观察。已部署到真实 G1 EDU 23 DOF / Orin，并在 Mac 打开图形界面。未执行动作、声光或控制模式写入，未操作 Quantumult X。

## 当前入口与设备

| 项目 | 核验结果 |
| --- | --- |
| Mac 图形界面 | <http://127.0.0.1:18765/>，真实 Orin 服务的 SSH 转发 |
| Orin SSH | `unitree@<ORIN_IP>`，`wlan0` |
| Orin 身份 | NVIDIA Orin NX Developer Kit，aarch64，Ubuntu 20.04.6，Tegra R35.3.1 |
| 内部 DDS | `eth0`，`192.168.123.164/24` |
| Mac 网络 | `en0`，`<MAC_IP>/24`；连接进程绑定接口，无系统路由改动 |
| SSH ED25519 指纹 | `<ORIN_HOST_FINGERPRINT>` |
| Orin 服务监听 | `127.0.0.1:8765`，强制 `--read-only` |
| 当前发布标识 | `a346eb8080ac`，385 个清单文件逐个 SHA256 验证；此前发布保留 |
| 部署目录 | `/home/unitree/workspace/g1-body-console` |

无线地址由路由器分配，变化后需重新查找并核验主机指纹；不要因为 IP 相同就接受其他设备。Mac 本地演示的 `8765` 与真实入口 `18765` 分开。

## 2026-10-21 官方 GitHub 参数核查更新

北京时间 **10:20:53（UTC+8）** 回读验证：基础档案 v5 指纹 `94e5781dcdcccb86669174a60e5e57c512f31d272fe68b3a1cba380253e0f839` 与 Mac 源码完全一致，23 个关节反馈年龄 39 ms，只读开启、无控制 owner、导航 idle；本次运行事件日志中的 `command_accepted=0`。项目服务 PID 8470；官方视频进程 PID 2557（`videohub_pc4`）和机器人 boot ID 保持不变。

10:15:02 保存的 GitHub 核查包含 14 组公开查询、198 个去重命中，完整读取 14 个相关 issue 的 47 条评论；没有把命中数当作完整审核数。已查的 8 个核心仓库均未开启 Discussions，不代表整个组织。尚未核实适用于本机的电机配置或标定只读导出协议；新增实参 **0**，253 项继续为 null。模型限位、独立电机与夹爪标定、其他配置的社区增益均未填入本机参数。

**72 项定向测试、前端构建、385 文件远端 SHA256 清单和 pip 依赖检查通过**；浏览器已验证只读、23 关节和 v5 核查面板。仅重启本项目控制台及 Mac 观察 worker（PID 33803），以加载资料可信度规则；观察仍为暂停、worker 在线，没有生成新环境报告。相机仍返回 `SDK_3102`。GitHub 核查没有增加参数探测 RPC，控制台保持原有反馈及状态 getter；未执行动作、标定、置零或控制模式写入，未修改 Quantumult X、网络、自启动或官方服务。

证据：[GitHub 核查与部署验证](G1_GITHUB_PARAMETER_VERIFICATION.json)、[参数核查及原帖来源](G1_MOTOR_PARAMETER_AUDIT.md)。

官方 GitHub 参数核查界面（截图 / 照片未公开，仅本地存档）

## 2026-10-21 非 SSH 参数接口档案更新

北京时间 **10:04:17（UTC+8）** 回读验证：基础档案 v4 指纹 `7aa83e6dc70cfb7a0aa7bfd60deb57e49f8cc86d9e64c84ad3a2ff501fc23efb` 与 Mac 源码一致，23 个关节反馈年龄 14 ms，只读开启、无控制 owner、导航 idle、`command_accepted=0`。当前项目服务 PID 6140；官方视频进程 PID 2557 和机器人 boot ID 保持不变。

09:58:03 完成的历史元数据查询现已进入基础模型及 Mac 面板：6 个服务 API 版本、28 条软件服务记录，包版本接口返回 3203（未实现）。没有登录运动主机 PC1，没有发配置或标定业务请求；Orin SSH 只作 DDS 查询桥接。服务版本不能解释为电机固件，Go2/B2 配置接口尚未核实适用于 G1。新增核定实机电机配置为 **0**，253 项继续为 null。

**66 项定向测试、前端构建、383 文件远端 SHA256 清单和 pip 依赖检查通过**；浏览器已验证只读、23 关节和新面板。Mac 观察 worker 已恢复在线，观察保持暂停，未生成新的环境报告；相机仍返回 `SDK_3102`。未修改 Quantumult X、系统网络、自启动或官方服务。

证据：[接口核查与部署验证](G1_INTERFACE_PARAMETER_VERIFICATION.json)、[参数核查及官方来源](G1_MOTOR_PARAMETER_AUDIT.md)。

非 SSH 接口核查界面（截图 / 照片未公开，仅本地存档）

## 2026-10-21 控制板档案更新

北京时间 **09:50:33（UTC+8）** 回读验证：基础档案 v3 指纹 `5627182be7253635c638ab701c791077b5593a7f818fcb52028bf08a6b7b9004` 与 Mac 源码一致，23 个关节反馈年龄 4 ms，只读开启、无控制 owner、`command_accepted=0`。新发布进程 PID 为 4593；本次检查中宇树视频进程 PID 2557 保持运行，没有停止或重启它。

本轮主板 / BMS 的被动回读、运动主机 SSH 拒绝连接、安装模型差异均保存到档案和“其他控制板核查”面板；这些是带日期的历史证据。新增核定的逐电机配置为 **0**，23 电机的 253 个实机配置字段仍为 null。没有用命令帧零增益、安装 XML 或 SDK 示例替代。

机器人本次启动后，相机读取返回 `SDK_3102`，Orin 未枚举出 `/dev/video*`，因此本轮没有验证新的相机画面。Mac 观察 worker 保持等待输入，没有以旧画面生成新报告。下文相机与观察成功记录均为此前验证，不能代表此次启动的当前状态。

**61 项定向测试、前端构建、远端清单 SHA256 和 pip 依赖检查通过**。未修改 Quantumult X、系统网络、自启动或官方服务。证据：[本轮验证](G1_BOARD_PARAMETER_VERIFICATION.json)、[参数核查](G1_MOTOR_PARAMETER_AUDIT.md)。

控制板核查界面（截图 / 照片未公开，仅本地存档）

## 读取到的身体和能力

| 验证项 | 实测证据 |
| --- | --- |
| 23 个关节 | 30 帧均有角度、角速度、估算力矩、温度、电压、原始状态码 |
| 稀疏编号 | `0..12`、`15..19`、`22..26`，共 23 个 |
| 连续状态 | 约 5.934 秒，tick `2891887 → 2897832`，CRC 异常 0 |
| 数据年龄 | 15–22 ms，指服务器收到身体反馈后的年龄，**不是 Wi-Fi RTT** |
| 温度 / 电压 | 采样时 32–46°C、49.5–50.0 V；随运行变化 |
| IMU | 姿态、角速度、加速度、四元数及原始温度均有反馈 |
| 状态机 | `GetFsmId → 0`，LowState `mode_machine → 4`；仅回读，未切换 |
| SDK 能力 | 47 个操作入口，来自锁定的官方 SDK |
| 实机固件 | `GetActionList` 声明 20 个动作 ID、4 个无 ID 命名条目 |
| 只读查询 | `GetFsmId`、`GetActionList`、`GetVolume` 均返回成功 |
| 网页模型 | 29 个 STL 文件均能加载，姿态来自实时关节反馈 |

固件动作 ID：`1, 11, 12, 13, 15, 17, 18, 19, 22, 23, 24, 25, 26, 27, 31, 32, 33, 34, 35, 99`。另外四个名称为 `Waist_Drum_Dance`、`Scratch_head`、`Spin_discs`、`Throw_money`；没有返回 ID，仅展示。界面保留原始 FSM、mode_machine、时长条件，不推断未声明的支持范围。

固件声明不等于实际动作已验证；温度、状态码和模型姿态也不是平衡或机械安全结论。完整电池电量、地图、定位和扫描尚未接入，没有验证动作或导航。相机已在后续发布接通，见下节。

完整采样与原始查询结果：[G1_ORIN_READONLY_VERIFICATION.json](G1_ORIN_READONLY_VERIFICATION.json)。

真实身体反馈（截图 / 照片未公开，仅本地存档）

实机固件能力（截图 / 照片未公开，仅本地存档）

## 相机接入更新

当前发布通过官方 `VideoClient.GetImageSample`（API 1001）读取既有视频服务，不直接打开摄像头、不重启官方服务。实机 D435i 返回 1920×1080 JPEG，本项目缩放为 960×540，配置采样上限 5 Hz。“环境观察”与“自主任务”页面均可预览。

Mac 连续 24 次取图得到 24 个不同序号、24 个不同 JPEG，服务端请求帧龄 46–235 ms；同时 23 关节持续反馈、CRC 异常 0。无控制 owner，导航 idle，`command_accepted` 为 0。Codex 快照脚本已实际保存并查看一帧。官方接口没有曝光时间戳，帧龄按取图请求计算，不能解释为传感器曝光延迟。

使用与断流处理见 [相机只读接入说明](G1_CAMERA_READONLY.md)，连续采样证据见 [G1_CAMERA_VERIFICATION.json](G1_CAMERA_VERIFICATION.json)。

Mac 环境观察实时相机（截图 / 照片未公开，仅本地存档）

## 自主观察更新

2026-10-21 增加 `g1_console/insight.py` 和新的网页面板。Orin 只管理输入批次、校验报告及展示；Mac 的 `scripts/observe_g1.py` 负责调用已登录 Codex 服务分析图像与身体状态。开始 / 暂停只改变观察模块状态，没有动作执行通路。当时项目只读服务 PID 为 `15430`，官方视频服务 PID `2295` 保持不变；当前状态见本文顶部日期更晚的验证。详细恢复、模型选择、数据流、过期处理见 [自主观察说明](G1_AUTONOMOUS_OBSERVATION.md)，实机证据见 [观察验证记录](G1_OBSERVATION_VERIFICATION.json)。

## 重连与机器人重启后的恢复

2026-10-22 已改为持续恢复连接器。双击项目根目录 **`连接G1只读控制台.command`**，或执行：

```bash
.venv/bin/python scripts/connect_orin_console.py --open
```

保持终端窗口运行。连接器扫描 Mac 当前 Wi-Fi 的私有 /24–/30 网段，核验原有 Orin 主机指纹后才请求密码；密码仅保留在当前进程内存。机器人重启或 DHCP 地址变化后自动重试并重建隧道，不需要再次手工 SSH 启动项目。`Ctrl+C` 结束本机恢复进程。

已核验的 Orin 上，项目服务缺失时核对 `active-release` 和 SHA256 清单，恢复本项目已有发布；默认只读。只有本次新启动且无控制者、未授权运动、无导航任务的服务才初始化 `eth0` 身体读取。已有服务保持当前模式及显式断开状态，不抢占端口、不杀进程、不恢复控制权或动作。入口模式不匹配时明确报告。

**这不能替代 Orin 自身联网与 SSH 开机可用性。** 当前 `<APP_REPORTED_IP>:22` 拒绝连接，全网段未发现已核验的 Orin SSH；Wi-Fi 自动连接、SSH 开机状态与实机重启验收仍待取得 Orin 入口后核查。尚未修改系统网络、自启动、SSH 配置或官方服务。Mac 本地改造与历史 Orin 发布分开，尚未向失联 Orin 部署新前端。详见 [重启恢复方案与证据](G1_REBOOT_RECOVERY.md)。

已有 `18765` 恢复进程时直接使用现有网页。更换 DDS 网卡需核查现场配置后处理；只关闭浏览器时可重新打开页面。Quantumult X 保持原样。

## 部署内容与隔离

```text
/home/unitree/workspace/g1-body-console/
  .venv/                       独立 Python 环境
  runtime/python/              CPython 3.11.15 aarch64
  runtime/cyclonedds/           独立 CycloneDDS 0.10.2
  releases/a346eb8080ac/        当前代码、前端 dist、SDK 源码与官方模型
  releases/62e85764bb01/        保留的非 SSH 接口核查发布
  releases/7b1e6ba0dcd8/        保留的控制板核查发布
  releases/093bd2650496/        保留的自主观察发布
  releases/4acbab31645a/        保留的相机基线发布
  releases/4233d34f6b5a/        保留的身体与能力基线发布
  state/                       项目自己的事件记录
  active-release               当前发布标识
  start-readonly.sh            固定只读、loopback 监听的启动入口
  console.log / console.pid    本次后台运行记录
```

原有 `/home/unitree/workspace/unitree_g1_training`、系统 Python、已有 DDS 0.9.1 和官方运动服务均保留。官方 Python SDK 以当前发布目录源码的 editable 安装保留 `crc_aarch64.so`；依赖检查 `pip check` 通过。没有把 Mac 虚拟环境或 Node 产物依赖目录复制到 Orin。Python CPU 路径已实测，GPU / ROS 推理兼容不在此次验证内。

官方依赖固定为 `unitree_sdk2_python@814556d15970dd2ecf1c9984e845ca02ab07e206`；身体网格来自 `unitree_rl_gym@276801e46c5d433564f24658bac64f254b7d2d4b`。Mac 构建后使用 `scripts/package_orin_console.py` 生成带 SHA256 文件清单的发布包，产物位于忽略的 `.g1-deploy/`。

## 初始发布验证范围（当前发布见顶部更新）

- **45 项定向测试通过**：包含所有 44 个写操作在服务、RPC、SDK 适配器处被拒绝，三种身体/能力查询确实到达 SDK，关闭只读适配器不发送停止或其他动作；另覆盖官方相机时效 / 来源，以及观察身份与意图头、批次与 worker 租约、暂停失效、断流 / 来源变化 / 超时拒收、历史标记、未知能力拒绝、零动作、CLI 启动警告与工具调用区分。
- TypeScript 检查与 Vite 构建通过；Three.js 分块有现存体积提醒。
- 实机 HTTP、WebSocket、DDS、CRC、固件查询和浏览器展示均通过；验证时无控制 owner、导航 idle、`command_accepted` 事件为 0。
- Mac 重连脚本已通过临时 `18766` 隧道核验远端只读状态，随后关闭测试隧道；正式 `18765` 保持运行。
- 只读模式同时在界面、服务和适配器拦截写操作，包括 StopMove、模式切换、声光；没有启动训练或运动例程。
