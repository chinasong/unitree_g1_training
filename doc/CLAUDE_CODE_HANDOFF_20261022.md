# G1 项目与机器人工作台交接给 Claude Code

整理日期：**2026-10-22，Asia/Shanghai（UTC+8）**。Orin 进程快照 **16:31:28**；Mac 两个工作台入口的 GET 复核 **16:35:31**；Orin 当前发布全部文件哈希复核 **16:43:52**。本文是交接时点的状态，不代表接手时机器人仍在线。

## 1. 先知道这些

**工作台已部署、身体和相机已接通；软件遥控器还没有完整替代实体遥控器。** 当前主要阻塞为官方服务的 **7301 / LocoState 不可用**、运动板真实固件及模式进入条件未知。起身、姿态保持、独立平衡和行走未实机验收。

用户本次要求整理项目、G1 连接和工作台内容，自行转交 Claude Code。本次只读核验并写交接文件，没有发送新的实机动作、切模式、升级、改网络或联系其他会话。

| 项目 | 交接状态 |
| --- | --- |
| 工作区 | `.` |
| Git 来源 / 分支 | `https://github.com/chinasong/unitree_g1_training.git` / `master` |
| 基础 HEAD | `e2f688b1bc2cb8327e4b89f2c3a36602a7bdcaed`，2025-12-31 的原项目提交 |
| 未提交工作 | `.gitignore`、`README.md`、`g1_douji_voice_chat.py` 已修改；`g1_console/`、`console/`、`scripts/`、`tests/` 和大量审计文档为未跟踪文件。**HEAD 不代表当前实现** |
| 实机配置 | 用户确认 G1 EDU、23 DOF、无手、Jetson Orin；App 显示 G1 EDU+、软件 v1.5.6、硬件 v1.0，这些不等于运动固件版本 |
| Mac | Apple Silicon / macOS 26.x；Quantumult X TUN 全代理保持原状 |
| 实机入口 | `unitree@<ORIN_IP>`，SSH 固定主机指纹见下节 |
| Orin 发布 | `da5d189707fa`，包内 **495** 个清单文件；最近部署已逐项校验 SHA256 |
| Mac 本地包 | `03034ef80b29`，495 个清单文件与本地源码匹配；**尚未部署**，与 Orin 有 5 个 Python 文件差异 |
| 运行身份 | boot `fc0c327a-b381-400a-b20d-6307f4f43e2e`；PID `6251`、UID `1000`、start_ticks `249464`；run_id `a55319b05192`。这些均为快照，不能盲用 PID |
| 运行范围 | `--manual-remote --damping-test`；owner=null、armed=false；唯一可写种类为 damp，但单次凭据已经使用，不可再次发送 |
| 身体 / 画面 | 23 关节反馈已接入；FSM=0；相机 live，最近 60 秒 **300/300** 成功、0 超时；双膝显著弯曲，完整支撑姿态未知 |
| 版本 / 错误 | 服务广告 `ai_sport=8.7.5.212`；真实 humanoid 运行固件未知；7003–7006 仍为 7301；版本查询 1006 曾明确返回 3203 |

机器可读交接：[当前核验与关键证据](evidence/claude_handoff_20261022.json)。文件盘点：[源码、实验与文档清单及 SHA256](evidence/claude_handoff_inventory_20261022.json)。清单包括未跟踪文件，排除依赖、运行缓存和构建目录；它不等同于 Git 提交或发布包。

**同机接手直接使用现有工作区。换机器接手必须携带未提交源码和文档，不能只 git clone。** `.g1-deploy/` 包含未入 Git 的本机部署材料；是否需要复制按用途选择，避免把含现场画面的原始目录整包传到无关服务。本文不包含密码、cookie、SSH 私钥或会话 token。

## 2. 用户目标和已确定的边界

目标是 HomeBody 式“观察 → 理解身体与环境 → 调用现有技能 → 看反馈 → 修正任务策略”。Mac 上的编码 Agent 开发和调试策略，Orin 运行身体接口，官方运动控制器负责已有运动能力；**不训练新的运动模型、不用 LLM 直接闭环驱动每个电机**。

用户希望桌面图形软件展示 G1 身体细节、相机、能力和任务建议，最终替代遥控器。当前为本地网页工作台和 `.command` 启动器，没有打包成独立 `.app`。能力目录、参数表或按钮存在不等于这台机器人已支持并实测通过。

- Quantumult X 不关闭、不重启、不改配置；若以后确需重启由用户手动操作。
- 用户明确拒绝当前实施 Wi-Fi 省电修改，保持只读诊断。不得借“修复连接”执行该候选试验。
- 按用户全局指南：系统网络、sudo、系统/官方服务、登录项、权限及认证材料修改先确认。不要写 Wi-Fi 密码、改 SSH key 或动其他项目。
- 可以修改本项目代码、文档、测试，启停本项目自己拉起的服务；保留未提交工作。避免同一工作区有两个编码会话同时改同一文件。
- 历史上用户允许根据画面和身体反馈选择简单动作验证，但当前脚底和背部支撑不可见，承重不明。交接先只读核验，不扩大动作范围、不删除保护凭据、不用试发模式来猜服务就绪条件。
- 不用示例 Kp/Kd、模型 effort、软件输入限值、SDK 默认值替代本机参数；未知字段保持 null，历史状态不提升为当前状态。

## 3. 连接拓扑和怎样恢复工作台

```text
Mac 浏览器 http://127.0.0.1:18766/   （另起的 Vite preview）
       └── /api 和 WebSocket → http://127.0.0.1:18765/
                                   （项目 Paramiko SSH 桥，也可直接浏览）
              └── en0 上直连、先验主机指纹 → unitree@<ORIN_IP>:22
                    └── Orin 127.0.0.1:8765  FastAPI + 静态 UI
                          └── Orin eth0 / DDS domain 0 / 192.168.123.0/24
                                ├── Orin 192.168.123.164
                                └── humanoid 应答线索 192.168.123.161
```

Orin 主机指纹：`<ORIN_HOST_FINGERPRINT>`。由 `scripts/connect_orin_console.py` 在任何密码发送前核验，不匹配则跳过。密码交互输入、只保留进程内存，不写交接文档。App 显示的 `wlan0:<APP_REPORTED_IP>` **没有被证明是这台 Orin 的 SSH 地址**；历史上它能供 App 使用而 Orin SSH 不可达，两者不能混为一谈。

**先检查现有 18765/18766，能用就复用，勿重复启动。** 当前连接器和 Vite preview 已由前一轮启动；终端 session 编号只是工具运行上下文，不是跨编码工具的管理接口。

连接器的原命令（仅在没有已有连接器时运行）：

```bash
cd .
.venv/bin/python scripts/connect_orin_console.py \
  --host <ORIN_IP> --user unitree --interface en0 \
  --port 18765 --remote-port 8765 --manual-remote --damping-test --open
```

这与双击 `连接G1软件遥控器.command` 的范围相同，**不代表开放全部写操作，也不发送阻尼**。连接程序检查现有服务的范围；只在本项目服务缺失且发布校验通过时恢复它，不停止官方服务，不取得控制权、不自动解锁、不恢复断线前动作。

连接器每次检查 en0 所在的有限私网网段，结合缓存/优先地址发现 SSH，固定指纹确认身份；心跳 5 秒、失败后默认 10 秒再查。通过 macOS `IP_BOUND_IF` 绑定 en0，不需要改 Quantumult X。`.g1-console/connection.json` 保存地址和诊断元数据，不保存密码。

`连接G1只读控制台.command` 要求 `read_only=true`。**它不会把当前人工遥控服务偷偷切成只读；范围不匹配会拒绝转发并保留服务。** 要变更项目服务运行范围需明确处理自己的进程，再重新验证；不要加宽旗标以绕过范围检查。

18766 当前 preview 配置位于 `.g1-deploy/mac-readonly-preview.config.ts`。文件名是历史命名，**代理本身不保证只读，最终由 Orin 服务端门控**。若 preview 确实缺失而 18765 正常，可在 `console/web` 下运行：

```bash
npm exec vite -- preview --config ../../.g1-deploy/mac-readonly-preview.config.ts
```

该配置代理 `/api`（含 WS）到 18765，并保持所需 Origin；不是 DDS 隧道。若文件缺失，按现有配置重建项目内文件，不修改系统代理。Mac 本地 `8765` 常用于离线/演示，不要与 Orin 的同端口混淆。

只读 HTTP 冒烟示例，取得会话 cookie 后只 GET；输出只选必要字段：

```bash
.venv/bin/python - <<'PY'
import http.cookiejar, json, urllib.request
base = 'http://127.0.0.1:18765'
op = urllib.request.build_opener(urllib.request.ProxyHandler({}),
    urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
op.open(base + '/api/session', timeout=5).read()
s = json.load(op.open(base + '/api/state', timeout=5))
print({k:s.get(k) for k in ('run_id','mode','connected')})
print('owner=', s['control']['owner'], 'armed=', s['manual_remote']['armed'])
print('fsm=', s['telemetry'].get('fsm'), 'camera=', s.get('camera',{}).get('status'))
PY
```

相机保存单帧：`.venv/bin/python scripts/read_g1_camera.py --url http://127.0.0.1:18765 --manual-remote --output /tmp/g1-current.jpg`。这里的 `--manual-remote` 只明确允许从当前人工服务读图，脚本仍只 GET。不要把单帧/相机在线当作脚底承重证据。

## 4. Orin 部署、目录和版本校验

```text
/home/unitree/workspace/g1-body-console/
  .venv/                    # Orin 专用 Python 运行环境，不复制 Mac venv
  runtime/python/           # 已备好的 aarch64 CPython 3.11.15
  runtime/cyclonedds/        # CycloneDDS 0.10.2
  releases/<12位发布号>/     # Python、web/dist、锁定 SDK、模型 mesh、release-manifest.json
  active-release            # 当前选择：da5d189707fa
  console.pid, console.log  # 只属本项目；PID 文件不是进程身份的充分证据
  recovery.lock
  state/                    # 持久化日志与测试凭据，发布切换不清空
    damping-test.json        # 已使用，严禁重置/删除
```

进程 cwd 为 `.../releases/da5d189707fa`。典型环境包含 `CYCLONEDDS_HOME=.../runtime/cyclonedds`、`LD_LIBRARY_PATH=.../runtime/cyclonedds/lib`、`G1_CAMERA_BACKEND=unitree`、`G1_DATA_DIR=.../state`、`G1_HOST=127.0.0.1`、`G1_PORT=8765`；准确运行旗标仍要读 `/proc/<pid>/cmdline` 和服务状态，不能从脚本名推断。

`scripts/orin_console_recovery.py` 是兼容 Orin 系统 Python 3.8 的项目恢复包装器；校验发布号与 manifest、逐文件 SHA 和路径边界，使用现有 venv 启动。若端口已监听或已有自己的冷启动进程则不重复启动。没有注册全局 systemd/launchd 自启；**自动恢复依赖 Mac 连接器仍在运行且 Orin 的 Wi-Fi/SSH 恢复可达**，不是离线驱动故障的根治方案。

本地构建打包：

```bash
cd console/web
npm run check
npm run build
cd ../..
.venv/bin/python scripts/package_orin_console.py
```

产物：`.g1-deploy/console-release.tar.gz`、`release-manifest.json`（相对路径→SHA256）、`release-id`。发布号为 manifest 规范 JSON 的 SHA256 前 12 位。**打包不等于上传、激活或重启服务**；当前 495 文件是已有包的证据，不是未来每包的固定数量。

**交接核验发现的本地/已部署差异：**本地 manifest 与压缩包为 `03034ef80b29`，本地 495 项文件全部匹配；Orin active-release、进程 cwd 和远端 manifest 均为 `da5d189707fa`，远端 495 项文件也全部匹配。因此不是哈希损坏，不能将两份产物混称同一个发布。

| 相对路径 | 本地尚未部署的改动 |
| --- | --- |
| `g1_console/capabilities.py` | 增加统一 `require_write_scope()`，缺少范围旗标时拒绝隐式放行 |
| `g1_console/service.py` | service 分发门控调用同一范围规则 |
| `g1_console/adapters.py` | SDK 最终边界调用相同规则，保留单次阻尼凭据限制 |
| `g1_console/contracts.py` | 速度契约改用集中定义的 `MAX_VX/MAX_VY/MAX_OMEGA`，数值仍 0.3/0.2/0.4 |
| `g1_console/config.py` | 移除重复速度配置字段 |

这些改动在本次交接开始前已存在；本次未改动它们、未部署本地包。对比原文保存在 `.g1-deploy/research/claude-handoff-local-unreleased-20261022.diff`，两侧 SHA 和本次 139 项定向测试结果见机器可读交接。后继不能把尚未部署的门控重构当作 Orin 当前执行路径。

以往部署使用过一次性 `/tmp/g1_deploy_v36_remote.py`，没有成熟的一键全量部署 CLI。后继若做可重复部署，应在项目内实现并测试：固定指纹 SSH → 新目录 SFTP → 核验清单 → 核对 boot/PID/cwd/start_ticks、owner 空/未解锁/导航 idle → 仅停止匹配的自己的服务 → 激活与恢复相同范围 → 检查官方进程未被改动、持久凭据保留、UI/相机/回读正常。不要复制盲目 kill 命令。

**现有 editable SDK 的 import 路径仍指向旧发布 `953b51275852`**；上一轮已对实际加载的 LocoClient/ClientBase/ClientStub 三文件与锁定源码逐项 SHA 校验一致。它不是混版的充分证据；也不能随意删除旧 releases 或重绑依赖，先查实际 import 与哈希。

最近凭据 SHA256：`bc14c8abee96581486e38cd1c4c16a52b5a464aa39de58afad62c0eae38bbd60`。这是交接快照用于比对，不替代重读真实文件。

## 5. 项目内容地图

| 目录 / 文件 | 内容和责任 |
| --- | --- |
| `g1_console/app.py`、`__main__.py`、`config.py`、`contracts.py` | FastAPI、启动模式、Cookie/Origin/意图校验、数据契约与时效/输入限制 |
| `g1_console/service.py`、`adapters.py`、`capabilities.py`、`remote_preflight.py` | 状态整合、项目控制权租约、最终执行门控、官方 SDK、能力目录和不下发的请求核对 |
| `body.py`、`assessment.py`、`state_learning.py`、`standing.py` | 身体基础档案、状态解释、短时观测、官方姿态路线与支撑证据；不从被动样本推导安全范围 |
| `mode_readback.py`、`hg_sport_state.py`、`safety_feedback.py`、`service_versions.py` | 独立模式读取进程、HG 状态、辅助 IMU/电池等反馈、显式服务名称/版本绑定 |
| `camera.py`、`camera_rpc.py`、`camera_process.py` | 官方 Video API 1001 的隔离读图、JPEG、时效和超时统计；不是直抢 USB 相机 |
| `insight.py`、`scripts/observe_g1.py` | 有输入批次 ticket/来源/时效的只读视觉观察与结构化建议；模型推理依赖 Mac 已登录 CLI 服务 |
| `navigation.py`、`ros_bridge.py`、`simulation.py` | 几何地图/A*、定位扫描桥接、有限纠错、合成 demo；真实地图/定位/障碍输入未完整接通验收 |
| `g1_console/assets/` | 身体结构、基础知识、官方源文件摘录、版本和安全审计证据。`g1_motor_knowledge.json` schema 27、`g1_motion_control_review.json` schema 15 |
| `console/web/src/` | React/TypeScript UI，`App`、`BodyView`、`CameraView`、`RemoteView`、`RemoteOperations`、`OperationDialog`、`LocoReadback`、`ControllerReadiness`、姿态/手册/知识/观察/地图等面板 |
| `scripts/connect_orin_console.py`、`orin_console_recovery.py` | Mac 发现、固定身份 SSH 转发、项目恢复与开机只读诊断 |
| `scripts/package_orin_console.py`、`start_g1_console.sh` | 打包与本地开发/合成演示启动；不把 Mac 环境复制到 Orin |
| `scripts/build_g1_body.py`、`build_g1_motor_knowledge.py`、`build_g1_capability_knowledge.py` | 从锁定模型/官方源码/带 SHA 证据构建知识；更改输入须同步构建及验证 |
| `scripts/audit_g1_*.py` | 电机接口/服务/板卡/版本/日志/状态与实际 SDK 只读审计；每个脚本先读守卫和运行环境，不整目录盲跑 |
| `scripts/analyze_g1_*.py`、`g1_*_evidence.py` | 非明文配置、FMX、库文件、包缓存的离线分析及证据整理 |
| `scripts/sync_unitree_deps.py`、`check_unitree_compat.py`、`unitree-deps.lock.json` | 官方依赖锁定、恢复与兼容检查；`externals/` 被忽略 |
| `tests/` | 30 个 Python 测试文件，门控、时效、只读审计、证据来源、连接恢复等；没有证明所有物理动作 |
| `doc/` | 按主题/时间的调查、机器可读证据、截图、官方手册；旧文档中的“当前”只能按其日期解读 |
| `.g1-deploy/` | 被忽略的本机 wheels/包/manifest/preview 配置/原始研究输出和现场图像；不是已提交源码 |
| `.g1-console/`、`.g1-observer/` | 被忽略的连接/事件/观察进程本地运行状态；不要当成可分发账号材料 |
| `.venv/`、`console/web/node_modules/`、`console/web/dist/` | 可重建本机依赖/构建；依赖按锁文件恢复，dist 是运行所需产物 |
| `armdev/` | 原项目手臂数据、CSV/MuJoCo/LowCmd 回放，含可真实下发的早期实验，勿拿来直接验证整机 |
| `recorddata/` | ROMP 动捕转 G1 CSV、平滑、可视化 |
| `cameradev/` | 相机、人脸/目标识别与对话，含 `yolo11n.pt`；不属于现有官方运动闭环 |
| `videodev/`、`legdev/`、`g1dev/`、`trainging/` | 原项目视频转 CSV、腿部、设备数据、右臂录制实验；`trainging` 是现有目录拼写 |
| `g1_douji_voice_chat.py`、`douji_voice_chat_ubuntu.py`、`requirements.txt` | 原项目“豆几”语音对话和旧依赖，独立于新控制台；前者有未提交改动，保留 |
| 4 个中文 `.command` | 本地控制台、连接只读、连接受限遥控、启动自主观察；实际旗标见第 3、8 节 |

主要 HTTP：`GET /api/session`、`/api/state`、`/api/body`、`/api/camera.jpg`、`/api/camera/status`，`WS /api/live`。状态流约 5 Hz。`/health` 或 `/ready` 成功只证明其定义的服务条件，不等于可运动。

控制类 POST 包括 `/api/connect`、`/api/control/{acquire,heartbeat,release}`、`/api/control/manual/{arm,disarm}`、`/api/control/preflight`、`/api/command`。只读官方 getter 也通过能力命令入口分发，不要把所有 POST 当作实机动作或反过来把所有查询都当成无副作用。修改执行逻辑要追到 `service.py`、能力范围与 adapter 的最终调用。

默认身体时效 0.6 秒、感知 0.7 秒、窗口租约 2 秒、速度有效期 0.25 秒；项目速度上限 0.3 / 0.2 m/s、0.4 rad/s。这些是**项目软件配置**，不是已验证的本机物理安全限值。

## 6. 身体、参数和安全证据已经学到了什么

- 23 DOF 的真实消息采用稀疏索引：双腿 `0..11`，腰 `12`，左臂 `15..19`，右臂 `22..26`。不能压成 0..22，也不能套用 29 DOF。
- 主反馈为 HG LowState，校验 CRC、tick、时效、坐标模式；q/dq/tau_est/温度/原始位字段按已确认 SDK 语义展示。`tau_est` 是估算力矩；原始状态位为 0 不证明所有健康；温度辅助函数默认阈值不等于实机热保护。
- 官方模型结构、轴、限位、质量和 mesh 用于身体可视化/参考。基础档案已记录参数来源；**253 个实机配置字段仍为空**，包括实际增益、标定、限流、热规则等未取得内容。不要将离线模型总质量当实机称重。
- BMS、双路 IMU、HG sport、Go 格式 odom 原始通道已做只读接入研究；部分单位、字段有效性及固件语义未核验。尤其 Go 消息的四个 `foot_force=0` 不能当作 G1 左右脚测力结果。
- 本轮身体形态为双膝约 160.7° / 158.5°、IMU 主体小角度倾斜；只可描述关节形态。3D 模型与用户曾确认一致，不代表已得到世界高度、坐地/站立分类或支撑多边形。
- 历史人工报告为“坐地、双脚贴地、背靠柜体”，曾更早报告人托住；重启后这些不能自动当作实时事实。相机没有脚底和背部完整视角，双脚与柜体承重、外部扶持、独立站稳均未知。
- “不摔倒”安全边界尚未建立。短时不动或没有摔倒、几次角度极值不能成为新安全包络；观察记录明确 `certified_safe=false`，不产生动作许可。

只读参数研究已检查官方文档/GitHub、Orin 板卡与启动日志、非明文配置、库文件和 OTA/安装缓存。MegaFlow `config.bin` 已离线解析为组件清单，未取得电机标定；`master_service.json` 的 FMX 内部数据未解码，匹配 Mixer/Gen 实现未找到。`libcryptopp.a`、`librealsense2.so` 已检查，不是“尚未扫描”；RealSense 相机标定不等于关节标定。Orin PC4 配置也不等于 `.161` 运动板配置。

## 7. 7301 / 3203 / 3104 与固件：最新证据和未解决内容

最有价值的新证据是 `scripts/audit_g1_installed_loco.py` 和 `g1_console/assets/safety_evidence_sources/installed_loco_sdk_20261022_v36.json`，窗口 **16:12:12–16:12:30**。它直接审计 Orin 实际加载文件；只放行固定官方读 API、空载荷、无 SDK lease，并用独立 DDS 接收器关联 request_id 和 api_id。

| 核查 | 实测与结论 |
| --- | --- |
| `LocoClient()`、`Init()` | 构造 0 次 RPC、Init 0 次 RPC；Init 只在本地设置协议/注册方法。不是漏了某个网络初始化动作 |
| sport API 1 | 成功 `1.0.0.0`，这是 API 协议号，不能填入运动固件 |
| sport 7001 / 7002 | 成功，FSM=0 / mode=0 |
| sport 7003–7006 | 四项均返回 **7301**，SDK 与独立收到的服务应答一致，值 null；本窗口 0 次 3104 |
| 被动公开数据 | 约 12 秒内 Go odom 619、HG sport 1194、HG lf sport 179 个样本；HG 的 task_id=4/time=0 语义未核验，不推定 StandUp 在执行 |
| 只读边界 | command_writes=0、lease_requests=0；没有为了就绪试发模式 |
| `.161` 版本读取 | 公开 GET 包版本曾返回 **3203 / API_NOT_IMPLEMENT**；历史 SSH:22、OTA:1026 拒绝连接，未读到运动二进制或其运行日志 |

**7301 = LOCO_STATE_NOT_AVAILABLE**；7302、7303 分别对应 FSM/task 不合法的其他错误，不要混为一个条件。当前公开代码只有客户端定义，没有本机闭源服务创建/有效性/就绪的确切判定；公开 odom、HG 广播存在不证明私有 LocoState 可用，也不能由未发现某个 DDS 名称证明对象不存在。

`ai_sport 8.7.5.212` 已收到显式名称/版本绑定广告，**尚未与 `.161 humanoid` 当前可执行构建建立可信对应**。App 软件 v1.5.6、PC4 组件版本、OTA 缓存版本和 API 协议号都不替代它。实际运动固件和精确 7301 条件保持未知。

新发布 UI 的 `LocoReadback.tsx` 提供“读取四项 LocoState”，依序查询四个固定 getter、不自动轮询/重试。应答 15 秒后过期、值撤回；7301 错误也有接收时间，不能保留占位 0 作为有效值。**16:25:35** 页面实际点击返回四次 7301；之后过期展示已核对。当前接口链路可达，不是修完传输超时就已经解决 7301。

本次四个 getter 的成功标准：收到 code=0 且有符合契约的实值、时效内可重复读取；随后还必须独立核验本机模式适用与控制归属。不得用“按钮能点击”“窗口 owner 已获得”“Init 完成”代替最终运动消费者证据。

## 8. 软件遥控器覆盖、官方起身和唯一电机测试

能力目录当前 **61 个操作条目**，跨运动/状态/手势/声光/查询；官方基础知识的 **76 个方法记录**含跨语言/重载，二者统计口径不同，均不是已实测动作数。固件 GetActionList 曾返回 20 个带 ID 项和 4 个未核定 ID 的项目；以实际读取为准，不给无 ID 名称自造编号。

| 入口 | 已确认的客户端映射 | 尚缺条件 |
| --- | --- | --- |
| 阻尼 | sport/7101，data=1 | 不是姿态保持或通用急停；单次实机凭据已用 |
| 站立预备 / 保持候选 | C++ StandUp，7101/data=4 | 与本机遥控锁定站立的等价性、进出条件未核验 |
| 蹲姿 | 7101/data=2 | 不能把坐地当蹲姿；姿态支撑未知 |
| 躺到站 | Python Lie2StandUp，7101/data=702 | 固件分支、规定躺姿与当前姿态适用性未知 |
| 蹲站双向转换 | 7101/data=706 | 蹲到站/站到蹲共用，不是可盲重发的幂等“站起” |
| 平衡站立 | BalanceStand，7102/data=0 | 需要本机模式和直立支撑条件；不是坐地起身，7003 仍拒绝 |
| 进入行走控制 | 7101/data=500 | 先有合法预备、支撑与控制权，尚未实测 |
| 用户/内部控制切换 | 7110 / 7111 | 是写操作，不能当客户端 Init；不随意退出官方平衡控制 |

官方 [23 DOF 遥控目录](https://support.unitree.com/home/en/G1_developer/remote_control) 的 >8.6.0.0 分支手册已保存为 `doc/official/G1_REMOTE_CONTROL_23DOF_20260407.pdf`，SHA256 `b50e7a6382f7069461c047825771d99a58ca387113f9b17667172427fc44d569`。它是已核对过的官方材料，后续若问“最新”要重新核对，不能以本文时点代替未来资料。

手册的椅坐路线包含人工扶直，不能直接套到坐地靠柜；自适应起身按键的公开 SDK 等价接口尚未核定。没有核验过的任意姿态缓慢起身接口/时长参数；不要把 `duration` 或速度参数伪装成起身减速控制。

**13:26:06 唯一一次电机模式测试：**发送 `SetFsmId(1)` 一次，SDK 返回 **3104**，原始结果仍 `outcome_unknown`；之后 GetFsmId 与 HG fsm_id 双路观察 0→1。未重发、未执行起身/行走/LowCmd，7301 在阻尼前后均存在。90 秒 792 样本内左肘最大变化 **23.4°**、峰值速度 **1.28548 rad/s**，原因未知，不能称整段静止。充电重启后 FSM 回到当前采样的 0。

凭据 `state/damping-test.json` 必须跨发布和重启保留。不能因“3104 没成功确认”重置或重试。此前语音官方应答成功，但实际播出未由可靠回读确认。项目窗口租约测试通过只证明浏览器互斥，**不是官方运动控制权已接管**。

软件停止（零速度请求）依赖软件与网络链路；Damp 是带电控制模式、可能丢失站立支撑；两者都不等于切断动力的独立硬件急停。官方遥控组合键也须依手册具体场景解释，不把舞蹈中断键扩大为通用硬急停。

`--manual-remote --verify-only` 只核对请求、不下发；`--accessory-test` 只允许 TTS/RGB；当前 `--damping-test` 仅单次阻尼。严禁为“补齐功能”移除这些门控或直接调用绕过 adapter。

## 9. 摄像头、Agent 观察与导航的真实完成度

相机使用官方 Video `GetImageSample` / API 1001，隔离进程读图；展示源 1920×1080、预览 960×540、约 5 Hz，并记录请求/接收时刻、age、sequence、超时窗口。最新两入口复核显示 live、连续成功；**只证明这个窗口的链路**。SDK_3102/3104 的调查和修正见相机稳定文档，不抢占官方 USB 或重启其服务。

`scripts/observe_g1.py` 通过 Mac 已登录 Codex CLI 做离散批次图像+身体解读，并把事实、变化、未知和建议提交 `insight.py`。它不是 Orin 本地常驻模型，也不是自动运动控制器；画面会发送给该推理服务，运行前读清输入和授权范围。4 个中文启动器中的 `启动G1自主观察.command` 带历史测试模型参数；不据此宣称未来模型可用性。

**观察 worker 要求强制只读服务，当前 manual+damping 服务不符合；启动会拒绝。** 不要为让 worker 跑起来放宽它的安全检查。想恢复连续观察，应明确切换本项目服务到只读并验证，或设计受限观察契约后定向测试。

导航已有几何算法、地图视图、定位/扫描桥接契约与合成演示。真实现场定位、地图、扫描一致坐标系和自主移动尚未验收；MCP、长期策略学习、任务级授权、完整可执行语义 Agent 均待实现。架构文档是设计和历史进度，不能全当成功能清单。

## 10. 无线断链已经定位到哪里

详见 `doc/G1_WIFI_DRIVER_READONLY_DIAGNOSIS_20261022.md`、`G1_ROTATED_LOG_RPC_AUDIT_20261022.md`。已证明某次开机先连上 `.28`，随后无线固件命令链路报错，再断链/重连失败，**不是单纯 SSH 没自启**；SSH 在机器可达时已查 enabled/active。

实机模块 `8852bu v1.19.3-45-g7583a804a.20230505`，内核 `5.10.104-tegra`，匹配 DKMS `/usr/src/rtl8852bu-1.19.3`，USB `0bda:a85b`。`halbb_fw_set_reg IO offload fail:1` 是包装层通用失败，不能解读成 Linux EIO 或确定的某个 MAC 错误。`Error Enable Power Control` 涉及发射功率寄存器，不等于 Wi-Fi 省电配置。轮转日志仍没保留足以确认首因的具体底层返回码。

已有官方 PC2 Wi-Fi 排障资料和较新驱动线索，但没有证明能修复本次故障；没有安装或加载新驱动。AP 端同期日志未取得，省电/AP 兼容/供电/温度/固件首因均未证实。用户明确选择暂不改配置，`G1_ORIN_WIFI_REPAIR_PLAN.md` 只是候选试验和回滚文档。

连接器解决的是“发现正确 Orin IP + 身份核验 + 项目服务/转发恢复”，不能自称根治无线固件失效。曾观察充电重启后恢复，不是已通过多次完整重启和长期稳定验收。

## 11. 开发环境、官方依赖与验证

Mac 优先 `.venv/bin/python`；Node/npm 使用当前可用环境，锁定依赖见 `requirements-console.lock.txt`、`requirements-mac-connection.txt`、`console/web/package-lock.json`。当前前端 React 19.3.1 / TypeScript 7.0.2 / Vite 8.3.4 / Three 0.186.1；这是锁定事实，不是要求跟随升级。新环境前端 `npm ci`，后端按锁文件安装，不全局改 Python。

系统 `/usr/bin/git` 和系统 Python 曾触发 Xcode license 阻塞。本机可用 Git：

```text
~/.cache/codex-runtimes/codex-primary-runtime/dependencies/bin/fallback/git
```

可给当前 shell 设置任务变量 `G1_GIT` 或直接绝对路径调用；勿擅自 sudo 接受许可。`scripts/sync_unitree_deps.py` 内部仍会找 `git`，若受影响先显式给该命令传入合适的 PATH，不修改系统配置。

`unitree-deps.lock.json` 锁定并可恢复：

| 仓库 | 提交 |
| --- | --- |
| `unitree_sdk2_python` | `814556d15970dd2ecf1c9984e845ca02ab07e206` |
| `unitree_rl_gym`（主要使用模型资源） | `276801e46c5d433564f24658bac64f254b7d2d4b` |

`externals/` 是被忽略的第三方源码目录，`.gitmodules` 声明不等于真的已跟踪 submodule。C++ SDK/ROS2 还用于带 SHA 的接口摘录和历史对照，具体 commit 以各审计 sources 清单为准，不把不同日期的核对提交混成当前运行版本。**没有学完宇树全部开源代码**，覆盖边界见 `G1_OPEN_SOURCE_COVERAGE.md`。

本次交接对当前本地源码重新运行的定向验证：

```bash
.venv/bin/python -m pytest -q tests/test_g1_mode_readback.py \
  tests/test_g1_motion_control_review.py tests/test_g1_manual_remote.py \
  tests/test_g1_damping_test.py
# 139 passed，1 warning，0.92 秒；不调用实机
```

上一轮 TypeScript 检查和 Vite 生产构建已通过；基础模型来源 SHA 校验/构建验证已通过；实机 7 项固定 getter、公开状态流、页面四项回读/错误过期和部署核验已完成。本次没有更改或重建前端；只读重新核验了远端 495 项哈希，也核对了当前本地包和 5 文件差异。曾有早期全量 414 passed，但不是当前树的全量结果，不能沿用。已知 Starlette/httpx 弃用提示和 BodyView 约 569 KB 分包警告未消除。

本次交接验证是：固定身份 SSH 读取进程/发布/模式/凭据摘要，远端 495 项 SHA 全匹配；两个 Mac 页面及 API GET 均 200、run_id 相同、owner=null、armed=false、相机 live；当前本地定向测试 139 passed；文档路径/JSON/清单哈希检查。**没有重跑物理动作或改变运行范围**。

本地合成 demo：`./scripts/start_g1_console.sh --demo`，访问 Mac `http://127.0.0.1:8765/`；默认离线启动器不等于连接机器人，合成演示也不是物理仿真安全证明。

## 12. 证据阅读顺序和接手后下一步

优先按时间和证据类型判断，实时状态 > 带时间的实机原始证据 > 已核对来源知识 > 历史总结 > 设计。持久化 `current=false` 是刻意声明历史，不应为界面好看改成 true。

| 先看什么 | 用途 |
| --- | --- |
| `doc/evidence/claude_handoff_20261022.json` | 本次整理的时间、连接身份、进程、范围、相机和 GET 复核摘要及来源 SHA |
| `.g1-deploy/research/claude-handoff-live-20261022.json`、`claude-handoff-mac-http-20261022.json` | 本机保留的原始只读核验；不依赖 `/tmp` 才能找到这些结果 |
| `.g1-deploy/research/claude-handoff-remote-manifest-20261022.json`、`claude-handoff-local-unreleased-20261022.diff` | 当前远端逐文件哈希，以及 5 个尚未部署的本地文件差异；不可将本地 release-id 当作 active-release |
| `g1_console/assets/safety_evidence_sources/installed_loco_sdk_20261022_v36.json` | 实际 SDK Init/7 getter/独立应答/三状态流证据，避免重复“漏初始化”猜测 |
| `.g1-deploy/research/deploy-v36.json`、`postdeploy-v36-refresh.json`、`postdeploy-v36-final.json` | 最近部署核验、32 次状态/3 帧和最终状态；是研究输出，不是现成部署工具 |
| `doc/screenshots/g1-locostate-expiry-20261022.png` | 最新 LocoState 面板实机回读及过期展示 |
| `doc/G1_REBOOT_REMOTE_VERIFICATION_20261022.md` | 较早 15:52 发布 `eea233656e28` 的重启/窗口控制与预检验收，版本已被本交接更新 |
| `doc/G1_CONTROLLER_READINESS_20261022.md`、`G1_LOCO_READINESS_DIAGNOSIS.md` | 历史版本/控制归属/7301 排除证据，旧“当前”不能覆盖本次快照 |
| `doc/G1_REMOTE_MANUAL_AND_DAMPING_TEST_20261022.md`、`doc/evidence/damping_test_20261022/` | 唯一阻尼测试、原始未知应答、90 秒轨迹；该文充电离线头部是历史，不代表现在离线 |
| `doc/G1_SOFTWARE_REMOTE.md`、`G1_ROBOT_FOUNDATION.md`、`G1_MOTOR_PARAMETER_AUDIT.md` | 遥控/身体参数设计，注意早期 47/53 操作统计与当前 61 条目的差别 |
| `doc/G1_CAMERA_STABILITY_AUDIT.md`、`G1_CAMERA_READONLY.md`、`G1_AUTONOMOUS_OBSERVATION.md` | 相机超时处理、只读观察工作流和约束 |
| `doc/G1_MAC_CODEX_ARCHITECTURE.md`、`G1_BODY_CONSOLE.md` | 目标架构/搭建，含待实现项与历史默认模式 |
| `doc/G1_REBOOT_RECOVERY.md`、`G1_ORIN_READONLY_DEPLOYMENT.md` | 恢复/初次部署历史；旧 IP/发布号/385 文件不当作现状 |
| `doc/G1_WIFI_DRIVER_READONLY_DIAGNOSIS_20261022.md`、`G1_ROTATED_LOG_RPC_AUDIT_20261022.md` | 无线固件失败链和轮转日志结论 |
| `doc/G1_FMX_ANALYSIS.md`、`G1_FMX_LOADER_SEARCH.md`、`G1_LARGE_LIBRARY_AUDIT.md`、`G1_OPAQUE_CONFIGURATION_AUDIT.md` | 电机配置探索中已做过的路径与未能证明的内容 |
| `doc/G1_PROGRESS_HISTORY.md`、`doc/evidence/claude_handoff_inventory_20261022.json` | 其他全部历史文档/实验/源码索引 |

接手任务建议按以下顺序推进：

1. 只读核验固定设备身份、当前地址、boot、项目 PID/cwd/发布/manifest、run_id、body/camera、owner/armed 与持久凭据。若在线复用，不因换编码工具重启机器人或系统服务。
2. 阅读已安装 SDK 审计，针对尚缺的 **`.161` 当前运行构建映射、私有 LocoState 的创建/有效性判定、官方支持的查询契约**提出可验证假设。优先官方资料/官方只读接口与可读日志，避免重复遍历全部目录和猜私有 RPC。
3. 区分传输 3104、服务拒绝 7301、未实现 3203。新增读取必须只允许固定已确认的读 API，有请求/应答关联、时间、原始失败和未知值，写入有来源 SHA 的基础档案，补相应定向测试。
4. 只有实际固件、模式路由、控制归属、当前支撑和停止/保护手段的适用证据齐备，才规划下一项简单动作；现有单次阻尼不能重发。不要为完成遥控器宣称当前可从任意坐地姿势起身。
5. 软件层可继续把缺失的诊断、部署复现和 UI 消费链补完整，按最终浏览器/真实反馈验证。每轮报告明确：代码改好、已部署、请求被服务接受、实机动作达到结果，四者分别给证据。

尚未解决的交接结论：**7301 具体触发条件、实际运动固件、完整控制权、起身/保持/平衡实机可用性、双脚/外部支撑承重、253 个电机配置字段、无线首因与长期重启恢复**。不要用重复“等待”、默认参数、放开门控或声光测试替代这些验收。
