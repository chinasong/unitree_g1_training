# G1 EDU 23 身体智能控制台

适配目标：G1 EDU、23 自由度、无手部执行器、Jetson Orin。2026-10-20 已部署到真实 Orin，以强制只读模式接通 23 关节、IMU、固件能力查询和官方相机；Mac 通过 SSH 转发打开图形界面。没有执行机器人动作。部署位置、恢复步骤与实测证据见 [Orin 只读部署记录](G1_ORIN_READONLY_DEPLOYMENT.md)。

Mac 本机 Codex 的接入与策略学习设计见 [Mac Codex + G1 架构设计](G1_MAC_CODEX_ARCHITECTURE.md)。2026-10-21 已新增 [Mac Codex 自主观察](G1_AUTONOMOUS_OBSERVATION.md)，结合官方相机与身体状态给出中文理解和任务建议；完整 MCP、任务授权与学习模块尚待实现。“训练”指能力学习与任务策略调试，不训练新动作模型。

## 打开软件

**真实机器人：**访问 <http://127.0.0.1:18765/>。双击项目根目录 `连接G1只读控制台.command`，保持窗口运行；连接器持续发现地址、核验已有 SSH 主机指纹、重建隧道，在项目服务缺失时恢复已部署的只读发布及 DDS 读取。SSH 密码只在核验指纹后输入，仅保留在进程内存。已有服务模式不符时保留原服务并报告，不自动切换。Orin 自身仍需先连入 Wi-Fi 并开放 SSH；当前该入口未恢复，实机重启验收待完成。见 [重启恢复方案](G1_REBOOT_RECOVERY.md)。Quantumult X 无需改动。

真实 G1 身体反馈，只读模式（截图 / 照片未公开，仅本地存档）

**Mac 本地开发 / 演示：**双击 `启动G1控制台.command`。它创建项目虚拟环境、安装锁定依赖、恢复锁定的官方源码、构建网页，然后自动打开浏览器。默认是未连接状态，不会初始化 DDS。

```bash
# 明确标记的合成演示，不加载机器人 DDS
./scripts/start_g1_console.sh --demo

# 普通启动，进入界面后选择连接
./scripts/start_g1_console.sh
```

地址：<http://127.0.0.1:8765/>。关闭启动终端中的服务可用 `Ctrl+C`。无需全局后台服务、登录项或云端账号。依赖：Python 3.11+、Git、Node.js 22.12+；有 uv 时会自动创建 Python 3.11 环境。首次启动需要访问包源和 GitHub。

界面包含六个页面：身体总览、环境观察、官方能力、自主任务、运行记录、连接与配置。全程显示演示 / 真实 / 未连接状态；演示遥测和演示地图不能作为实机验证。

G1 身体总览，当前为合成演示数据（截图 / 照片未公开，仅本地存档）

演示导航依据位置反馈验证到达，误差约 0.12 米（截图 / 照片未公开，仅本地存档）

## 身体能力清单

| 模块 | 已实现 | 数据与边界 |
| --- | --- | --- |
| 23 关节 | 官方 STL 交互模型；角度、速度、加速度、估计力矩、驱动与电机温度、电压、原始状态码 | HG `rt/lowstate`，CRC、递增 tick、时效检查 |
| IMU | 横滚、俯仰、偏航、角速度、加速度、四元数、温度 | 官方身体反馈；页面显示主要姿态指标 |
| 官方能力 | 61 个操作入口（运动 25、需额外条件 16、只读查询 14、声光 5、停止 1），参数表单、固件动作清单查询、结果与错误码 | G1 Loco / ArmAction / Audio 三个官方客户端 |
| 导航 | 地图选点、A* 路径、身体净空膨胀、扫描约束、官方速度执行、位置验证、有限次重规划 | 需要外部定位、占用地图与接近 360° 的扫描 |
| 相机 | 官方视频 JPEG 实时预览、来源和时效检查、手动保存当前帧 | 此 Orin 已实测官方 VideoClient；ROS Image 为另选桥接来源，不能混用 |
| 自主观察 | Mac Codex 定期分析真实画面、23 关节、IMU 和能力目录，显示事实、变化与建议 | 单批次分析；有来源、采样时间、过期标记和暂停；建议没有执行通路 |
| 执行记忆 | 目标、实测位姿、成功 / 失败 / 停止结果与事件日志 | 仅本机；演示与真实环境按模式、坐标系区分 |
| 手 | 明确禁用手指 / 抓取能力 | 用户设备没有手部执行器；官方参考网格包含固定橡胶手外形 |
| 电池 / 出厂参数 | 明确标记未知 | HG LowState 没有电池电量；不估算电量、Kp / Kd、持续力矩或出厂标定 |

23 关节采用官方 **稀疏电机编号**：双腿 `0..11`、腰部 `12`、左臂 `15..19`、右臂 `22..26`。不能把 23 个关节按数组 `0..22` 顺序写入机器人。页面上的角度范围、力矩范围和连杆质量来自锁定的官方仿真 XML，是参考值，不是实机持续安全上限。

能力目录覆盖锁定 Python SDK 的 G1 三个客户端公开业务方法，合并底层方法与便利别名，包含：行走 / 停止、蹲站 / 起身 / 坐下、站立高度、平衡 / 速度档位、官方臂任务、16 个预定义手势、固件动作 ID、用户 / 内部控制切换、TTS、音量、RGB 灯和 PCM 播放。

本机固件 `GetActionList` 已回读 **20 个动作 ID、4 个无 ID 命名条目**。界面保留其 FSM、mode_machine 和时长条件；无 ID 的条目只展示，不臆造调用编号。61 个 SDK 入口与固件声明分开展示，均不代表已经执行验证。能力目录仅允许 `GetFsmId`、`GetActionList`、`GetVolume` 三种查询；相机另通过官方 `GetImageSample` 只读获取。44 种写操作、控制权和导航请求均由服务端拒绝，包含停止、声光和模式切换。

实机固件能力清单，所有动作入口锁定（截图 / 照片未公开，仅本地存档）

“SDK 已定义”不代表这台机器的固件支持。手势必须由真实 `GetActionList` 声明后才启用；未知返回格式保持禁用。SDK 应答仅表示接口接受，导航到达必须以位置反馈验证。蹲站等 FSM 操作用新的 FSM 回读验证；其他手势尚无可靠完成事件，不能把应答当作动作完成。

## 在 Orin 上运行

这台 Orin 的实际安装已完成，使用独立 Python 3.11.15 与 CycloneDDS 0.10.2，DDS 网卡核验为 `eth0`。当前部署强制 `--read-only`，监听 `127.0.0.1:8765`，详情见部署记录。以下是其他设备从源码安装的参考；当前不要使用普通模式替换已运行的只读服务。ROS 桥接使用 ROS 发行版对应的 Python，不混用两个解释器。

```bash
# Orin 项目根目录，示例使用 uv
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -r requirements-console.lock.txt
.venv/bin/python scripts/sync_unitree_deps.py

# 按官方 SDK README 安装 CycloneDDS 后执行
# CYCLONEDDS_HOME 指向该设备实际的 CycloneDDS 安装前缀
uv pip install --python .venv/bin/python -e externals/unitree_sdk2_python
```

CycloneDDS 原生库安装遵循 [官方 SDK 安装说明](https://github.com/unitreerobotics/unitree_sdk2_python#installation)。如果库不在默认搜索路径，先设置 `CYCLONEDDS_HOME`；本项目已有 `.venv/opt/cyclonedds` 时，启动脚本会自动使用它。不要把 Mac 的 `.venv`、原生库或 `node_modules` 复制到 Orin。

前端可以在 Mac 构建，把 `console/web/dist/` 整个复制到 Orin 同一相对路径，再启动：

```bash
# Mac：生成与 CPU 架构无关的浏览器文件
cd console/web
npm ci
npm run build

# Orin：已放好 dist 后，不需要在 Orin 安装 Node
G1_SKIP_WEB_BUILD=1 G1_NO_BROWSER=1 ./scripts/start_g1_console.sh
```

Mac 通过 SSH 转发访问，不需要把机器人控制端口开放到局域网：

```bash
ssh -o ExitOnForwardFailure=yes -N \
    -L 127.0.0.1:18765:127.0.0.1:8765 '<Orin用户名>@<Orin地址>'
```

然后打开 Mac 的 <http://127.0.0.1:18765/>；该端口与 Mac 本地演示的 8765 分开。在“连接与配置”填写真实 DDS 网卡名，点击“连接真实机器人”。网卡 `eth0` 只是初始值，必须现场核查。DDS 网卡初始化后如需更换，重启本服务。

如果明确需要局域网直连，可设置 `G1_HOST`、至少 24 字符的 `G1_API_TOKEN`，以及逗号分隔的 `G1_ALLOWED_HOSTS`（实际 Orin IP / 主机名，不含协议和端口）；通配监听必须有明确主机白名单。远程浏览器需要输入该 token。该模式没有内置 TLS，优先使用 SSH 转发。token 不写入项目、URL 或日志。

## 接入真实观察与导航

此 Orin 已通过既有官方视频服务接通相机，无需 ROS 图像桥接。启动脚本设置 `G1_CAMERA_BACKEND=unitree`，初始化真实 DDS 后自动采样。实时查看与 Codex 快照见 [相机说明](G1_CAMERA_READONLY.md)。以下 ROS 桥接用于后续定位、地图、扫描，以及其他设备的可选图像输入；官方相机启用时拒绝外部图像注入，不能同时使用 `--image`。

软件不假设机器人现成提供可直接导航的 DDS 位姿。先在 Orin 上运行与现有传感器匹配的驱动、定位 / SLAM、二维投影扫描。桥接只读取传感器，不启动驱动，不训练模型，不发布低层电机命令或 ROS `cmd_vel`。

所需输入：

| ROS 2 输入 | 要求 |
| --- | --- |
| `sensor_msgs/LaserScan` | 接近 360°，至少 98% 有效量测；同一帧时间有扫描到身体坐标系的 TF |
| `nav_msgs/OccupancyGrid` | 与定位同一 map 坐标系；每边 4–160 格，分辨率 0.025–1 米；轴对齐原点 |
| TF | 在扫描采样时刻可查询 `map → base_link` 与 `base_link → 扫描帧`；拒绝用“最新 TF”代替旧帧 |
| `sensor_msgs/Image` | 可选；cv_bridge 转 BGR，再缩放为最大 640 像素、3 Hz JPEG 预览 |
| 时间 | 使用真实墙钟、设备时钟同步；不使用 ROS 模拟时间 |

在已加载 ROS 2 环境的终端运行（下面话题与 frame 均需替换为实机配置）：

```bash
python3 -m g1_console.ros_bridge \
  --url http://127.0.0.1:8765 \
  --scan /scan --map /map \
  --image /camera/color/image_raw \
  --map-frame map --base-frame base_link
```

桥接需要该 ROS 环境已有 `rclpy`、`tf2_ros`、`sensor_msgs`、`nav_msgs`、`cv_bridge`、OpenCV。上述默认话题不代表已确认 G1 的真实话题。大型地图需由定位系统输出符合范围的局部地图。桥接重启导致来源 ID 改变时，在界面断开后重新连接，避免把两个感知生产者混用。

真实导航启用前：获得新鲜身体反馈、FSM 回读为 500、地图 / 定位 / 扫描有效，并在取得控制权时确认平整场地。未知栅格不可通行。闭环每次重新观察身体、障碍物和位姿；地图变化或 5 秒没有足够进展会重规划，最多 3 次；90 秒超时失败；目标距离小于 0.13 米才报告到达。

当前演示是合成场景和运动学位姿反馈，不是物理动力学仿真，不能据此证明实机平衡、碰撞安全或定位精度。

## Agent 接口与 HomeBody 思路

借鉴 [HomeBody](https://github.com/Stanford-TML/homebody) 的任务调度、环境状态、能力执行与反馈分层；没有直接移植其硬件相关控制器。现有任务执行器接受几何目标；当前实机部署仍锁定全部动作。Mac Codex 已接入只读语义观察；自动探索未知房间、语义地图与视觉目标导航待实现。

```text
图形界面 / 上层 Agent
    ↓ 身体清单、能力目录、任务目标
单一运动仲裁器 + 控制权 / 时效检查
    ↓
官方 G1 Loco / ArmAction / Audio → 原生运动与平衡
    ↑                                      ↓
地图 / 定位 / 扫描 / 图像 ← 真实反馈 ← rt/lowstate
    ↓
位置与 FSM 验证 → 停止 / 重规划 / 本地执行记忆
```

上层 Agent 可以通过同源 HTTP 或 Bearer token 读取清单、观察状态、执行白名单能力，不能直接提交任意 RPC 或电机目标。

| 接口 | 用途 |
| --- | --- |
| `GET /api/body`、`GET /api/state` | 身体清单；当前遥测、能力可用性、地图、任务与事件 |
| `WS /api/live` | 5 Hz 状态推送 |
| `GET /api/camera.jpg`、`GET /api/camera/status` | 已认证的当前 JPEG 与相机元数据；图像过期超过 1 秒返回 404 |
| `POST /api/insight/control`、`/heartbeat`、`/capture`、`/result` | 自主观察开始 / 暂停、Mac 心跳、采样与报告；只返回建议 |
| `POST /api/connect`、`/api/disconnect` | 切换真实 / 演示连接 |
| `POST /api/control/acquire`、`/heartbeat`、`/release` | 单一控制权；传入固定 owner；acquire 包含 `floor_verified` |
| `POST /api/command` | `owner`、`capability`、`params`、`request_id`、`issued_at`、递增 `sequence`；高风险动作需 `acknowledge_risk` |
| `POST /api/goal` | `owner`、`x`、`y`、`request_id`、`issued_at`、递增 `sequence` |
| `POST /api/observation`、`/api/camera` | 已认证的真实传感器桥接，模式、来源、序号与时效检查 |

POST 必须包含 `X-G1-Intent: operator`。`issued_at` 为 Unix 秒；owner 使用 16–64 字符的字母、数字或连字符标识。请求 ID 防重复执行，sequence 防止迟到的旧移动指令在停止后重新启动。运动请求不要自动重试。停止行走在普通控制模式下可于控制权丢失或遥测过期后发送，但需有效的控制台身份；当前只读部署拒绝该操作。

控制权心跳有效期 2 秒，速度有效期 250 ms，速度上限为前后 0.3 m/s、横向 0.2 m/s、旋转 0.4 rad/s，唯一定义在 `g1_console/contracts.py` 的 `MAX_VX / MAX_VY / MAX_OMEGA`；这些是本项目保守限值，不是官方硬件性能极限。界面松开移动按钮、窗口失焦、反馈过期或检测到障碍时停止继续发速度。掉线恢复不自动恢复移动。日志与最近 40 条任务记忆保存在忽略的 `.g1-console/`，不保存相机图片或连接密钥。

“停止行走”调用零速度，保留官方控制器；阻尼、零力矩和切换用户控制会改变支撑状态，单独列为需核查现场条件的操作，不能代替机器人实体急停。软件不能保证机器人永不摔倒；二维平面扫描无法覆盖落差、全部台阶、地面摩擦变化或手臂扫掠碰撞。

## 已验证与待验证

2026-10-22（北京时间，UTC+8）全量：项目根目录 `pytest.ini` 限定 `testpaths = tests`，`.venv/bin/python -m pytest -q` 共 **414 项**；能力目录 **61 项**。以下各段为当时记录，数字以当时为准。

2026-10-20（北京时间，UTC+8）在 Mac 完成：

- 32 项定向测试：23 关节稀疏映射、真实 SDK HG 消息 / CRC、全部 47 操作接线与错误码保留、控制权超时、独立速度超时、迟到命令拦截、旧 / 假感知拦截、有限重规划、合成位姿到达、HTTP / WebSocket 身份边界、只读三层拦截与关闭不发动作、实机嵌套固件清单解析，以及官方相机超时恢复、JPEG 缩放、来源与时效、断连清理、快照地址约束。
- TypeScript 检查和 Vite 生产构建通过。官方身体模型单独延迟加载；Three.js 分块仍超过 500 KB，构建有体积提醒。
- 官方依赖 SHA 校验、15 个 SDK 符号导入、HG 序列化 / CRC、23 / 29 DOF MuJoCo 各 100 步离线检查通过。Linux timerfd 线程不能在 Mac 实跑。
- 浏览器实际操作：右肩编号 22 与关节详情、取得控制权、地图目标导航，演示位置误差约 0.12 米时报告到达。

```bash
CYCLONEDDS_HOME="$PWD/.venv/opt/cyclonedds" .venv/bin/python -m pytest tests/test_g1_console.py tests/test_g1_camera.py -q
CYCLONEDDS_HOME="$PWD/.venv/opt/cyclonedds" .venv/bin/python scripts/check_unitree_compat.py
cd console/web && npm run build
```

测试依赖另需 `pytest`；SDK 边界测试需要安装官方 SDK 和 CycloneDDS。单元测试仅注入替身 / 构造消息，不向真实机器人发送动作。

2026-10-20 23:17:35（北京时间，UTC+8）实机验证：Orin 上连续 30 帧均有 23 关节反馈，tick 递增，数据年龄 15–22 ms，CRC 异常 0；FSM、20 个固件动作 ID 与 4 个命名条目回读成功，Mac 图形界面可见真实身体和能力清单。数据年龄不是 Mac 到 Orin 的网络往返延迟。证据见 [验证 JSON](G1_ORIN_READONLY_VERIFICATION.json)。

2026-10-20 23:51:37（北京时间，UTC+8）相机实机验证：官方原始 1920×1080，Mac 预览 960×540，连续 24 张 JPEG 序号与内容均不同；同时 23 关节持续反馈。Mac Codex 已用快照脚本取图并视觉查看；浏览器显示“实时画面”。证据见 [相机验证 JSON](G1_CAMERA_VERIFICATION.json)。

2026-10-21 自主观察更新：三组相关测试合计 **45 项通过**，新增真实输入校验、身份与租约、暂停与断流失效、未知能力、历史结果和无动作通路验证；修复 CLI 的无工具启动警告被误识别为工具调用。TypeScript / Vite 构建通过。实际模型、采样批次、耗时与零动作状态见 [观察验证 JSON](G1_OBSERVATION_VERIFICATION.json)。运行方法见 [自主观察说明](G1_AUTONOMOUS_OBSERVATION.md)。

尚未验证 / 接入：完整电池电量、定位 / 地图 / 扫描与 ROS 桥接、Orin GPU 推理、真实行走 / 平衡、任何动作执行与自主导航。当前验证覆盖 Orin CPU 上的只读服务、DDS、官方 RGB 取图、Mac 网页与按需视觉查看；后续动作阶段需另行明确任务范围。
