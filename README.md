# Unitree G1 Training Project

面向 **G1 EDU 23 自由度、无手部执行器、Jetson Orin** 的身体智能控制台与实机验证记录。复用官方运动服务，不训练新动作模型。真实 DDS 控制服务运行在 Orin / Linux，Mac 作为图形客户端。

## 当前状态（2026-10-22 16:35，Asia/Shanghai）

- **部署**：2026-10-22 21:04 起发布 `3283959a54da`（驾驶页、一键启停 + 姿态门控、座椅启动停用、菜单精简）以完整人工遥控范围运行在 Orin `<ORIN_IP>`；Mac 用 `连接G1软件遥控器（行走）.command` 连接。切回阻尼范围：`scripts/deploy_orin_console.py --scope damping`。
- **本地与运行版**：Orin 运行 `3283959a54da`；本地新包 `9356e2202908`（按证据开放路线、座椅启停均关闭、电机温度门控）待部署。20:55 座椅一键启动倾倒的复盘见 [G1_SEQUENCE_TIP_20261022.md](doc/G1_SEQUENCE_TIP_20261022.md)；之前的 `efa485eedb04`、`03034ef80b29`、`da5d189707fa` 保留在 Orin 可回滚。
- **身体**：23 关节与相机在线，双路 FSM **0**，双膝约 **160.7° / 158.5°**；脚底载荷、外部支撑和独立站稳未知。
- **已验证**：页面取得 / 释放控制权，起身、姿态保持、平衡请求的参数核对（均未发送动作）；单次阻尼 `SetFsmId(1)` 使 FSM 0 → 1（SDK 应答 3104 接收超时，未重发）；语音连接测试收到官方成功应答（实际播出未回读）。
- **未验证 / 阻塞**：起身、姿态保持、独立平衡和行走均不可宣称可用。只读平衡查询仍返回 **7301**，包版本查询仍返回 **3203**；`ai_sport 8.7.5.212` 仅确认服务广告，实际运动固件未核定。无线断链根因未确认。
- **安全边界**：最终无控制者、未授权运动；未修改 Quantumult X 或系统网络。

交接入口见 [CLAUDE.md](CLAUDE.md) 和 [项目、G1 连接及工作台完整交接](doc/CLAUDE_CODE_HANDOFF_20261022.md)，含最新核验、源码索引及未解决事项。较早 15:52 的结果见 [本轮重启与遥控验收](doc/G1_REBOOT_REMOTE_VERIFICATION_20261022.md)；此前的带日期记录见 [进展历史](doc/G1_PROGRESS_HISTORY.md)。大量控制台实现尚未提交 Git，请保留现有工作区。

## 入口

| 入口 | 作用 |
|---|---|
| `启动G1控制台.command` | Mac 本地开发 / 演示，默认不连接机器人 |
| `连接G1软件遥控器.command` | 连接 Orin，以 `--manual-remote --damping-test` 运行：唯一可写操作是单次阻尼 FSM 0→1，Orin 上已有测试凭据，不会重发；起身和行走锁定。打开 SSH 转发入口 `18765` |
| `连接G1只读控制台.command` | 连接 Orin，强制只读 |
| `连接G1软件遥控器（行走）.command` | 连接 Orin 上的完整人工遥控服务（行走 / 手势 / 停止）；只接受 `manual` 范围，起身仍由实体遥控器完成。流程见 [G1_WALKING_FIELD_PROCEDURE.md](doc/G1_WALKING_FIELD_PROCEDURE.md) |
| `scripts/deploy_orin_console.py --scope <readonly\|verify\|accessory\|damping\|manual>` | 部署本地打包并按范围重启 Orin 服务；需空闲，失败自动回滚，保留阻尼凭据，不发机器人指令 |
| `启动G1自主观察.command` | Mac Codex 结合画面、关节与 IMU 自动观察，不执行动作；要求只读服务，当前人工遥控服务不符合 |

`18766` 是另行启动的 Vite 预览（`.g1-deploy/mac-readonly-preview.config.ts`），代理到 `18765`；上述启动器不会自动开启它。

本地合成演示（首次需联网安装锁定依赖），之后访问 <http://127.0.0.1:8765/>：

```bash
./scripts/start_g1_console.sh --demo
```

控制台运行模式（`python -m g1_console`）：`--demo`、`--read-only`、`--manual-remote`，后者可叠加 `--verify-only`（只核对接口，禁止实机写入）、`--accessory-test`（只开放语音与 RGB 灯）或 `--damping-test`（单次阻尼）。约束见 [g1_console/config.py](g1_console/config.py)。

保存一帧相机画面供视觉查看：

```bash
.venv/bin/python scripts/read_g1_camera.py --url http://127.0.0.1:18765 --manual-remote --output /tmp/g1-current.jpg
```

## 文档

| 主题 | 文档 |
|---|---|
| 控制台安装、感知接入与验证边界 | [G1_BODY_CONSOLE.md](doc/G1_BODY_CONSOLE.md) |
| 软件遥控器设计与边界（历史统计 53 项，当前目录 61 项；均不等于已实测动作数） | [G1_SOFTWARE_REMOTE.md](doc/G1_SOFTWARE_REMOTE.md) |
| 软件遥控行走现场流程（阶段 A，未实机执行） | [G1_WALKING_FIELD_PROCEDURE.md](doc/G1_WALKING_FIELD_PROCEDURE.md) |
| 相机只读接入 | [G1_CAMERA_READONLY.md](doc/G1_CAMERA_READONLY.md) |
| Codex 自主观察 | [G1_AUTONOMOUS_OBSERVATION.md](doc/G1_AUTONOMOUS_OBSERVATION.md) |
| 身体基础档案（23 电机参数、字段定义） | [G1_ROBOT_FOUNDATION.md](doc/G1_ROBOT_FOUNDATION.md)、[电机参数核查](doc/G1_MOTOR_PARAMETER_AUDIT.md) |
| 官方开源代码覆盖范围 | [G1_OPEN_SOURCE_COVERAGE.md](doc/G1_OPEN_SOURCE_COVERAGE.md) |
| Orin 只读部署 | [G1_ORIN_READONLY_DEPLOYMENT.md](doc/G1_ORIN_READONLY_DEPLOYMENT.md) |
| 下一阶段：Mac Codex + G1 Orin 架构（MCP、任务授权、策略学习待实现） | [G1_MAC_CODEX_ARCHITECTURE.md](doc/G1_MAC_CODEX_ARCHITECTURE.md) |
| 官方依赖版本与兼容性 | [UNITREE_UPSTREAM_UPDATE.md](doc/UNITREE_UPSTREAM_UPDATE.md) |
| 带日期的进展记录 | [G1_PROGRESS_HISTORY.md](doc/G1_PROGRESS_HISTORY.md) |

`doc/` 下其余文件为各轮审计与实机验证的 Markdown 记录和 JSON 证据。

## 项目结构

- `g1_console/`：控制台后端（FastAPI），含控制权租约、执行门控、相机、观察与模式回读。
- `console/web/`：控制台前端（React + three.js + Vite）。
- `scripts/`：Orin 打包部署与连接、只读审计、身体 / 能力档案构建、依赖同步。
- `tests/`：Python 测试。
- `.g1-deploy/`：Orin 离线 wheels 与发布包。
- 早期实验工具（不属于控制台）：
  - `armdev/`：手臂数据处理、CSV → MuJoCo / lowcmd 回放
  - `recorddata/`：ROMP 动捕转 G1 CSV、平滑与可视化
  - `cameradev/`：相机、人脸识别与语音对话
  - `videodev/`、`legdev/`、`g1dev/`、`trainging/`：视频转 CSV、腿部、设备数据、右臂录制
  - `g1_douji_voice_chat.py`、`douji_voice_chat_ubuntu.py`：“豆几”语音对话（Sherpa-ONNX + DeepSeek + Edge-TTS）

## 依赖与初始化

官方源码放在被忽略的 `externals/` 下，精确 commit 记录在 `unitree-deps.lock.json`。`.gitmodules` 中的声明不会下载它们（项目没有已跟踪的 submodule 条目）。

```bash
git clone https://github.com/chinasong/unitree_g1_training.git
cd unitree_g1_training
python3 scripts/sync_unitree_deps.py
python3 scripts/sync_unitree_deps.py --check
```

早期实验工具需要的官方包：

```bash
pip install -e externals/unitree_sdk2_python
pip install -e externals/unitree_rl_gym
```

早期工具用法示例：

```bash
python armdev/armdev_data.py
python armdev/g1_csv_to_lowcmd_player_with_legs.py
python armdev/g1_csv_to_mujoco.py
python recorddata/smooth_g1_csv.py
cd cameradev && python g1_humorous_chat.py
```
