# Mac Codex 自主观察

2026-10-21，北京时间（UTC+8）。适配 G1 EDU 23 DOF、无手、Orin。此模块结合官方相机和身体遥测生成中文观察及任务建议，不训练模型，也不执行机器人动作。

## 使用

1. 保持 Orin 的只读服务运行，双击 `连接G1只读控制台.command` 建立 Mac 转发。
2. 双击 `启动G1自主观察.command`，保持终端运行；重复启动会被项目锁拒绝。
3. 打开 <http://127.0.0.1:18765/> 的“环境观察”，点击“开始观察”。可随时“暂停观察”。`Ctrl+C` 结束 Mac 观察进程。

服务初始为暂停；Orin 控制台重启后需要重新连接真实 DDS、重新开始观察。没有添加系统自启或登录项。Mac 转发入口与本地演示 8765 分开；观察进程拒绝演示输入。

启动器使用本账号 CLI 实测支持视觉输入的 `gpt-5.6-sol`，推理强度读取当前 Mac Codex 配置。2026-10-21 实测桌面配置的 `gpt-6.1-sol` 被 CLI 账户接口拒绝，因此只为此观察进程明确指定模型，不改全局配置、不默默降级。可在终端明确覆盖：

```bash
.venv/bin/python scripts/observe_g1.py --model gpt-5.6-sol --interval 30
```

优先使用 Codex.app 自带 CLI，复用其现有 ChatGPT 登录态。没有读取或复制登录凭证。Mac 是采样与调度端，模型服务完成推理；不是把模型权重装到 Mac 或 Orin。图像与身体状态会提交给用户已登录的 Codex 服务。每次观察使用临时图像、JSON Schema 和结果文件，结束即清理；项目只保留进程日志，不持续录像。

## 数据流与边界

```text
Orin：官方 VideoClient + HG LowState / CRC + 能力目录
  → 同批次 JPEG、23 关节、IMU、能力可用性、数据来源与时间
  → SSH loopback → Mac 单一观察进程 → Codex 结构化视觉分析
  → Orin 校验批次 / 来源 / 结构 / 能力 ID
  → WebSocket → 图形界面：环境事实、身体解读、变化、不确定项、任务建议
```

- 采样要求：真实机器人、强制只读、新鲜身体反馈、官方相机。Mac 收到后再次按完整请求 RTT 保守检查：图像请求帧龄小于 1 秒、身体接收年龄小于 600 ms。官方接口没有曝光时间戳，不能把帧龄解释为曝光延迟。
- 默认每 30 秒尝试新一轮；Codex 返回 429 限流时，按 60、120、240、300 秒逐步退避，心跳及实时预览保持刷新。单轮推理最长 120 秒；一次只有一个分析。实时视频独立刷新，语义分析是离散采样，不保证捕获两帧之间的事件。
- 观察 worker 租约 20 秒，运行和推理中每约 3 秒续期。每批输入有唯一 ticket、相机来源、序号、JPEG SHA256、身体 tick、服务 run_id 和采样时间。
- 批次最长 150 秒。暂停、断连、服务重启、来源变化使未完成结果失效。画面断流或身体反馈失效时拒收结果。
- 结果在采样后 120 秒过期；暂停、worker 离线、图像或身体失效时，已有结果标为历史。继续刷新不表示旧判断重新生效。
- 结果分开标明视觉 / 身体证据与置信度。单帧 RGB 不能证明距离、摩擦、接触、支撑、完整通路或机器人不会摔倒。FSM / 电机原始状态码语义未知时保留未知。没有手、电池、深度、定位或地图时不补造能力与数据。

Mac worker 的 HTTP 客户端只允许 session、heartbeat、capture、result；不具有 command、goal、控制权或连接接口。Codex 使用临时目录、`--ephemeral`、`--ignore-user-config`、read-only sandbox；关闭 shell、浏览器、Apps、插件、MCP 配置、记忆及多 Agent 等工具。若输出事件出现工具调用，结果会被拒绝。图像文字和遥测视为数据，不能变成工具指令。

任务建议只显示文字、前置条件和已知官方能力引用，没有执行按钮或自动转换为任务的通路。Orin 的现有只读三层拦截仍拒绝全部 44 种 SDK 写操作，包括停止、灯光、语音及模式切换。

## 接口与代码

所有接口沿用控制台身份认证、同源限制、`X-G1-Intent: operator` 和请求体上限。

| POST 路径 | 用途 |
| --- | --- |
| `/api/insight/control` | 用户开始 / 暂停观察，仅改本模块状态 |
| `/api/insight/heartbeat` | 注册单一 Mac worker，返回当前 enabled 与 run_id |
| `/api/insight/capture` | 获取有唯一 ticket 的新鲜图像与身体输入 |
| `/api/insight/result` | 提交严格结构化报告或有限错误分类 |

`GET /api/state` 与 `WS /api/live` 的 `insight` 包含进程在线、采样 / 分析 / 历史状态与报告。实现分别在 `g1_console/insight.py`、`scripts/observe_g1.py`、`console/web/src/ObservationView.tsx`。

验证命令：

```bash
CYCLONEDDS_HOME="$PWD/.venv/opt/cyclonedds" .venv/bin/python -m pytest tests/test_g1_console.py tests/test_g1_camera.py tests/test_g1_insight.py -q
cd console/web && npm run build
```

实机结果见 [观察验证记录](G1_OBSERVATION_VERIFICATION.json)。当前能力限于观察与建议；长期语义地图、任务策略记忆、深度 / 接触估计和自主执行尚未接入。

2026-10-21 已完成至少 5 次实机分析，早期成功轮次耗时约 27–55 秒。结果识别出瓷砖、鞋类、设备及圆形小物体，并结合 23 关节、IMU 和硬件清单给出不确定项及建议。暂停后报告成为历史，恢复后重新完成了多轮分析。已确认部分模型请求失败为服务返回 429 限流，并加入明确提示及退避；相机也出现过取图超时。程序会自动重试，不能承诺每轮成功或固定延迟。45 项相关测试通过，TypeScript / Vite 构建通过；实机验收没有控制权持有者、没有接受任何动作指令，导航保持 idle。

界面截图：观察摘要（截图 / 照片未公开，仅本地存档）、任务建议（截图 / 照片未公开，仅本地存档）、暂停与历史标识（截图 / 照片未公开，仅本地存档）。Quantumult X 未修改。

Codex 用法依据官方 [非交互模式](https://learn.chatgpt.com/docs/non-interactive-mode) 和 [CLI 参数](https://learn.chatgpt.com/docs/developer-commands?surface=cli)。
