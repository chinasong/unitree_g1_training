# G1 项目接手入口

请先读 [完整交接](doc/CLAUDE_CODE_HANDOFF_20261022.md)，再按其中的证据索引定位源码。交接快照为 **2026-10-22 16:35，Asia/Shanghai（UTC+8）**，接手时须重新只读核验在线状态。

- 项目：`.`；G1 EDU、23 自由度、无手、Orin。目标是复用官方动作学习任务策略，不训练运动模型。
- **大量实现尚未提交 Git**。HEAD 为 `e2f688b1bc2cb8327e4b89f2c3a36602a7bdcaed`；仅克隆 GitHub 无法取得当前工作台。保留现有未提交和未跟踪文件，不 reset/clean。
- 交接时 Orin 为 `unitree@<ORIN_IP>`，固定主机指纹 `<ORIN_HOST_FINGERPRINT>`。通过 `scripts/connect_orin_console.py` 先验指纹、后认证；Orin 地址与固定指纹只存本机未跟踪的 `.g1-console/orin.json`（`orin_identity()`，缺失即拒绝连接），源码与公开快照不含；不将 App 的 `<APP_REPORTED_IP>` 当作已验证的 Orin。
- 图形入口 `http://127.0.0.1:18766/`，代理到 SSH 桥 `18765`，再到 Orin `127.0.0.1:8765`。先复用已有服务，勿占用同一端口再启动。
- 2026-10-22 21:04 起运行发布 `3283959a54da`，**完整人工遥控范围**（`--manual-remote`），run_id `dcb801b47e73`，部署记录 `.g1-deploy/research/deploy-20261022-210443.json`。含驾驶页、按官方路线的一键启动 / 停止（`g1_console/mode_sequence.py`，每步双路回读 + 关节静止 + **姿态门控**：任一时刻倾斜 > 0.8 rad 中止，站立类步骤终点须 ≤ 0.35 rad，倾斜时拒绝启停）与阻尼保护、菜单精简。**本地新包 `9356e2202908` 待部署**（`b65c178`：按 `mode_sequence.EVIDENCE` 开放路线，座椅启停均关闭；电机 ≥ 70 ℃ 拒绝 / 中止一键启动）。**座椅路线一键启动已停用**：20:55 实测锁定站立（FSM 4）后机器人倾倒，无人受伤；事后落座保持使左髋绕组升到 79 ℃，现场 L2+B 阻尼后关机（复盘 `doc/G1_SEQUENCE_TIP_20261022.md`）。蹲姿路线与一键停止未实机验证。行走 / 手势须取得控制权 + 现场四项确认（60 秒授权）；单独模式切换仍锁定。切换范围用 `scripts/deploy_orin_console.py --scope <...>`，现场流程见 `doc/G1_WALKING_FIELD_PROCEDURE.md`。唯一阻尼测试凭据已使用且保留，不能删除、重置或重发。
- 之前的发布 `efa485eedb04`、`6588d79ff31c`、`6a9753508b9a`、`03034ef80b29`、`da5d189707fa`（damping-test 范围）仍在 Orin releases/ 下，可用部署工具切回。最新部署记录 `.g1-deploy/research/deploy-20261022-210443.json`。
- 7301 仍未解决，实际运动固件版本未知。`ai_sport 8.7.5.212` 是收到的服务广告；`1.0.0.0` 是协议版本。起身、姿态保持、平衡和行走尚未完成实机验证。
- 身体基础档案 `g1_console/assets/g1_motor_knowledge.json` schema 27。参考模型值、SDK 默认值、历史读数与实机配置分开；未知保持 null。23 关节索引为 `0..12,15..19,22..26`。
- **Quantumult X 不动**。用户已明确暂不修改 Orin Wi-Fi 省电；系统网络、sudo、官方服务、系统权限及认证材料修改先确认。项目内代码、测试、文档和自己的服务可按已授权范围工作。
- 接手先只读核验连接、部署、身体反馈和凭据。摄像头看不到脚底/背部支撑；不能据此确认承重、独立站稳或“不会摔倒”。历史简单动作授权不补足现场适用证据，不能为试探 7301 随意切换模式。
- 已有唯一电机模式写入为 13:26 的一次阻尼：SDK 3104、双路 FSM 0→1；窗口内左肘变化约 23.4°。不能称为全程静止测试，也不能因超时重试。

优先阅读：交接总览 → `doc/evidence/claude_handoff_20261022.json` → `scripts/audit_g1_installed_loco.py` 及其 v36 证据 → `g1_console/mode_readback.py`、`service.py`、`capabilities.py` → 对应定向测试。

Mac 使用 `.venv/bin/python`。系统 Git/Python 曾因未接受 Xcode license 失败；可用交接中列出的 bundled Git，勿擅自 sudo 接受许可。不要把 shell 临时 session ID 或旧 PID 当作持续有效的服务身份。

## Claude / Codex 分工（2026-10-22 起）

目的：Claude Code 额度只花在判断上，执行型工作交给本机 Codex。

| 交给 Codex | 留给 Claude |
| --- | --- |
| 规格写清的功能与测试、前端面板、离线工具 | 安全门控与执行路径（下列受保护文件）的修改 |
| 文档维护、数字与日期一致性、证据整理 | 根因分析、实验设计、对 Codex 改动的审查与合并 |
| 大体量离线分析（审计输出、日志汇总） | 一切实机操作、部署、需要用户决定的事项 |

派活：Claude 写任务单（目标、`允许修改:` 段落每行一个路径或 glob、验收测试、禁止事项），然后运行

```bash
.venv/bin/python scripts/dev/codex_task.py <任务单.md> --slug <短名> [--effort high]
```

- 脚本优先用 Codex 桌面版自带的 CLI（npm 版过旧），默认模型 `gpt-5.6-sol`（config 默认模型在 ChatGPT 账号下拒绝 exec）。WebSocket 连不上时会回退 HTTPS，日志里的 TLS 报错和 MCP 授权报错可忽略。
- 脚本要求主工作区已提交；在 `.worktrees/<slug>` 新建分支 `codex/<slug>`，以 `workspace-write` 沙箱（无网络）运行 `codex exec`，完整过程记录在 `.codex-runs/<时间>-<slug>/codex.log`，不进入 Claude 上下文。
- 结束后输出 `result.json`：Codex 的结构化报告、实际改动文件、是否触碰受保护文件或超出允许范围、自报文件与实际是否一致。
- **受保护文件**（见 `scripts/dev/codex_task.py` 的 `PROTECTED`）：`g1_console/{service,adapters,capabilities,config,contracts,app,mode_readback}.py`、连接 / 恢复 / 打包脚本、`*.command`、`CLAUDE.md`、`scripts/dev/`、依赖锁定与 `.gitignore`。Codex 只能在报告中提建议。
- Codex 不提交、不联网、不碰机器人。Claude 不采信自报测试结果，自己重跑后再在主工作区合并（`git merge codex/<slug>` 或挑选文件），然后 `git worktree remove .worktrees/<slug>`。
- 同一时间同一文件只有一方在改；Codex 在 worktree 工作期间，Claude 不改它允许修改的文件。
