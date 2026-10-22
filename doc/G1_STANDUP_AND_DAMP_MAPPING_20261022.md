# 离线核查：手册 ② 锁定站立与 L2+B 阻尼的 SDK 对应（2026-10-22 21:20，Asia/Shanghai）

只读官方资料与锁定 SDK，未接触机器人。起因见 [座椅一键启动倾倒复盘](G1_SEQUENCE_TIP_20261022.md)。

## 资料

| 来源 | 位置 |
| --- | --- |
| 官方 23 DOF 遥控手册 2026-04-07 | PDF（官方 PDF 未随公开仓库发布，请从宇树官网获取），逐项核对记录 `g1_console/assets/safety_evidence_sources/official_remote_manual_20261022_v34.json` |
| C++ SDK G1 loco 客户端 | `g1_console/assets/capability_sources/g1_loco_client.hpp:214-224` |
| Python SDK（锁定 `externals/unitree_sdk2_python` @ `814556d1`） | `unitree_sdk2py/g1/loco/g1_loco_client.py:119-138`；示例 `example/g1/high_level/g1_loco_client_example.py` |

## 结论 1：手册 ② 锁定站立 → 候选 `StandUp()` = `SetFsmId(4)`，等价性仍未核定

- 手册：② 锁定站立 = **L2 + ↑ 短按**；座椅路线 `开机 → ① → ② → ③ → 演示 → ④ → 关机`，**② → ③ 之间需要人工扶直站立**。
- C++ SDK：`StandUp() { return SetFsmId(4); }`（hpp:222）。这是 SDK 中唯一把 FSM 4 命名为“站立”的入口，名字不叫“锁定站立”。
- Python SDK 的 G1 客户端**没有** FSM 4 的辅助函数（只有 Damp 1 / Start 500 / Squat2StandUp·StandUp2Squat 706 / Lie2StandUp 702 / Sit 3 / ZeroTorque 0）；官方 G1 示例的起身路线是 `Damp → 0.5 s → Squat2StandUp`（蹲姿）或 `Damp → Lie2StandUp`（躺姿），**没有座椅路线示例**。
- 其他机型里 FSM 4 名称不同：H2 `StandUp()`、R1 `Stance()`；同一 FSM 号在不同机型语义可能不同，不能互相推导。
- 资料中**没有**一处写明“L2+↑ 等于 SetFsmId(4)”。项目自己的能力目录早已标注 `prepare` 为“本机固件和当前坐姿适用性未核定，实机锁定”（`g1_console/capabilities.py:21-22`）。**一键启动的座椅路线没有遵守这条标注，直接把 FSM 4 当成了 ②——这是设计阶段的错误，不只是姿态检查缺失。**
- 20:55 实测与两种解释都相容：(a) FSM 4 就是 ②，而 ② 本来就需要人在发送前扶住（手册要求人工扶直）；(b) FSM 4 不是 ②。离线资料无法区分。

## 结论 2：L2+B 是否经过 FSM 1 —— 资料未写明，本机无实测证据

- 手册：① 阻尼 = **L2 + B 短按**；运控中“阻尼保护”= **L2 + B 长按 5 秒**。手册只描述进入阻尼，不提 FSM 号。
- SDK：`Damp()` = `SetFsmId(1)`（Python 与 C++ 一致）。13:26 的唯一一次 SDK 阻尼双路回读 FSM 0→1——这证明的是 **SDK 调用 → FSM 1**，不是实体按键。
- 21:07–21:09 现场按 L2+B 后关机：监控每 5 s 一读，断联前所有读数都是 FSM 3，**未读到 FSM 1**。可能是阻尼到关机不足 5 s，也可能实体按键的阻尼不经过 loco FSM 回读。无法判定。
- 项目遥测目前**不记录实体遥控器按键**，所以历史数据里找不到答案。

## 可做的只读核验（需现场，由用户决定时间）

HG `rt/lowstate` 自带 `wireless_remote[40]`（SDK `idl/unitree_hg/msg/dds_/_LowState_.py:31`），按键位按 `unitree_sdk2py/utils/joystick.py:108-129` 解析（字节 2–3：LT/RT/back/start/LB/RB、方向键与 ABXY）。在遥测中同时记录按键位与双路 FSM，就能**只读**观察：

1. 机器人吊住或端坐、FSM 0：现场按 L2+B → 是否读到 FSM 1、耗时多少；
2. （仅在问题 1 成立且人工扶持就位时）FSM 1 下按 L2+↑ → 是否读到 FSM 4。

软件不发任何指令，只录按键与回读。这一步需要改遥测采集（受保护的 `adapters.py`），要先实现并部署。

## 在此之前保持

- 座椅路线一键启动保持停用；软件不发送 FSM 4。
- 实体遥控器 L2+B 仍是现场卸力手段，但“它进入 FSM 1”目前只是推断。
