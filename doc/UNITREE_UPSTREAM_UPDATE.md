# 宇树官方代码核查与更新

核查日期：**2026-10-20（Asia/Shanghai，UTC+08:00）**。
通过 GitHub 官方 API 和下载后的 Git 历史交叉核对，以下版本为核查时各仓库默认分支的最新提交。

## 本次同步

| 官方仓库 | 项目历史版本 | 本次锁定版本 | 官方提交时间（北京时间） | 新增提交 |
| --- | --- | --- | --- | --- |
| [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | `18b9ef8e57e69b73c5e9b301b8481fce16a44b9b` | `814556d15970dd2ecf1c9984e845ca02ab07e206` | 2026-09-21 20:06:06 | 43 |
| [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | `757b05158058a2a8005810c2bb2e1e8667cf3f17` | `276801e46c5d433564f24658bac64f254b7d2d4b` | 2025-07-25 18:20:21 | 4 |

历史版本取自本项目 `47996c9` 的父提交。`47996c9` 在 2025-12-22
移除了 `externals/` 下的 Git 子模块记录，约定外部依赖单独安装；仅剩的
`.gitmodules` 文件无法让 `git submodule update` 恢复这些依赖。
因此本次保留 `externals/` 的忽略规则，下载官方源代码到原有路径，新增
`unitree-deps.lock.json` 和同步脚本记录、恢复精确提交，不把第三方源码复制进主仓库。

## 与本项目有关的变化

- Python SDK：G1 `LocoClient.Start()` 的 FSM ID 从 `200` 改为 `500`
  （2026-04-20）；新增 `GetFsmId()`（2026-06-17），以及
  `SetSpeedMode()`、`SwitchToUserCtrl()`、`SwitchToInternalCtrl()`（2026-07-13）。
  这些接口能否在真机使用仍需结合机器人固件验证。
- 项目使用的 G1 `AudioClient.TtsMaker(text, speaker_id)`、`SetVolume(volume)`
  以及 HG 消息定义在上述 SDK 对比范围内没有变化。
- RL Gym 的 4 个新提交增加 G1 的 C++ / LibTorch 实机部署示例与说明，
  没有更改本项目依赖的 G1 模型资源。
- `g1_douji_voice_chat.py` 现在优先加载本项目锁定的 SDK，避免始终使用
  `/home/unitree/unitree_sdk2_python` 中可能过期的版本；原安装路径仍可后备。

## 恢复与验证

在项目根目录执行：

```bash
python3 scripts/sync_unitree_deps.py
python3 scripts/sync_unitree_deps.py --check
```

同步脚本下载缺失仓库并检出锁定提交；会在更改前检查所有现有依赖，遇到本地修改、
非 Git 目录或来源不匹配时停止。`--check` 只核对本地提交、来源与工作区，不联网。
同步不会自动追踪远程最新分支；后续升级应先核对上游差异，再更新锁定文件。

安装 SDK 与 MuJoCo 后，可执行不连接机器人的离线检查：

```bash
CYCLONEDDS_HOME="$PWD/.venv/opt/cyclonedds" \
  .venv/bin/python scripts/check_unitree_compat.py
```

检查内容包括项目使用的 SDK 导入符号、现有音频/运动方法、语音和握手方法签名、
HG 消息序列化往返、CRC 对电机指令变化的检测，以及官方 G1 23/29 自由度模型、
网格加载和各 100 步无窗口仿真。

### 本机实际验证结果

本机 Apple Silicon / Python 3.11.15 已完成：

- 两个官方仓库的来源、精确提交与干净工作区校验通过。
- 12 个项目使用的 SDK 导入符号、8 个现有客户端方法及语音/握手方法签名通过。
- HG 消息序列化往返通过；修改电机指令后 CRC 从 `4262932383` 变为 `3916461062`。
- G1 23/29 自由度模型与网格加载、各 100 步无窗口仿真通过，状态保持有限数值。
- 同步脚本在临时 Git 仓库的 5 项行为测试通过：精确恢复与重复执行、只读缺失检查、
  本地修改保护、来源校验、多依赖修改前预检。

`RecurrentThread` 在 macOS 上不能导入：官方实现依赖 Linux 的 `timerfd_create`。
检查脚本对这一项验证类定义存在并明确输出 `SKIP runtime`；其运行行为需在 Linux 上验证。
本次没有验证强化学习训练、机器人固件兼容性、DDS 实机通信或真实关节运动。

### Apple Silicon 离线检查环境复现

以下只安装到本项目 `.venv/`，不更改系统 Python 或全局开发工具配置。
CycloneDDS 的 IDL 工具需一并构建，以满足 Python 绑定的安装目录检查。

```bash
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python \
  'cmake==4.4.4' 'ninja==1.13.2' 'numpy==2.4.6' \
  'mujoco==3.16.0' 'opencv-python==5.0.0.93'

export DEVELOPER_DIR=/Library/Developer/CommandLineTools
dds_work=$(mktemp -d)
git clone --depth 1 --branch 0.10.2 \
  https://github.com/eclipse-cyclonedds/cyclonedds.git "$dds_work/source"
.venv/bin/cmake -S "$dds_work/source" -B "$dds_work/build" -G Ninja \
  -DCMAKE_MAKE_PROGRAM="$PWD/.venv/bin/ninja" \
  -DCMAKE_INSTALL_PREFIX="$PWD/.venv/opt/cyclonedds" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_IDLC=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_DDSPERF=OFF -DENABLE_SECURITY=OFF
.venv/bin/cmake --build "$dds_work/build" --parallel 8
.venv/bin/cmake --install "$dds_work/build"
CYCLONEDDS_HOME="$PWD/.venv/opt/cyclonedds" \
  uv pip install --python .venv/bin/python \
    'cyclonedds==0.10.2' -e ./externals/unitree_sdk2_python
CYCLONEDDS_HOME="$PWD/.venv/opt/cyclonedds" \
  .venv/bin/python scripts/check_unitree_compat.py
```

本次构建的 CycloneDDS `0.10.2` 对应提交为
`9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd`；SDK 的包版本号仍为 `1.0.1`，
因此应以锁定文件中的 Git 提交识别本次上游版本。

## 平台与其他官方项目

- SDK 官方依赖精确要求 `cyclonedds==0.10.2`。本机 Apple Silicon / Python 3.11
  没有可用的该版本二进制包，需要按
  [SDK 官方安装说明](https://github.com/unitreerobotics/unitree_sdk2_python#installation)
  构建 CycloneDDS，再设置 `CYCLONEDDS_HOME` 安装 Python 绑定。
- RL Gym 的 Isaac Gym 训练环境要求另行配置 Ubuntu / NVIDIA GPU，见
  [官方安装指南](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/doc/setup_en.md)。
  Mac 上的模型加载与短时仿真不能代替强化学习训练验收。
- [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) 是另一个基于
  Isaac Lab 的官方训练框架，支持 G1 29 自由度；核查时最新提交为
  `4960b84732b0c2ec593dccbfe963fda1bcd7b1e3`（2025-11-19）。它需要单独的
  Isaac Lab 环境和模型资产，当前项目没有直接引用，本次没有替换现有 RL Gym 流程。
- [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) 核查时最新提交为
  `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`（2026-09-07）。它提供 SDK/DDS
  仿真桥接，当前项目使用的是 MuJoCo Python 模型回放，本次没有引入新的仿真服务。

本次更新只涉及本地依赖与接入，不包含机器人固件更新、实机动作执行或部署。
