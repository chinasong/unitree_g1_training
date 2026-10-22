# G1 轮转日志、7301 与 3203 只读复核

核查日期：**2026-10-22，北京时间（UTC+8）**。日志扫描 11:42:26–11:42:46；驱动二进制检查 11:44:54；配对应答核查 11:45:08–11:45:33；软件界面点击核验 11:52:13–11:52:22。

## 结论与边界

- 找到原始错误码缺失的一层具体原因：本机 `rtw_hal_mac_add_cmd_ofld` 二进制把底层非零返回值统一转换成 `1`，且没有保留该处打印原始状态的代码。这个 `1` 不能当作底层 MAC 状态码或 Linux `EIO`。
- 全部现存目标日志都扫描到 EOF，没有找到原始 MAC 错误码、busy/polling 等细节。**无线故障的具体首因仍未知**，不能从现有记录还原。
- 官方只读请求均收到匹配应答：7301 表示 LocoState 不可用；3203 表示服务端 API 未实现。当前采样没有 3104，不能据此承诺以后不会超时。
- **实际运动固件版本、7301 内部就绪条件、双脚及柜体承重数值仍未知**。没有用 App 版本、协议版本、其他板卡版本或示例参数补造。
- 本轮机器人配置写入、动作写入、lease 请求均为 **0**。没有改 Wi-Fi、驱动、SSH、官方服务或 Quantumult X。

机器记录：[G1_ROTATED_LOG_RPC_AUDIT_20261022.json](G1_ROTATED_LOG_RPC_AUDIT_20261022.json)。所有记录均为带时间的历史证据，不是未来时刻的在线保证。

## 日志覆盖范围

本轮 boot ID：`aed898cb-acfd-44a2-8e9c-50f1f00269aa`。

只读枚举 `/var/log/kern.log*`、`syslog*`、`dmesg*`，实际存在以下 8 个普通文件；没有保留下来的 `kern.log.N` 或 `syslog.N`。压缩文件按解压内容扫描和计算 SHA-256。

| 文件 | 读取字节（解压后） | 行数 | 覆盖 |
| --- | ---: | ---: | --- |
| dmesg | 101,719 | 1,364 | EOF |
| dmesg.0 | 81,950 | 1,044 | EOF |
| dmesg.1.gz | 102,034 | 1,368 | EOF |
| dmesg.2.gz | 101,065 | 1,358 | EOF |
| dmesg.3.gz | 100,744 | 1,355 | EOF |
| dmesg.4.gz | 100,554 | 1,353 | EOF |
| kern.log | 57,462,298 | 579,983 | EOF、读取期间未变化 |
| syslog | 203,345,852 | 1,836,262 | EOF、读取期间有追加 |

没有触发字节上限、超长行跳过或读取错误。`syslog` 在读取中增长，因此其散列只标识实际读到的字节流，不表示原子文件快照。旧 dmesg 保存的是启动日志，不能补回对应启动之后未保留的运行期日志。

`kern.log` 和 `syslog` 各自包含 BB offload 失败 **148,756** 条、RF warning **38,941** 条、RF 写入失败 **8,480** 条、TX power control 失败 **1,243** 条、H2C class 错误 **1,189** 条。这些文件会重复记录事件，**没有把二者相加**。

8 个文件中的以下匹配计数全部为 0：包装函数原始 status、MAC offload busy、offload 详细诊断、platform tx、指定 Wi-Fi 固件异常格式、指定 USB 端口 fault。最新启动分段没有匹配到本次扫描的故障格式。零匹配只说明这些格式未被保留下来，不能排除没有打印的故障或硬件供电问题。

发现 `/etc/logrotate.d/rsyslog`，但 `/etc/logrotate.conf` 不存在，`logrotate.timer` 未启用/不存在。这不足以排除 cron 或其他轮转机制，也没有把它认定为无线故障原因。

## 已安装驱动的实证

模块：`/lib/modules/5.10.104-tegra/updates/dkms/8852bu.ko`，大小 **6,581,080** 字节，版本 `v1.19.3-45-g7583a804a.20230505`；本机匹配的 DKMS 源码位于 `/usr/src/rtl8852bu-1.19.3`。

模块 SHA-256：`4a5c04932df776166e9af87bf0528c1b858a74f134824a2566b0a6946f918f47`。

对应源码 `phl/hal_g6/hal_api_mac.c:10709–10727` 在下层返回错误时存在 `PHL_TRACE`；`phl/phl_debug.h` 和 Makefile 的 debug 条件说明该打印可以被编译去除。对实际模块反汇编进一步确认：

```text
fdcd0: cbz  x0, fdcf8     # MAC 对象缺失时也返回 1
fdce0: blr  x2            # 调用下层 add_cmd_ofld
fdce4: cmp  w0, #0
fdce8: cset w0, ne        # 所有非零状态合并为 1
fdcf4: ret
fdcf8: mov  w0, #1
```

该完整函数只有 `_mcount` 和下层回调调用，没有错误日志调用。其精确原始码格式串 `%s(): fault, status = %d.\n` 在模块中出现 **0** 次；作为阳性对照，`IO offload is busy` 格式串出现 **1** 次，polling timeout 格式串出现 **2** 次，`IO offload fail` 出现 **5** 次。因此结论限定为**这一包装层的原始码打印缺失**，没有声称全部 MAC 日志都被编译关闭。模块里另一处通用的 `fault, status` 字符串不是该函数的证据。

下层 `mac_ax/fwofld.c:1910–1952` 包含地址对齐、固件未就绪、offload 状态非 IDLE、缓冲分配/编码、发送和完成等待等分支。它们是源码候选路径，**尚未确定实机命中了哪一个**。

此前时间线中 BB/H2C/TX power 错误早于无线断链；断链后还出现 supplicant 超时、AP 拒绝及 `no-secrets`。不能把最后的 `no-secrets` 倒推成最初丢密码，也不能把 BB 的 TX power 控制当成 NetworkManager 的 Wi-Fi 省电。原时间线见 [此前驱动诊断](G1_WIFI_DRIVER_READONLY_DIAGNOSIS_20261022.md)。

## 7301、3203 与版本接口

本轮 10 次固定官方查询每次各有 1 个配对应答，**0 次 3104**。请求 ID、publication handle、DDS GUID 和被动 RTPS 源地址共同把这些应答关联到 `192.168.123.161`。sport 应答 participant 为 `0110652f-d08a-5632-cfa3-319f000001c1`，自报进程 `humanoid`、PID `3359`；自报元数据不等于已读取可执行文件或构建版本。

| 服务 / API | 返回码 | 有效值或含义 |
| --- | ---: | --- |
| sport / 1 | 0 | `1.0.0.0`，API 协议版本 |
| sport / 7001 | 0 | FSM `0` |
| sport / 7002 | 0 | 控制模式 `0` |
| sport / 7003–7006 | 7301 | 平衡模式、摆腿高度、站立高度、步态相位均未知 |
| motion_switcher / 1001 | 0 | `{form:"0",name:"ai"}` |
| robot_state / 1003 | 0 | 28 个服务；ai_sport 原始 status=0（开启） |
| robot_state / 1006 | 3203 | 无有效包版本 |

7301 的官方定义是 `UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE`。公开头文件没有给出服务端内部判定条件；与 FSM=0 同期出现不能证明 FSM=0 就是唯一触发条件。见 [G1 官方错误定义](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_error.hpp)。

3203 的官方 RPC 契约为 `RPC_ERR_SERVER_API_NOT_IMPL`，与 3204 参数错误、3205 lease 拒绝、3104 客户端等待超时分开。官方 Python 参考服务端在文本和二进制 handler 均未注册时返回 3203，早于 lease 检查。**这是公开参考实现的解释，不是实机 C++ 服务端源码证明**。见 [RPC 常量](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/rpc/internal.py)、[参考服务端](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/rpc/server.py)。

本轮版本请求遵照官方 `GetPkgVersion` 的 API 1006 和空 JSON 参数；实机仍明确返回 3203。没有为此获取 lease、试传未知参数、切换模式或探测未知 API。见 [官方版本客户端](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/b2/robot_state/robot_state_client.py)。

`.161:22` 和 `.161:1026` 本轮拒绝连接；只检查了这两个已知端口，不能据此声称该节点没有其他读取接口。Orin OTA 默认目标 `.164:1026` 是另一节点，不能替代运动节点的版本。

因此基础模型继续保留：`device_motion_firmware=null`、`exact_7301_predicate=null`。App `v1.5.6`、API `1.0.0.0`、PC4 模块版本都不升级为运动固件证据。23 自由度手册中的固件分支条件尚未匹配，当前靠柜坐地姿势仍没有获得起身/保持/平衡切换的适用性证明。

## 软件功能与实机界面核验

新增每张只读查询卡片的最近查询时间、返回码及未知状态。统一标注“可发起查询”，避免把按钮存在当作接口可用。历史成功回执不会覆盖更新的失败，也不会混用演示模式的结果。错误响应中的占位零不作为有效参数。

Mac 新界面：[http://127.0.0.1:18766/](http://127.0.0.1:18766/)。通过既有本机 SSH 转发读取实机后端，**没有部署或重启 Orin**；Orin 仍为 release `953b51275852`。此地址是当前项目进程提供的本地预览，不是已安装的开机服务。

界面实点结果（北京时间）：

- **11:52:13**，读取状态机成功，返回 `0`；卡片显示带时间的历史样本。
- **11:52:19**，读取平衡模式返回 **7301**；卡片显示 “LocoState 不可用 · 值未知”。
- **11:52:22**，读取官方包版本返回 **3203**；卡片显示 “服务端未实现 API · 值未知”。

当时 23/23 关节、相机持续更新；控制权为空、armed=false，动作入口保持锁定。只能据此确认只读查询链路和错误展示可用，**没有验证运动执行可用**。

查询卡片与 7301 回执（截图 / 照片未公开，仅本地存档）

包版本查询与 3203 回执（截图 / 照片未公开，仅本地存档）

基础模型新增此次应答来源与 RPC 契约说明，重新生成电机知识索引。只更新了 Mac 项目文件，没有把尚未确认的数值写入实机参数；新基础模型也尚未部署到 Orin。

验证：**51 项 Python 定向测试、14 项前端测试通过，TypeScript/Vite 构建通过**。现有 Starlette/httpx 弃用提示和 BodyView 分块大小提示不影响本轮结果。

## 剩余证据缺口

1. 原始 MAC 状态码：现存运行日志没有保留，实际包装层也没有打印分支；仅凭继续读取同一批日志无法恢复，需要能保留该状态的既有诊断记录或厂商说明。
2. 运动固件：需要 `humanoid` 对应的正式只读版本入口、该节点构建清单或官方可核验的包版本；当前应答不能提供。
3. LocoState 就绪：需要与本机构建相匹配的官方判定说明或只读状态证据，不能通过试起身、切模式或忽略 7301 来反推。

这三项仍明确标记未知；无线故障没有宣称根治，软件遥控器没有宣称全部动作已可执行。
