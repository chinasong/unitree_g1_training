# G1 官方服务与能力参数核查

2026-10-21，北京时间（UTC+8）。设备：G1 EDU / 23 DOF / 无手 / Jetson Orin。

基础档案 v6 持久保存此次结果，不训练运动模型，不增加动作权限。“身体总览”展示服务核查，“官方能力”展示参数、源码、版本差异及历史回读。

## 本机服务核查

只读取固定服务的 `/proc`、systemd 定义、普通账号可读配置及有界日志。未执行读到的脚本，未提权、重启官方服务、写配置、标定或调用运动接口。审计脚本仅放入 `/tmp`；项目控制台发布另行记录。

| 已核实线索 | 证据与边界 |
|---|---|
| 4 个官方进程 | master_service、ota_pipe_service、videohub_pc4、MegaFlowRuntimeDaemon；保存 PID、父 PID、start_ticks、argv、cgroup |
| master_service 启动 | SysV 生成服务使用 `/etc/init.d/master_service start`，启动官方二进制，无额外参数；MainPID=0 不代表未运行 |
| master_service 配置 | 有界全文读取约 11.47 MB 历史日志，61 条匹配，包括解析 master_service.json 成功；配置为 468 字节非明文，不证明包含电机参数 |
| MegaFlow 配置 | 实际进程 `-c` 指向本机 `bin/config.bin`，14640 字节非明文；不解码、不推定电机标定 |
| 视频启动定义 | 指定 CYCLONEDDS_URI 为本机 cyclonedds.xml；DDS 设置不等于相机或电机标定 |
| motionInfo | journal 出现目录查询文字；不能推定公开协议、动作编号或参数 |
| 41 个配置记录 / 90 个文件条目 | 固定目录、有界读取；无命中不证明系统不存在其他资料 |

root 进程的 exe、cwd 和 fd 链接不可读，未提升权限。历史日志含不同年代时间；用 Mac 接收时间注明批次，历史 PID/日期不作为当前状态。配置中的命令是惰性数据。非明文文件仅记录指纹、大小和路径。

**新增核定的逐电机配置为 0 项。23×11 = 253 个 actual_config 字段保持 null。** 启动路径及解析成功不能替代生效增益、零位、驱动固件和出厂标定。

## 官方能力参数覆盖

固定 Python SDK `814556d15970dd2ecf1c9984e845ca02ab07e206`、C++ SDK `d1abf001220d3e5f59e00a0c4aaa595e00089a4a`。20 个来源文件记录提交、SHA256 和官方链接。C++ 文件与此前官方仓库全文件指纹核对。

| SDK 领域 | Python 业务方法 | C++ 业务方法 / 重载 |
|---|---:|---:|
| G1 Loco | 23 | 32 |
| G1 Arm | 2 | 4 |
| G1 Audio | 6 | 6 |
| 共享 Video 读取 | 1 | — |
| AGV 专用变体 | — | 2 |
| 总计 | 32 | 44 |

76 项包含跨语言重复及别名，不表示实机新增 76 项能力。覆盖固定版本上述客户端的全部公开业务方法；初始化、内部通用 `_fsm_api` 及厂商未公开的 App 私有接口不计入能力。AGV 两项单列为当前硬件未确认，不能套用速度/升降限值。LowState/MotorCmd 字段沿用原档案，项目不发布电机命令。

每项保存参数名称、类型、输入/输出方向、默认值、API ID/版本、载荷/别名语义与官方实现。无证据的单位、有效范围和设备支持保持未知。SDK 默认值、传输类型、项目限制、本机读数分开。8 项常量无公开封装，仅列定义。

- Arm ID 调用：Python 使用 data，C++ 使用 action_id；C++ 名称调用使用 action_name。不混用。
- C++ 静态 right kiss=12，Python=13；不覆盖固件返回。Python 蹲站=706、躺站=702；C++ Squat=2、StandUp=4，不合并语义。
- HighStand 的 4294967295 是哨兵，不是米；ShakeHand 默认依赖客户端历史。
- Python TTS 从 0 自加不增长；C++ 后置自增。PlayStop 固定返回 0，不能作为设备确认成功。
- GetMimicMotion 要求稳定状态；未核实前提，未调用复用 API 7100 的入口。示教名称调用和停止自定义动作也仅保存定义。

## 7 次只读回读

固定官方 getter，采用 C++ 的空字符串载荷，Client(service, False)，不申请 lease。每项保存接收时间与原始返回。

| 方法 | API | 返回码 | 本批次有效值 |
|---|---:|---:|---|
| GetFsmId | 7001 | 0 | data=0 |
| GetFsmMode | 7002 | 0 | data=0 |
| GetBalanceMode | 7003 | 7301 | 未取得 |
| GetSwingHeight | 7004 | 7301 | 未取得 |
| GetStandHeight | 7005 | 7301 | 未取得 |
| GetActionList | 7107 | 0 | 已保存完整原始目录，实时目录另行刷新 |
| GetVolume | 1005 | 0 | volume=41 |

7301 官方定义为 LocoState not available。失败项 payload 为 null，不转换为 0，不诊断电机故障，不切换模式重试。成功值也是离散历史数据，不作为永久当前值。GetPhase 已弃用，未查询。

## 重现与验证

- `g1_console/assets/service_audit_sources/`：脱敏原始证据、Mac 接收时间、脚本和快照 SHA256。
- `g1_console/assets/capability_sources/` 与锁定 Python SDK：官方源码。
- `scripts/build_g1_capability_knowledge.py`、`scripts/build_g1_motor_knowledge.py`：AST/XML/文本生成器，不导入官方控制代码。
- `tests/test_g1_service_capabilities.py`：验证实际 getter 发包清单、失败值不落成有效值、来源篡改拒绝、公开方法覆盖、语言差异与权限隔离。
- `doc/G1_SERVICE_CAPABILITY_VERIFICATION.json`：实际发布与运行证明。控制台只读，观察 worker 继续暂停。

逐电机实参仍需与本机序列号/版本绑定的官方导出及字段定义。没有核实读取协议，不试发配置、标定和私有 RPC。

官方来源：[C++ Loco](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/g1/loco/g1_loco_client.hpp)、[C++ Arm](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/g1/arm/g1_arm_action_client.hpp)、[Python Loco](https://github.com/unitreerobotics/unitree_sdk2_python/blob/814556d15970dd2ecf1c9984e845ca02ab07e206/unitree_sdk2py/g1/loco/g1_loco_client.py)。
