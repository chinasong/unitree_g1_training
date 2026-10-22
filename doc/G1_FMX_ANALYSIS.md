# G1 实机 FMX 后续只读核查

2026-10-21。**完整载荷仍未核定，新增实机电机配置及标定参数 0 项。** 基础档案 v8 保存核查证据，23×11 = 253 个 `actual_config` 字段继续为 null。未把推测、公开研究示例或 SDK 默认值填入实机字段。

## 实机文件与加载程序

| 实际材料 | SHA256 | 结果 |
| --- | --- | --- |
| `/unitree/module/master_service/master_service.json`，468 B | `31eae39936576504c40be767bd93cd34b7489154eca989e5e5c2781b05e3d977` | FMX v1 标记；内容是否含电机参数未知 |
| `/unitree/module/master_service/master_service`，9,083,232 B | `380e34de9c2def2692c0bf1b73e969d3167c8270f2ed5f974bb01abd4e96f24e` | ELF AArch64，含符号，但相关入口经过保护 |

偏移 8 的小端 u32 为 448，等于文件总长减 20。20 字节之后有 448 字节，满足 8 字节分组对齐；32 字节之后有 436 字节，不满足该对齐。这些只是可复现的结构事实，**不能据此确认头部长度、种子语义或算法**。未为了套用算法补齐或截断文件。

离线解析五个入口的固定 8 条指令跳板，均调用 `0x1e61e8`，最终目标落在 `.bss` / `SHT_NOBITS`，目标内容没有保存在 ELF 文件中：

| 符号 | 入口 | 目标 |
| --- | --- | --- |
| LoadFile | `0x4beb80` | `0xa3ee2c` |
| Load | `0x1eaa08` | `0xa3f4a8` |
| Gen | `0x95a68` | `0xa3fc78` |
| MixerC1 | `0x4c1e58` | `0xa3fc58` |
| MixerC2 | `0x4c7b70` | `0xa3fc58` |

这解释了仅有符号仍无法从磁盘直接读出完整实现；没有执行或仿真该程序，也没有附加、调试或转储正在运行的官方进程。

## 本机其他加载程序的范围

只读检查 `/unitree/module`、`/unitree/ota`、`/unitree/lib`、`/unitree/robot/pkg/module`、`/home/unitree/megaflow_sdk/ws/bin` 和 `ws/lib` 中实际存在的普通 ELF，最大深度 4，最多 96 个，每个不超过 64 MiB，不跟随符号链接。

实际检查 7 个 ELF：master_service、videohub_pc4、ota_pipe_service、FlexFlowInvoker、MegaFlowRuntimeCli、MegaFlowRuntimeDaemon、MegaFlowRuntimeSpawner。仅 master_service 出现 `unitree::security::Mixer::` 符号。未发现第二份可直接分析的 Mixer 实现；**搜索结果不覆盖全部磁盘、运动主机或无符号实现**。

检查结束时重新核对上述两个源文件 SHA256。四个官方进程的 PID、启动 tick、参数及系统 boot ID 与先前采集一致。所有结果带采集时间，是历史证据，不充当实时健康状态。

## 公开研究的离线假设测试

原研究者的 [Humanoids as attack vectors](https://cybersecurityrobotics.com/research/humanoids-as-attack-vectors/) 提供 Blowfish / LCG 相关线索，同时说明种子派生仍未完整逆向。它不是宇树格式规范，不能替代本机加载实现。

在精确指纹匹配的本机副本上做 16 个有界组合：候选载荷起点 12/20、两种分组字节序、LCG 输出移位 16/24、两种层次顺序。候选种子仅取一个文件字段，没有暴力搜索或枚举设备标识。得到完整 JSON 的组合为 **0**。候选密钥也没有以原字节、字反转或十六进制文本形式直接出现在这份磁盘程序中，但这不排除运行时派生。

无完整 JSON **不足以否定外层解码**，更不能确定内部没有电机参数。当前 `payload_decoded=false`、`motor_configuration_present=null`。将来即使得到可解析 JSON，也必须另行验证校验规则、实机控制器、字段单位和生效版本，才能进入 `actual_config`。

继续推进需要与本机程序指纹匹配的可分析 Mixer/Gen 实现，或宇树正式格式及读取说明。没有这些材料时不猜测私有读取 API。

## 产物与验证

- `scripts/analyze_g1_fmx.py`：离线分析器；固定源文件、盘点和候选材料指纹，不导出原始种子、密钥或解码字节。
- `g1_console/assets/service_audit_sources/fmx_analysis_20261021.json`：结构、入口、有界测试和结论。
- 同目录 `fmx_inventory_20261021.json`、`fmx_receipt_20261021.json`：本机盘点、接收时间、源文件及分析器 SHA256。
- 基础档案 `service_audit.fmx_analysis`；控制台“身体总览 → Orin 官方服务核查 → FMX 后续核查”。

运行 `tests/test_g1_fmx.py`、`test_g1_configuration_analysis.py`、`test_g1_foundation.py`、`test_g1_service_capabilities.py`，33 项通过；前端 TypeScript / Vite 构建通过，保留已有 3D chunk 大小提示。测试覆盖输入和快照篡改、跳板模式拒绝、私有材料复现、未知参数留空及无错误能力升级。部署和页面验证记录见 `G1_FMX_ANALYSIS_VERIFICATION.json`。

未发送动作、控制命令、配置或标定调用，未申请控制权，未修改或重启官方服务，未修改 Quantumult X。仅更新自建只读控制台。
