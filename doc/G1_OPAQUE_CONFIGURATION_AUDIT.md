# G1 两处非明文配置的只读分析

2026-10-21，北京时间（UTC+8）。设备为用户确认的 G1 EDU / 23 DOF / 无手 / Orin。

**一份成功解析，一份仅确认容器标记；新增可核定的实机电机配置为 0 项。** 已加入基础档案 v7 的 `service_audit.configuration_analysis`。控制台“身体总览 → Orin 官方服务核查”显示相同结果。23×11 = 253 个 `actual_config` 字段继续为 null。

## 文件与加载依据

通过固定 SSH 主机指纹的连接读取文件，在 Mac 离线分析，没有执行厂商二进制、启动脚本或解码后的配置。10:50:14 北京时间重新读取了四个原文件的 SHA256，与分析副本一致；四个官方进程的 PID、argv、start_ticks 和 boot ID 与先前核查一致。旧日志与新分析分别保留，不改写首次核查证据。

| Orin 原文件 | 大小 | 结论 |
|---|---:|---|
| `/unitree/module/master_service/master_service.json` | 468 B | 起始 `464d5801`，确认 FMX v1 标记；内部字段未知 |
| `/unitree/module/master_service/master_service` | 9,083,232 B | AArch64 ELF，保留符号；配置加载入口跳往运行时区域 |
| `/home/unitree/megaflow_sdk/ws/bin/config.bin` | 14,640 B | AES-256-CBC 加密的完整 JSON；本文件填充符合 PKCS#7 |
| `/home/unitree/megaflow_sdk/ws/bin/MegaFlowRuntimeDaemon` | 21,448,952 B | AArch64 ELF，加载函数、AES 调用与 JSON 解析代码可静态检查 |

完整指纹、程序地址、接收时间与进程复核见 `g1_console/assets/service_audit_sources/configuration_analysis_20261021.json`，快照与脚本指纹见同目录 `configuration_receipt_20261021.json`。

## MegaFlow：已确认格式，内容属于组件和平台清单

实际进程的 `-c` 参数指向上述 `config.bin`。本机 ELF 的 `decrypt_and_parse_config_with_all_macs`（0x4632c0）依次读取接口地址和文件，使用大写 MAC 与内置材料构造密钥，调用 `aes_decrypt`（0x45c6d0），再调用 `cJSON_Parse`（0x4635ac）。AES 包装函数调用 `AES_set_decrypt_key`（0x45c74c）与 CBC 解密（0x45c768），使用 256 位密钥与 16 字节 IV。

按这份程序的逻辑，仅对本机接口地址尝试离线解码，只有 eth0 对应成功；没有猜测密码或暴力搜索。结果是 **14,628 B 完整 JSON + 12 B 填充**，明文指纹为 `f58a6a0d5751879ca6d6c3d2c08b2d25f9d888af1ec7e52e85f5ad4a52c32aec`。解码、填充和完整 JSON 均核验通过，随后逐层检查字段结构。

| 内容 | 本机配置中的数量 / 声明 |
|---|---|
| 顶层 | baseInfo、items、platform、uuid |
| 组件 | bins 5、deps 85、libs 24、plugins 2、weights 0 |
| G1 EDU 通用动作控制插件 | 1.0.16.20251127，g1edu_motion.zip |
| G1 EDU 全身舞蹈插件 | 1.0.3.20251128，g1edu_dance.zip |

该 JSON 的结构是组件、插件与平台连接配置，未发现逐电机增益、限流、力矩、零位或标定字段。插件清单不证明对应文件已安装、进程已加载、当前固件支持或动作安全；不增加执行入口。组件声明版本也不冒充电机驱动固件版本。

平台凭据、插件 secret、设备标识、接口实际地址、密钥/IV 与原始明文均未放入基础模型、网页或发布包。离线复现脚本只输出允许公开的清单和没有值的字段结构；不输出解密明文。最初人工分析的私有副本仅在忽略目录中以 0600 权限保存。

## master_service：容器可识别，内部内容仍未核定

历史日志的配置解析成功用于定位文件，不能证明配置属于电机。`Mixer::LoadFile`（0x4beb80）、`Mixer::Load`（0x1eaa08）和 `ServiceBase::ParseConfigContent`（0x4dd490）的文件内入口均为短跳转桩，共用 helper 0x1e61e8。其中 LoadFile 的目标 0xa3ee2c 位于 `.bss` / SHT_NOBITS，磁盘文件中没有对应指令字节。符号表给出的函数长度不能直接当作磁盘上的完整函数体。

程序还包含 Mixer、Blowfish 符号，但仅凭名称不能确认 FMX 的完整加解码流程、头部含义或内部字段。未运行、附加调试、转储或修改官方进程，也未将公共研究中的其他设备参数套入本机。独立研究关于 FMX 的描述只作背景，不能替代本机证据：[The cybersecurity of a humanoid robot](https://cybersecurityrobotics.com/research/cybersecurity-of-a-humanoid-robot/)。

核对了[官方 SDK 固定版本静态库](https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/lib/aarch64/libunitree_sdk2.a)的 Git blob 与 SHA256，未找到可直接使用的 Mixer / Blowfish 符号；这个结论仅适用于此次检查的文件。

因此保持 `payload_decoded=false`、`motor_configuration_present=null`。要进一步静态核定，需要匹配该版本的可分析加载实现或官方格式说明；即使解码成功，也须逐字段确认是否为运动主机实际生效的配置及标定，而非服务启动设置。

## 重现与验证

Mac 项目 venv 的研究依赖为 pyelftools 和 pycryptodome，不加入 Orin 控制台运行依赖。文件副本和接口地址放在忽略目录，脚本对四个输入逐一核对固定大小与 SHA256，不适用于其他版本。

```bash
.venv/bin/python scripts/analyze_g1_opaque_configs.py \
  --folder .g1-deploy/research/opaque-20261021 \
  --private-addresses .g1-deploy/research/opaque-20261021/network-addresses.private.json
.venv/bin/python scripts/build_g1_motor_knowledge.py
.venv/bin/python -m pytest tests/test_g1_configuration_analysis.py tests/test_g1_foundation.py -q
```

测试覆盖输入指纹篡改、分析快照篡改、错误填充、非完整 JSON、私有字段不导出、实际材料复现，以及“不把插件清单或未知 FMX 内容升级为电机实参”。私有材料缺失时，实际文件复现测试明确跳过。发布与实时验证见 `G1_CONFIGURATION_ANALYSIS_VERIFICATION.json`。

离线分析脚本不发 RPC。控制台更新后仅恢复只读状态订阅及原有读取接口；本次未申请控制权、发布 LowCmd、执行动作、修改官方配置或标定、重启官方服务，也未使用 sudo。Quantumult X 未改动；Mac 观察仍暂停。

后续 FMX 静态分析、公开研究假设验证及基础档案 v8 的结果见 [FMX 后续核查](G1_FMX_ANALYSIS.md)。本文保留 v7 当时的核查记录。
