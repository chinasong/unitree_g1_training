# G1 两份大库与 FMX 加载关联核查

2026-10-21，北京时间（UTC+8）。**已补扫 libcryptopp.a 和 librealsense2.so.2.50.0；未取得匹配当前 master_service 的完整 Mixer/Gen 实现，新增实机电机配置、标定参数 0 项。253 个 actual_config 字段保持 null。** 基础档案 v10 新增带时间、哈希和范围限制的核查证据。

## 实机文件与扫描结果

Orin 于 2026-10-21 11:40:52–11:41:23 北京时间完成检查，Mac 接收于 11:41:23。两机时钟有轻微差异，接收时间不作为机器人事件发生时间。限定两份普通文件，单文件上限 200 MiB；不跟随链接、不加载库、不运行厂商程序。

| 文件 | 实际大小 | 扫描结果 |
| --- | ---: | --- |
| /usr/local/lib/libcryptopp.a | 158,103,474 B（150.78 MiB） | ar 共 168 个成员；nm 全符号输出 28,215 行；目标符号及标记均未命中 |
| /usr/local/lib/librealsense2.so.2.50.0 | 161,735,304 B（154.24 MiB） | nm 全符号输出 125,567 行，动态符号 84,621 行；含 .symtab / .debug_info；目标符号及标记均未命中 |

两份文件均检查 demangle 后的符号、strings -a -n 4、原始字节标记。目标包括 unitree::security::、Mixer、UT_MIXER、ParseConfigContent、decrypt_and_parse、master_service、FMX。Gen 以所属 security 命名空间及 Mixer 类关联识别，不把普通库的 generic generator 函数认定为官方 Mixer::Gen。符号输出包含重复项、未定义项及 ar 成员标题，**不是唯一函数数**；动态符号是子集，不能与全符号数相加。

SHA256：

- libcryptopp.a：`40c71d267d6fb97daab8a3dc7bac76f533f9ffc8f03cfd1a5bc173c89188cb84`
- librealsense2.so.2.50.0：`ef030d74fca4d9dd7531ae1505c1d95b9eb1d376d4a97a876cf1134d051cf61e`

两份文件读取前后 SHA256、大小、mtime_ns 一致。RealSense SONAME 为 librealsense2.so.2.50，build ID 为 `903edfeb6a1a20fb3449ca11371778c7cc0ba044`。文件名和 SONAME 不证明其与上游标签源码可复现一致，也不证明机器人正在加载它。

## 为什么仍不能填入电机参数

libcryptopp.a 可见 Blowfish 相关符号行 31 条、AES 4 条、SHA256 571 条、CBC_Mode 11 条。这与[作者将 Crypto++ 定义为通用密码库](https://raw.githubusercontent.com/weidai11/cryptopp/master/Readme.txt)一致，但无法确定 FMX 使用何种算法、密钥生成、封装顺序或校验。该上游说明不用于认定安装库版本。没有继续用通用算法组合猜测解密。

RealSense 的 rs2_ 相关输出反映相机 SDK。[v2.50.0 官方上游 rs_device.h](https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.50.0/include/librealsense2/h/rs_device.h) 中 calibration table 接口归属 RealSense 相机设备，不能替代 G1 电机编码器、零位或控制器标定。本次没有打开设备、调用任何标定接口或读写相机 EEPROM。

## 与当前 master_service 的对应核查

本次重新读取的 master_service、468 B FMX 配置和 master_service_pc4 模块清单指纹，均与此前证据一致。模块清单声明 1.0.0.2；仍不升级为整机或电机驱动固件版本。

当前 master_service SHA256：`380e34de9c2def2692c0bf1b73e969d3167c8270f2ed5f974bb01abd4e96f24e`。

离线解析该文件 .symtab / .dynsym 与 DT_NEEDED：没有 CryptoPP / RealSense 命名符号，也没有对这两个库的直接 NEEDED 依赖。**libcrypto.so.1.1 与 libcryptopp.a 是不同名称和对象，不能混用作为关联依据。** 这不能排除静态裁剪、改名、混淆、运行时加载或保护代码。

使用确实存在的 Mixer 符号作正向对照：

| 已知入口 | 入口地址 | 跳板后目标 | 磁盘可分析性 |
| --- | --- | --- | --- |
| Mixer::Gen | 0x95a68 | 0xa3fc78 | SHT_NOBITS，无文件字节 |
| Mixer::LoadFile | 0x4beb80 | 0xa3ee2c | SHT_NOBITS，无文件字节 |

两者均经此前核定的 0x1e61e8 跳板。说明静态检查能识别已知入口，而完整受保护实现仍未取得。没有执行、仿真、调试、附加进程、读取进程内存或绕过保护。

## 基础档案与复现

`service_audit.large_library_review` 保存本次两个大库、当前 master 的关联检查、正向对照、限制和所需官方材料。此前 v9 的 fmx_loader_review 和 receipt 保持原样，其“两份大库未扫”只代表旧快照；新面板明确标注已补扫。

- `scripts/audit_g1_large_libraries.py`：仅读取指定库、固定实机文件和进程身份；binutils 静态处理，不执行库代码。
- `scripts/analyze_g1_large_libraries.py`：校验固定快照及本机 master 副本 SHA256，离线复现核查；只输出允许的结构和身份字段。
- `large_library_receipt_20261021.json`：绑定两个脱敏 JSON 和上述两个脚本；原始大库、私有配置、凭据不进入发布包。

离线复现：

```bash
.venv/bin/python -m scripts.analyze_g1_large_libraries --master-folder .g1-deploy/research/opaque-20261021
.venv/bin/python scripts/build_g1_motor_knowledge.py
```

页面入口：“身体总览 → Orin 官方服务核查 → 大库补扫”。没有新增机器人读取协议、参数写入或动作入口。

继续取得实参需要与当前 master_service 指纹对应的官方 Mixer/Gen 实现，或正式 FMX 格式、校验及只读导出说明。即使后来解析出 JSON，也须核实字段归属、单位、控制器版本和实机生效性，不能把文件内容直接当作当前电机实参。

## 验证与部署

- 相关 pytest：**39 passed**，包括新证据指纹、篡改拒绝、离线复现、未知字段保留以及既有 FMX、服务能力和基础档案测试。
- 前端 `npm run build` 通过；保留现有 3D 视图分包体积提示。
- Orin 自建只读控制台已部署发布 `7c88c315e2a5`；启动后重新核对 **414 个发布文件**以及两个大库的 SHA256。四个官方进程的 PID、启动时钟和参数均未变，boot ID 一致。
- 2026-10-21 **11:47:29 北京时间（UTC+8）**验收：23 个关节反馈在线，采样延迟 19 ms，无控制权持有人，导航 idle，本次运行接受动作指令 0 条。Mac 观察 worker 保持 paused，未重启。
- Mac 浏览器已确认基础档案 v10、“大库补扫 2 / 2”及只读锁定提示；页面截图（截图 / 照片未公开，仅本地存档）。相机当时仍处于 retrying / SDK_3102，未作为本轮成功项。

完整带时间验收保存在 [G1_LARGE_LIBRARY_VERIFICATION.json](G1_LARGE_LIBRARY_VERIFICATION.json)。全程不执行动作，不修改官方服务或 Quantumult X。
