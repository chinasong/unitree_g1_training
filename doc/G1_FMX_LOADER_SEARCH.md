# G1 Mixer / Gen 与版本匹配核查

2026-10-21，北京时间（UTC+8）。**本次仍未读出 FMX 完整载荷，新增电机配置、标定参数 0 项，253 个 actual_config 字段保持 null。** 基础档案 v9 新增可追溯的模块版本与检索证据；没有将版本声明、示例或算法猜测当作电机实参。

## 当前 master_service 的身份

只读取得 `/unitree/robot/pkg/module/master_service_pc4/module.json` 与 `/unitree/robot/pkg/version/version.json`，两者均声明 master_service_pc4 为 **1.0.0.2**。模块清单声明提交 `dc964a3354534a80881d92daaf18d421fafc7916`，构建时间文本为 `2025-02-28 10:35:54`，未声明时区。

| 实际文件 | 大小 | 本次重新核对的 SHA256 |
| --- | ---: | --- |
| master_service | 9,083,232 B | `380e34de9c2def2692c0bf1b73e969d3167c8270f2ed5f974bb01abd4e96f24e` |
| master_service.json | 468 B | `31eae39936576504c40be767bd93cd34b7489154eca989e5e5c2781b05e3d977` |

清单只保存文件路径和大小，**没有文件哈希或签名**。大小匹配不能证明声明提交可重现当前二进制；未取得该提交对应源码。模块版本不是整机固件或逐电机驱动固件，`identity.firmware_version` 仍为 null。

## 本机实现搜索与更新包

在先前 7 个服务 ELF 之外，扩展到 sbin、bin、etc、ota、robot/pkg、var/data、NVIDIA OTA 目录、安装的 Unitree 头文件和库、MegaFlow 依赖目录。每目录最大深度 5，最多 5,000 个文件；本次遇到 786 个文件，扫描 62 个唯一 ELF、10 个静态库，另记录 1 个 Debian ar 包；50 份重复二进制按内容哈希去重。每个二进制上限 64 MiB，不跟随符号链接。

未发现额外 Mixer/security 已定义符号或 FMX 文件。SDK 安装头文件仅有 ParseConfigContent 声明，SDK 静态库和 OTA 服务有该符号，不能据此认定含 Mixer 实现。73 个 ELF/ar 容器中 18 个没有已定义符号输出（包括 Debian 包），所以“没有符号命中”不等于“没有实现”。libcryptopp.a 与 librealsense2.so 超过大小上限未扫描。当前账号读取 `/proc/1296/maps` 被拒绝，未提权；磁盘 DT_NEEDED 不能代替实际运行时库清单，也不能排除静态、混淆或动态加载代码。

| 本机材料 | 已确认 | 尚未确认 |
| --- | --- | --- |
| `/unitree/sbin/mscli`、ota_pipe_cli | 磁盘文件指纹、构建 ID；无已定义符号输出 | 未取得完整 Mixer/Gen；没有执行这些程序 |
| ota_pipe_service | Package::Unpack / InitHeader / UnpackFile 的固定跳板均通过 0x250ba8，目标落在 SHT_NOBITS | 目标指令在磁盘无内容；未执行、仿真或读取进程内存 |
| 缓存 unitree_patch_pc4 UPK | 路径版本 1.0.0.2，21,509,748 B，起始标记 UTPK | 已安装清单为 **1.0.0.6**；缓存不能当作当前对应包，载荷未解开 |
| 缓存 video_hub_pc4 UPK | 路径版本与清单同为 1.0.1.1，75,892 B，起始标记 UTPK | 未解开载荷，版本文本相等不能证明内容相同 |

UTPK 标记不是正式格式说明。OTA 解包入口也不能替代 FMX 加载实现，不能推定二者算法相同。没有解包、安装或运行更新包。另只读确认：补丁清单指向的 `/unitree/var/data/ota_update/ota_pipe/ota_pipe_service` 路径不存在；实际运行的 `/unitree/ota/pipe/ota_pipe_service` 已按独立指纹分析。这再次说明模块清单不能证明所有列出文件都已安装。

## 官方公开资料

[官方 xr_teleoperate issue #299](https://github.com/unitreerobotics/xr_teleoperate/issues/299) 中协作者提供 mscli 服务管理路径，社区回复提到 master_service 服务目录。本次只将其作为本机文件定位线索，**没有执行评论里的服务命令**；该 issue 不提供 FMX 格式或电机标定读取协议。

2026-10-21 11:16 北京时间，GitHub 代码 API 对官方组织索引的默认分支检索结果：

| 查询 | 命中 |
| --- | ---: |
| org:unitreerobotics Mixer | 0 |
| org:unitreerobotics FMX | 0 |
| org:unitreerobotics master_service | 0 |
| org:unitreerobotics UTPK | 1，digital_servo 的 STM32 HAL Legacy 头文件，命中 USB 宏 OUTPKTERRM，无 G1 格式依据 |

四项均返回 incomplete_results=false，保留查询、结果地址和采集时间。此结果不覆盖未索引资料、历史提交、私有代码或二进制，不能宣称宇树内部没有实现或读取工具。

## 进入基础档案的内容与验证

`service_audit.fmx_loader_review` 保存模块声明、实机文件指纹、目录范围、限制、更新包版本差异和官方检索记录。新增离线分析器 `scripts/analyze_g1_fmx_loader_search.py` 在核对固定输入 SHA256 后复现结果，只导出允许的身份和结构字段，安装/移除命令和包载荷不进入档案。`fmx_loader_receipt_20261021.json` 绑定分析器和四份脱敏证据的哈希。先前 FMX 证据与 receipt 保持原样。

控制台入口：“身体总览 → Orin 官方服务核查 → Mixer / Gen 扩展核查”。分析器不注册任何读取 API 或动作接口。验证覆盖输入篡改、模块版本不冒充电机固件、缓存补丁不冒充当前版本、未知实参留空和私有材料复现；测试、部署、页面结果见 `G1_FMX_LOADER_SEARCH_VERIFICATION.json`。

2026-10-21 11:25:51 北京时间（UTC+8），发布 `5ead91113d49` 已在 Orin 启动，411 个发布文件的 SHA256 逐一复核通过；实机基础档案与本地 v9 完全一致。相关测试 **36 项通过**，前端 TypeScript / Vite 构建通过（保留原有 3D 分包大小提示）。23 关节遥测在线，验证时延迟 34 ms，当前运行接受的动作命令为 0，控制权为空。四个官方进程的 PID、启动时钟、命令行和系统 boot ID 未变；模块清单文件哈希及缺失路径也重新核对。Mac 观察进程保持暂停。

Mac 原有控制台标签页已刷新，v9、新面板、三项模块声明和缓存版本差异均可见：页面截图（截图 / 照片未公开，仅本地存档）。摄像头仍返回既有 `SDK_3102`，此次验证不包括实时图像能力。

继续读取需要与上述 master_service 指纹对应的官方 Mixer/Gen 实现，或正式的 FMX 格式、校验和只读导出说明。即使后来解析出 JSON，仍须核对控制器归属、单位、版本和实机生效性，才能填入实际电机参数。

全程未执行动作、LowCmd、控制权申请、未知 RPC、配置或标定写入；未修改或重启官方服务，未操作 Quantumult X。发布仅更新自建只读控制台。
