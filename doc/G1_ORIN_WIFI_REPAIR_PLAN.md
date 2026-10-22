# Orin 无线掉线：证据与未实施的单变量试验

2026-10-22，北京时间（UTC+8）。当前已恢复 `<ORIN_IP>` 的固定指纹 SSH 和只读控制台。用户已明确选择**“暂不修改，保持只读”**；本文件中的系统修改**未执行，不属于待执行授权**。方案仅作为研究和回滚设计保留。

## 11:33 源码核查补充

本机匹配版本源码表明，`Power Control` 报错来自 TX 发射功率寄存器操作，不能作为省电/LPS 致故障的证据；`IO offload fail:1` 是 HAL 通用失败码，不是 Linux errno EIO。底层 MAC status 未在前次日志中保留，具体触发条件仍未知。下文省电方案只是未实施的候选，优先级不能由该报错名称推出。完整日志重新计数及当前健康状态以[驱动只读诊断](G1_WIFI_DRIVER_READONLY_DIAGNOSIS_20261022.md)为准；下文数字保留先前采样记录，不与新统计相加。

## 已确认与未知（前次记录）

- 上次开机的 NetworkManager 曾在 10:10:57 自动连接当前 Wi-Fi 并取得 `.28`；10:38:37 记录断链，10:38:53 为 `supplicant-timeout`，10:40:10 重连结束于 `no-secrets`。期间有 8 次 `ASSOC-REJECT status_code=1`。日志也明确记录保存的认证信息存在，不能把 `no-secrets` 单独认定为密码丢失或密码错误。
- 同一次开机的内核日志中，网卡驱动从开机约 179 秒开始出现 `halbb_fw_set_reg IO offload fail`；断链时出现 RF API 失败。读取的该次开机日志段累计有 143797 条 I/O offload 失败、37247 条 RF API 失败和 1243 条 Power Control API 失败。未匹配到指定 USB 设备的断开 / 重置 / error 行；零匹配不能排除所有硬件问题。
- 当前驱动为 Realtek `8852bu`，版本 `v1.19.3-45-g7583a804a.20230505`，内核 `5.10.104-tegra`。当前开机已在约 21.72 秒监听 SSH、31.10 秒完成 Wi-Fi 激活；截至 11:17 的内核日志样本没有上述三类失败。
- 当前 Wi-Fi profile 的 `802-11-wireless.powersave=0`，继承系统 `wifi.powersave=3`，实测 `iw` 返回 `Power save: on`。USB 网卡的 runtime power control 为 `on`、状态 `active`；不能把 Wi-Fi 省电和 USB autosuspend 混为一项。
- 故障层已定位到无线链路 / 网卡驱动；固件 I/O 报错的具体触发原因仍未知。省电、驱动与 AP 兼容性、硬件等分支尚未完成因果验证。一次重启恢复不等于永久修复。

证据见 [本次重启与网络核查](G1_REBOOT_NETWORK_AUDIT_20261022.json)。原始日志保留在被 Git 忽略的本机研究目录；项目证据不含 SSID、密码和认证日志正文。

## 拟执行：只关闭当前 Wi-Fi profile 的省电

目的：用一个可回滚变量检验省电是否参与掉线；这是**排障试验 / 临时缓解候选**，不是已证实的根治。

| 项目 | 当前值 | 拟改值 |
| --- | --- | --- |
| profile UUID | `2a326059-b7e2-480a-aaa0-de96f8fe4980` | 不变 |
| `802-11-wireless.powersave` | `0`，继承全局启用 | `2`，仅该 profile 禁用 |
| 当前接口运行时省电 | `on` | `off` |

先通过固定 SSH 指纹核验 Orin，再重新读取当前 active UUID 与上述当前值；不符则停止，不套用旧快照。下列两条为待批准命令：

```bash
sudo nmcli connection modify uuid 2a326059-b7e2-480a-aaa0-de96f8fe4980 802-11-wireless.powersave 2
sudo iw dev wlan0 set power_save off
```

不主动断开 / 重连 Wi-Fi，不重启 NetworkManager、SSH 或机器人，不更新 / 卸载网卡驱动，不修改 Quantumult X、其他 Wi-Fi profile、IP、DNS、密码或机器人模式。第一条持久保存 profile，第二条立即调整运行时设置；驱动若异常，仍可能导致 SSH 暂时失联，Mac 连接器继续使用已有发现与指纹核验逻辑。

[NetworkManager 官方参数定义](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/settings-802-11-wireless.html)说明 `0` 使用全局值、`2` 禁用、`3` 启用；它不证明本机此次故障由省电导致。

## 验证与回滚

批准后读取 profile 值与 `iw dev wlan0 get power_save` 确认生效，连续检查 SSH、关节 tick、相机新帧和上述内核错误增量，保持无控制者、未授权运动、无动作记录。观察窗口内未复发只能记为该窗口稳定。

下一次由用户执行机器人重启后，核对新 boot ID、自动联网、同一 SSH 指纹、只读服务恢复，以及省电仍为 `off`；需要至少两次实际重启通过，才能完成既定重启恢复验收。用户当前未授权由软件触发机器人重启。

回滚到本次已读取的原值：

```bash
sudo nmcli connection modify uuid 2a326059-b7e2-480a-aaa0-de96f8fe4980 802-11-wireless.powersave 0
sudo iw dev wlan0 set power_save on
```

如驱动 I/O 失败仍持续，下一步核对宇树针对该硬件 / 内核的官方驱动包；更新驱动另行提出精确版本、兼容性、离线恢复和回滚方案，不通过自动重启系统服务掩盖问题。

执行边界来自用户提供的 AGENTS.md：**“先确认：……系统网络配置……sudo”**。已完成读取、证据保存和回滚设计；用户选择不实施。11:22 的只读复核显示相同 boot ID、SSH active、23 关节与相机新帧持续更新，profile 仍为 default、运行时省电仍为 on。本次开机 1374 行内核日志未匹配到前次三类失败；这只证明采样窗口未复发，不能证明长期稳定。
