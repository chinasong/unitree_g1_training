#!/usr/bin/env python3
"""Bind a bounded cache inventory to the already reviewed master fingerprint."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'g1_console/assets/service_audit_sources'


def build():
    raw = json.loads((BASE/'package_cache_inventory_20261021.json').read_text())
    previous = json.loads((BASE/'fmx_loader_inventory_20261021.json').read_text())
    assert raw['identity_before'] == raw['identity_after']
    assert {(f['path'], f['sha256']) for f in raw['identity_after']} == {(f['path'], f['sha256']) for f in previous['recheck']}
    assert all(f['unchanged_during_read'] for f in raw['identity_after'] + raw['binaries'] + raw['packages'])
    expected = {f['sha256'] for f in previous['binaries']}
    assert all(f['sha256'] in expected and not f['mixer_fmx_symbols'] for f in raw['binaries'])
    return dict(current=False, checked_at_utc=raw['received_at_utc'], read_only=True,
        motor_configuration_values_obtained=0, payload_decoded=False, motor_configuration_present=None,
        command_writes=0, rpc_calls=0, vendor_program_executions=0, configuration_calls=0, calibration_calls=0,
        conclusion='新增发现目录版本为 1.0.0.6 的补丁包；仍为未解析 UTPK。备份工具与此前文件哈希相同，未找到匹配的 Mixer/Gen 实现或 FMX 格式说明',
        files_scanned=len(raw['files']), binaries_scanned=len(raw['binaries']), roots=raw['roots'],
        master_identity=raw['identity_after'],
        packages=[dict(path=p['path'], bytes=p['bytes'], sha256=p['sha256'], format=p['format'],
                       payload_decoded=p['payload_decoded'], implementation_match_confirmed=False,
                       version_meaning='目录版本与已安装补丁清单 1.0.0.6 一致；未证明包内内容匹配 master_service' if '1.0.0.6.upk' in p['path'] else '路径/包版本不等于电机固件或标定版本') for p in raw['packages']],
        limitations=['仅核查所列目录，深度 16、最多 3000 文件、单文件 128 MiB；不覆盖整机所有存储',
                     'APT lock 和 partial 无读取权限；未提权。未发现宇树命名的 dpkg 包记录不能证明未安装宇树软件',
                     '标准 Debian 包是 chrony 时钟同步组件，只列目录；NVIDIA 引导/内核载荷不是已确认的 G1 电机配置',
                     '符号未命中不排除裁剪、加密或运行时加载；UTPK 与 FMX 不能仅凭头部认定算法相同'],
        required_material='与当前 master_service 哈希绑定的厂商 FMX 格式/可审计导出器，或明确只读且匹配本固件的电机配置与标定读取协议',
        errors=raw['errors'])


if __name__ == '__main__':
    (BASE/'package_cache_review_20261021.json').write_text(json.dumps(build(), ensure_ascii=False, indent=2)+'\n')
    def item(p):
        return dict(path=str(p.relative_to(ROOT)), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    receipt = dict(sources=[item(BASE/n) for n in ('package_cache_inventory_20261021.json','package_cache_review_20261021.json')],
                   scripts=[item(ROOT/'scripts'/n) for n in ('audit_g1_package_cache.py','analyze_g1_package_cache.py')])
    (BASE/'package_cache_receipt_20261021.json').write_text(json.dumps(receipt,indent=2)+'\n')
