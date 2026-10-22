"""Review exact installed module metadata and offline loader copies, never run them.

Only identities, file fingerprints and bounded search results leave this module.
Install/remove commands, package contents and any configuration plaintext stay out.
No network, vendor execution, emulation, runtime memory reads or decoding guesses.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path

from scripts.analyze_g1_fmx import file_span, stub_target
from scripts.analyze_g1_opaque_configs import read_va, verified_file


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT/'g1_console/assets/service_audit_sources'
INVENTORY_SHA256 = '40e9e345f5d6283055cf20580b51e7cba93bc772533f83f57092d07a7dbbe188'
SEARCH_SHA256 = '0b2029b307bc0d5204e59263f770584a447dbbeae599701554355fb7f8a1e05c'
RECHECK_SHA256 = '7699587c3e8f397b3a81a934a530c0f574895c4b3ab093e6f8ac87688ae84fbf'
FILES = {
    'mscli': (5951720, '284aa30350e6afc988285e2efe62e36dc6209006ae14bc6b8d0575cdbbccf8e5'),
    'ota_pipe_cli': (6078920, 'bbaff3c92449f46d4e5a2c1cef6f990eaadaf6e89139967cb5f8f5f59a275c8d'),
    'ota_pipe_service': (9213664, 'fcca57b1f2c67af0123235cbbdd17f8f5a444a64d105a928b683cd5074be60e7'),
    'unitree_patch_pc4.upk': (21509748, 'fd20c663d203e526ad13eaff5c2fec780961e69a05d5911e9afafd3544dbc8fe'),
    'video_hub_pc4.upk': (75892, 'ac276e9f575aa2a3b4070ca5a1a2e8103e835cac42e01abf4275fe28d86e61e1'),
    'master_module.private.json': (2194, 'ecfa15fd170078b6bf89cc02f2becf4672b8c6f3566607bdfbe2717424857771'),
    'patch_module.private.json': (2880, 'df22b0309ebae1b78380473a297eca6d21fd1311d725a1a1c529e7d5df0572a0'),
    'video_module.private.json': (1913, 'e59171ecf2b3a3ce4101cc934fbe7ee775583874a2938cdbd38b32e304f35844'),
    'version.private.json': (160, 'b5d64c6e23e1a701172010a90ee8b17c83107c6560e9abb459db246b5f85842c'),
}
OTA_ENTRIES = {'Package::Unpack': 0x5e3a0, 'Package::InitHeader': 0x227e00,
               'Package::UnpackFile': 0x556098}


def checked(path, sha256, size=None):
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != sha256 or size is not None and len(data) != size:
        raise ValueError(f'input fingerprint mismatch: {path.name}')
    return data


def analyze(folder, master_folder, inventory_path, search_path,
            recheck_path=ASSETS/'fmx_module_recheck_20261021.json'):
    from elftools.elf.elffile import ELFFile

    inventory = json.loads(checked(inventory_path, INVENTORY_SHA256))
    search = json.loads(checked(search_path, SEARCH_SHA256))
    recheck = json.loads(checked(recheck_path, RECHECK_SHA256))
    data = {name: checked(folder/name, sha, size) for name, (size, sha) in FILES.items()}
    master_binary = verified_file(master_folder, 'master_service')
    master_config = verified_file(master_folder, 'master_service.json')
    assert inventory['read_only'] and not inventory['mapped_paths']
    assert all(inventory[k] == 0 for k in ('vendor_program_executions', 'rpc_calls', 'command_writes'))
    modules = []
    for name in ('master', 'patch', 'video'):
        filename = name+'_module.private.json'
        raw = json.loads(data[filename])
        # Explicit allowlist: never include executable installation/removal commands.
        modules.append(dict(name=raw['Name'], declared_version=raw['Version'],
            declared_commit=raw['Commit'], declared_build_time=raw['Time'],
            file_list_count=len(raw['FileList']), manifest_sha256=FILES[filename][1],
            manifest_path='/unitree/robot/pkg/module/'+raw['Name']+'/module.json',
            master_service_listed=any(p.endswith('/master_service') for p in raw['FileList']),
            source_commit_reproduced=False, motor_firmware_version_verified=False))
    versions = json.loads(data['version.private.json'])
    assert all(versions['Module'][m['name']] == m['declared_version'] for m in modules)
    assert recheck['read_only'] and recheck['command_writes'] == recheck['rpc_calls'] == 0
    for m in modules:
        assert any(f['path'] == m['manifest_path'] and f.get('sha256') == m['manifest_sha256']
                   for f in recheck['files'])
    master_list = json.loads(data['master_module.private.json'])['FileList']
    files = []
    for name, value in [('master_service', master_binary), ('master_service.json', master_config)]:
        path = '/unitree/module/master_service/'+name
        assert master_list[path] == len(value)
        assert any(x['path'] == path and x['sha256'] == hashlib.sha256(value).hexdigest()
                   for x in inventory['recheck'])
        files.append(dict(path=path, bytes=len(value), sha256=hashlib.sha256(value).hexdigest(),
                          manifest_size_matches=True, manifest_contains_hash=False))
    stream = io.BytesIO(data['ota_pipe_service'])
    elf = ELFFile(stream)
    stubs = []
    for name, address in OTA_ENTRIES.items():
        helper, target = stub_target(read_va(elf, stream, address, 32), address)
        section = file_span(elf, target, 4)
        stubs.append(dict(name=name, entry=hex(address), helper=hex(helper), target=hex(target),
                          section_type=section['sh_type'], file_backed=section['sh_type'] != 'SHT_NOBITS'))
    assert all(not s['file_backed'] for s in stubs)
    packages = []
    for filename, module, cached in [('unitree_patch_pc4.upk', 'unitree_patch_pc4', '1.0.0.2'),
                                      ('video_hub_pc4.upk', 'video_hub_pc4', '1.0.1.1')]:
        value = data[filename]
        assert value[:8] == bytes.fromhex('5554504b00010000')
        # Cached version comes from the observed path, not from a decoded header.
        path = next(p['path'] for p in inventory['packages']
                    if '/'+cached+'/' in p['path'] and module in p['path'])
        packages.append(dict(name=filename, path=path, bytes=len(value), sha256=FILES[filename][1],
            magic_hex=value[:8].hex(), cached_path_version=cached,
            installed_manifest_version=versions['Module'][module],
            path_version_matches_installed=cached == versions['Module'][module],
            payload_decoded=False, fmx_algorithm_equivalence_verified=False))
    bins = inventory['binaries']
    relevant = [b for b in bins if any('Mixer' in s or 'unitree::security::' in s for s in b['matches'])]
    return dict(schema_version=1, current=False, offline_only=True,
        checked_at_utc=recheck['received_at_utc'], payload_decoded=False,
        motor_configuration_present=None, motor_configuration_values_obtained=0,
        command_writes=0, rpc_calls=0, configuration_calls=0, calibration_calls=0,
        vendor_program_executions=0, motor_firmware_version=None,
        module_identity=dict(authority='本机 module.json 与 version.json 的一致声明；不是整机或电机固件版本',
            modules=modules, files=files, file_list_proves_content_hash=False,
            all_manifest_files_present_verified=False, manifest_file_recheck=recheck,
            installed_source_implementation_found=False),
        local_search=dict(roots=inventory['roots'], max_depth=inventory['max_depth'],
            max_binary_bytes=inventory['max_binary_bytes'], files_seen=inventory['files_seen'],
            unique_elfs=sum(b['kind'] == 'ELF' for b in bins),
            static_archives=sum(b['kind'] == 'archive' and not b['path'].endswith('.deb') for b in bins),
            other_ar_containers=sum(b['path'].endswith('.deb') for b in bins),
            without_defined_symbols=sum(not b['has_symbols'] for b in bins),
            duplicate_binaries=inventory['duplicate_binaries'], mixer_symbol_matches=relevant,
            additional_fmx_files=inventory['fmx_files'], errors=inventory['errors'],
            source_matches=inventory['sources'], mapped_paths_obtained=False,
            scope='此前 7 个服务 ELF 之外扩展的有界目录检查；nm 已定义符号检索，未证明无符号实现不存在'),
        ota_package_entries=stubs, cached_packages=packages, official_code_search=search,
        official_issue_lead=dict(url='https://github.com/unitreerobotics/xr_teleoperate/issues/299',
            role='官方仓库 issue 的服务管理路径线索，不是 FMX 规范或电机配置读取协议',
            commands_executed=False),
        robot_recheck=dict(boot_id=inventory['boot_id'], processes=inventory['processes'], files=inventory['recheck']),
        conclusion='确认 master_service 模块清单版本 1.0.0.2；未找到与当前二进制匹配的完整 Mixer/Gen 或正式 FMX 说明，载荷及电机参数仍未知',
        limitations=['模块清单只给出文件大小，无文件哈希或签名；声明提交没有已取得的源码与可复现构建相互验证',
            '补丁清单指向 /unitree/var/data/ota_update/ota_pipe/ota_pipe_service，但该路径不存在；清单不能证明文件均已安装',
            '符号检索不能识别被剥离、改名、混淆或动态加载的实现；两份大库超过 64 MiB 上限未扫描',
            '当前账号不能读取 master_service 的 /proc/maps；未提权或读取进程内存，DT_NEEDED 不能替代运行时模块清单',
            'UTPK 只确认标记；没有解包、安装或执行。不能把 OTA 包算法当作 FMX 算法，也不能把缓存补丁当作当前版本',
            'GitHub 结果限官方组织的索引默认分支，不覆盖历史、私有代码、二进制或未索引内容；未取得逐电机配置或标定'],
        required_material='当前 master_service 文件指纹对应的官方 Mixer/Gen 源码或 FMX 格式、校验及只读导出说明；即使解出字段也须另核实实机适用性')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--master-folder', type=Path, required=True)
    parser.add_argument('--inventory', type=Path, default=ASSETS/'fmx_loader_inventory_20261021.json')
    parser.add_argument('--search', type=Path, default=ASSETS/'fmx_official_code_search_20261021.json')
    parser.add_argument('--recheck', type=Path, default=ASSETS/'fmx_module_recheck_20261021.json')
    args = parser.parse_args()
    print(json.dumps(analyze(args.folder, args.master_folder, args.inventory, args.search, args.recheck), ensure_ascii=False, indent=2))
