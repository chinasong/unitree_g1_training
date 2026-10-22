"""Review two exact large-library scans and the matching master ELF offline.

No library loading, vendor execution, emulation, RPCs or motor-value inference.
Symbol and string absence is a bounded negative result, never proof of absence.
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
INVENTORY_SHA256 = 'd5c1b6b3887a96662b8695486ac13e31db7ed274b0895fc0230858fdf8370845'
FILES = {
    '/usr/local/lib/libcryptopp.a': (158103474, '40c71d267d6fb97daab8a3dc7bac76f533f9ffc8f03cfd1a5bc173c89188cb84'),
    '/usr/local/lib/librealsense2.so.2.50.0': (161735304, 'ef030d74fca4d9dd7531ae1505c1d95b9eb1d376d4a97a876cf1134d051cf61e'),
}


def analyze(master_folder, inventory_path=ASSETS/'large_library_inventory_20261021.json'):
    from elftools.elf.elffile import ELFFile

    raw = inventory_path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != INVENTORY_SHA256:
        raise ValueError('large library inventory fingerprint mismatch')
    scan = json.loads(raw)
    assert scan['read_only'] and scan['max_file_bytes'] == 200*1024*1024
    assert all(scan[k] == 0 for k in ('command_writes', 'rpc_calls', 'vendor_program_executions', 'runtime_memory_reads'))
    assert {f['path'] for f in scan['files']} == set(FILES)
    files = []
    for f in scan['files']:
        assert (f['bytes'], f['sha256']) == FILES[f['path']]
        assert f['unchanged_during_read'] and f['sha256_after'] == f['sha256']
        for table in [f['symbols']] + ([f['dynamic_symbols']] if 'dynamic_symbols' in f else []):
            assert table['returncode'] == 0 and table['has_symbol_output'] and not table['stderr_nonempty']
            assert not table['matches'] and not table['matches_truncated']
        assert f['printable_strings']['returncode'] == 0
        assert not any(f['printable_strings']['marker_line_counts'].values())
        assert not any(f['exact_byte_markers'].values()) and f['standalone_fmx_string_count'] == 0
        files.append(dict(f, implementation_match_confirmed=False, fmx_loader_found=False,
            classification='通用密码算法静态库；Blowfish/AES 等符号不证明 FMX 算法、密钥或加载顺序' if f['kind'] == 'archive'
                else 'RealSense 相机库；相机标定 API 不提供 G1 关节零位、编码器或电机标定'))
    master = verified_file(master_folder, 'master_service')
    config = verified_file(master_folder, 'master_service.json')
    for name, value in [('master_service', master), ('master_service.json', config)]:
        assert any(f['path'] == '/unitree/module/master_service/'+name
                   and f['sha256'] == hashlib.sha256(value).hexdigest() for f in scan['recheck'])
    stream = io.BytesIO(master)
    elf = ELFFile(stream)
    tables = [elf.get_section_by_name(n) for n in ('.symtab', '.dynsym')]
    symbols = [s.name for table in tables if table for s in table.iter_symbols() if s.name]
    needed = [t.needed for t in elf.get_section_by_name('.dynamic').iter_tags() if t.entry.d_tag == 'DT_NEEDED']
    # Positive control: exact installed-master Mixer names are visible to static inspection.
    entries = []
    for label, prefix in [('Mixer::Gen', '_ZN7unitree8security5Mixer3GenE'),
                          ('Mixer::LoadFile', '_ZN7unitree8security5Mixer8LoadFileE')]:
        locations = {int(s.entry.st_value) for table in tables if table for s in table.iter_symbols()
                     if s.name.startswith(prefix) and s.entry.st_shndx != 'SHN_UNDEF'}
        assert len(locations) == 1
        address = locations.pop()
        helper, target = stub_target(read_va(elf, stream, address, 32), address)
        section = file_span(elf, target, 4)
        assert section['sh_type'] == 'SHT_NOBITS'
        entries.append(dict(name=label, entry=hex(address), helper=hex(helper), target=hex(target),
                            section_type=section['sh_type'], file_backed=False))
    return dict(schema_version=1, current=False, offline_only=True,
        checked_at_utc=scan['received_at_utc'], started_at_utc=scan['started_at_utc'],
        completed_at_utc=scan['completed_at_utc'], read_only=True,
        payload_decoded=False, motor_configuration_present=None, motor_configuration_values_obtained=0,
        command_writes=0, rpc_calls=0, configuration_calls=0, calibration_calls=0,
        vendor_program_executions=0, runtime_memory_reads=0,
        scope='此前超过 64 MiB 上限的两份指定大库，单文件上限 200 MiB；全符号、动态导出、字符串与固定字节标记静态检查',
        files=files,
        master_linkage=dict(path='/unitree/module/master_service/master_service', sha256=hashlib.sha256(master).hexdigest(),
            needed=needed, cryptopp_or_realsense_direct_needed=any('cryptopp' in n.lower() or 'realsense' in n.lower() for n in needed),
            symbol_tables=[t.name for t in tables if t],
            named_cryptopp_symbols=sum('8CryptoPP' in s for s in symbols),
            named_realsense_symbols=sum(s.startswith(('rs2_', '_ZN3rs2', '_ZNK3rs2')) for s in symbols),
            positive_control_loader_entries=entries, static_or_runtime_linkage_excluded=False),
        robot_recheck=dict(boot_id=scan['boot_id'], processes=scan['processes'], files=scan['recheck']),
        source_references=[dict(url='https://raw.githubusercontent.com/weidai11/cryptopp/master/Readme.txt',
            role='库作者说明其为通用密码库；不用于确认 Orin 所装库版本或 FMX 格式'),
            dict(url='https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.50.0/include/librealsense2/h/rs_device.h',
            role='上游相机设备和 calibration table API 定义；未调用这些接口，也未证明实机库与标签源码完全相同')],
        conclusion='两份大库漏扫已补齐，未找到匹配当前 master_service 的完整 Mixer/Gen 实现或 FMX 加载依据；新增实机电机配置及标定 0 项',
        limitations=['符号和字符串未命中不能排除改名、隐藏、混淆或其他未扫描位置的实现',
            '符号行数含重复项、未定义项与静态库成员标题；动态符号是子集，不能相加当作唯一函数数',
            '通用算法符号存在不能确定 FMX 算法、密钥、容器布局或电机参数；没有继续猜测解码组合',
            'RealSense 的标定与 G1 关节标定归属不同；未打开相机设备、调用标定或写入 EEPROM',
            'DT_NEEDED 和命名符号只能反映磁盘信息；不排除静态裁剪、动态加载或运行时保护代码，未读取进程内存',
            '两份大库读取前后哈希和文件元数据一致；这是带时间的历史证据，不证明此刻加载或当前电机配置'],
        required_material='与当前 master_service SHA256 匹配的官方 Mixer/Gen 实现，或正式 FMX 格式、校验及只读导出接口；解出字段后仍须核实实机生效性和单位')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-folder', type=Path, required=True)
    parser.add_argument('--inventory', type=Path, default=ASSETS/'large_library_inventory_20261021.json')
    args = parser.parse_args()
    print(json.dumps(analyze(args.master_folder, args.inventory), ensure_ascii=False, indent=2))
