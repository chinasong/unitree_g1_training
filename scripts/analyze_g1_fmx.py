"""Offline FMX follow-up for one fingerprinted Orin file and its installed loader.

No vendor code is executed or emulated. Public-research candidates are hypotheses,
never verified plaintext. No raw key, seed, config bytes or decoded bytes are exported.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct

from scripts.analyze_g1_opaque_configs import verified_file


INVENTORY_SHA256 = '850e90b649b74e4b2c8b636f396fbdf6ca6af0f5e7e2b597148e7e479cf283f3'
RESEARCH_KEY_SHA256 = '030ed523c49326ce1790bd973a4905161334e788799d99ed3cd35aa74d3b454c'
RESEARCH_URL = 'https://cybersecurityrobotics.com/research/humanoids-as-attack-vectors/'
STUBS = {'LoadFile': 0x4beb80, 'Load': 0x1eaa08, 'Gen': 0x95a68,
         'MixerC1': 0x4c1e58, 'MixerC2': 0x4c7b70}


def file_span(elf, address, size):
    for section in elf.iter_sections():
        if section['sh_addr'] <= address and address + size <= section['sh_addr'] + section['sh_size']:
            return section
    raise ValueError('unmapped address')


def signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits-1)) else value


def stub_target(code, address):
    """Decode only this exact 8-instruction trampoline pattern, not an interpreter."""
    words = struct.unpack('<8I', code)
    if (words[:2] != (0xa9bf7bfd, 0xa9bf07e0)
            or words[3:5] != (0xa8c107e0, 0xa8c17bfd)
            or words[7] != 0xd61f0200 or words[2] & 0xfc000000 != 0x94000000
            or words[5] & 0x9f00001f != 0x90000010
            or words[6] & 0xffc003ff != 0x91000210):
        raise ValueError('unexpected loader trampoline')
    helper = address + 8 + signed(words[2] & 0x3ffffff, 26) * 4
    adrp, add = words[5:7]
    page = ((address+20) & ~0xfff) + (signed(((adrp >> 5) & 0x7ffff) << 2 | (adrp >> 29) & 3, 21) << 12)
    target = page + ((add >> 10) & 0xfff)
    return helper, target


def layout(data):
    if len(data) != 468 or data[:4] != b'FMX\x01':
        raise ValueError('unexpected FMX input')
    return dict(file_bytes=len(data), magic_hex=data[:4].hex(),
        offset8_u32_le=int.from_bytes(data[8:12], 'little'),
        bytes_after_20=len(data)-20, bytes_after_32=len(data)-32,
        block8_remainder_after_20=(len(data)-20) % 8,
        block8_remainder_after_32=(len(data)-32) % 8,
        header_size_verified=None, seed_semantics_verified=False,
        conclusion='偏移 8 的数值等于总长减 20；32 字节后不满足 8 字节分组对齐。仅为结构事实，不能确定头长或算法')


def hypotheses(data, binary, key):
    from Crypto.Cipher import Blowfish

    if hashlib.sha256(key).hexdigest() != RESEARCH_KEY_SHA256:
        raise ValueError('unexpected research candidate')
    def swap(value):
        return b''.join(value[i:i+4][::-1] for i in range(0, len(value), 4))
    def lcg(value, shift):
        seed = int.from_bytes(data[4:8], 'little')
        result = bytearray()
        for byte in value:
            seed = (seed * 1664525 + 1013904223) & 0xffffffff
            result.append(byte ^ ((seed >> shift) & 255))
        return bytes(result)
    results = []
    for start in (12, 20):
        for order in ('standard', 'u32_little'):
            def bf(value):
                if order == 'standard':
                    return Blowfish.new(key, Blowfish.MODE_ECB).decrypt(value)
                return swap(Blowfish.new(key, Blowfish.MODE_ECB).decrypt(swap(value)))
            for shift in (16, 24):
                for layers in ('blowfish_then_lcg', 'lcg_then_blowfish'):
                    payload = data[start:]
                    decoded = lcg(bf(payload), shift) if layers == 'blowfish_then_lcg' else bf(lcg(payload, shift))
                    try:
                        complete = isinstance(json.loads(decoded.rstrip(b'\0')), dict)
                    except (UnicodeError, ValueError):
                        complete = False
                    # Even a parse is a candidate requiring an independent validation gate.
                    results.append(dict(candidate_header_bytes=start, blowfish_byte_order=order,
                        lcg_output_shift=shift, layer_order=layers, complete_json=complete))
    return dict(source_url=RESEARCH_URL, authority='独立研究，非宇树格式规范；仅作离线假设',
        candidate_key_material_found_verbatim=key in binary,
        candidate_key_material_found_word_reversed=swap(key) in binary,
        candidate_key_material_found_ascii=key.hex().encode() in binary,
        results=results, hypotheses_tested=len(results), complete_json_count=sum(r['complete_json'] for r in results),
        applicable_to_installed_loader_verified=False, outer_layer_decode_verified=False,
        payload_decoded=False, motor_configuration_values_obtained=0,
        limitation='无完整 JSON 不足以否定外层解码；即使偶然能解析，也不能证明格式、校验、种子或实机生效参数已核定。未补齐、截断、暴力搜索或枚举设备标识')


def analyze(folder, inventory_path, key_path):
    from elftools.elf.elffile import ELFFile

    data = verified_file(folder, 'master_service.json')
    binary = verified_file(folder, 'master_service')
    inventory_bytes = inventory_path.read_bytes()
    if hashlib.sha256(inventory_bytes).hexdigest() != INVENTORY_SHA256:
        raise ValueError('inventory fingerprint mismatch')
    inventory = json.loads(inventory_bytes)
    assert not inventory['errors']
    remote = {item['path'].rsplit('/', 1)[-1]: item for item in inventory['recheck']}
    for name, content in [('master_service.json', data), ('master_service', binary)]:
        assert remote[name]['sha256'] == hashlib.sha256(content).hexdigest()
    stubs = []
    with (folder/'master_service').open('rb') as stream:
        elf = ELFFile(stream)
        for name, address in STUBS.items():
            section = file_span(elf, address, 32)
            assert section['sh_type'] != 'SHT_NOBITS'
            stream.seek(section['sh_offset'] + address - section['sh_addr'])
            helper, target = stub_target(stream.read(32), address)
            destination = file_span(elf, target, 4)
            assert helper == 0x1e61e8 and destination['sh_type'] == 'SHT_NOBITS'
            stubs.append(dict(name=name, entry=hex(address), shared_helper=hex(helper),
                target=hex(target), section=destination.name, section_type=destination['sh_type'], file_backed=False))
    research = hypotheses(data, binary, key_path.read_bytes())
    # A new parse must stop publication for manual validation; never silently promote it.
    if research['complete_json_count']:
        raise ValueError('candidate JSON requires independent manual validation')
    return dict(schema_version=1, current=False, offline_only=True,
        checked_at_utc=inventory['received_at_utc'],
        payload_decoded=False, motor_configuration_present=None, motor_configuration_values_obtained=0,
        command_writes=0, rpc_calls=0, configuration_calls=0, calibration_calls=0, vendor_program_executions=0,
        files=[dict(name=n, bytes=len(c), sha256=hashlib.sha256(c).hexdigest())
               for n, c in [('master_service.json', data), ('master_service', binary)]],
        structure=layout(data), loader_stubs=stubs, research_hypotheses=research,
        local_loader_search=dict(roots=inventory['roots'], maximum_depth=4, maximum_elf_files=96,
            maximum_elf_bytes=67108864, symlinks_followed=False,
            inventory_sha256=INVENTORY_SHA256, elf_count=len(inventory['elfs']),
            inspected=[dict(path=e['path'], bytes=e['bytes'], has_symbols=e['has_symbols'],
                mixer_symbol_count=sum('unitree::security::Mixer::' in s for s in e['nm_matches'])) for e in inventory['elfs']],
            alternative_mixer_implementation_found=False,
            scope='仅指定目录中实际存在且满足大小和深度限制的普通 ELF；不代表所有磁盘、运动主机或无符号实现'),
        robot_recheck=dict(boot_id=inventory['boot_id'], processes=inventory['processes'],
            files=inventory['recheck'], completed_at_utc=inventory['completed_at_utc']),
        conclusion='FMX 完整载荷仍未核定；已核验文件结构和 5 个加载/生成入口，并记录 7 个本机 ELF 的有界搜索。新增电机实参 0 项',
        required_material='与本机程序指纹匹配的可分析 Mixer/Gen 实现或宇树正式格式及读取说明；取得明文后仍须确认字段单位、控制器和生效版本')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--inventory', type=Path, required=True)
    parser.add_argument('--research-key', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.folder, args.inventory, args.research_key), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
