"""Offline analysis of the exact Orin files acquired on 2026-10-21.

Never executes a vendor program, opens a socket or writes a decrypted config.
Optional research dependencies: pyelftools and pycryptodome (Mac venv only).
The interface addresses are local private input; keys and plaintext stay in memory.
Only an allowlisted inventory may leave this module. This is not a generic decoder.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re


FILES = {
    'master_service.json': (468, '31eae39936576504c40be767bd93cd34b7489154eca989e5e5c2781b05e3d977'),
    'config.bin': (14640, 'fd69673a007839f4a33ba1507fe043f6dbcc373bae703ecaa58eea54e61e9e6f'),
    'master_service': (9083232, '380e34de9c2def2692c0bf1b73e969d3167c8270f2ed5f974bb01abd4e96f24e'),
    'MegaFlowRuntimeDaemon': (21448952, '3670595470afb44f037e083200ca33bc8905dfe06a2650399875c3c6fa501473'),
}
PLAINTEXT_SHA256 = 'f58a6a0d5751879ca6d6c3d2c08b2d25f9d888af1ec7e52e85f5ad4a52c32aec'


def verified_file(folder, name):
    data = (folder / name).read_bytes()
    size, sha = FILES[name]
    if len(data) != size or hashlib.sha256(data).hexdigest() != sha:
        raise ValueError(f'file fingerprint mismatch: {name}')
    return data


def read_va(elf, stream, address, size):
    for segment in elf.iter_segments():
        if (segment['p_type'] == 'PT_LOAD' and segment['p_vaddr'] <= address
                and address + size <= segment['p_vaddr'] + segment['p_filesz']):
            stream.seek(segment['p_offset'] + address - segment['p_vaddr'])
            return stream.read(size)
    raise ValueError('address is not file-backed; runtime expansion is not performed')


def parse_padded_json(data):
    # This exact file is complete JSON + PKCS#7 padding, not a NUL-terminated fragment.
    if not data or len(data) % 16:
        raise ValueError('invalid block size')
    padding = data[-1]
    if not 1 <= padding <= 16 or data[-padding:] != bytes([padding]) * padding:
        raise ValueError('invalid padding')
    plaintext = data[:-padding]
    try:
        parsed = json.loads(plaintext)
    except (ValueError, UnicodeError):
        raise ValueError('not a complete JSON document') from None
    if not isinstance(parsed, dict):
        raise ValueError('unexpected JSON root')
    return parsed, plaintext


def schema(value):
    if isinstance(value, dict):
        return {k: schema(v) for k, v in value.items()}
    if isinstance(value, list):
        # Report distinct element schemas, not an assumption based on the first element.
        return list({json.dumps(schema(v), sort_keys=True): schema(v) for v in value}.values())
    return type(value).__name__


def safe_inventory(parsed):
    if set(parsed) != {'baseInfo', 'items', 'platform', 'uuid'}:
        raise ValueError('unexpected top-level schema; manual review required')
    items = parsed['items']
    categories = {'bins', 'deps', 'libs', 'plugins', 'weights'}
    if not isinstance(items, dict) or set(items) != categories:
        raise ValueError('unexpected component categories')
    if not all(isinstance(items[k], list) for k in categories):
        raise ValueError('unexpected component type')
    plugins = []
    for item in items['plugins']:
        # No URL, secret, UID, platform credentials or arbitrary nested values are copied.
        filename = item['filePath'].replace('\\', '/').rsplit('/', 1)[-1]
        if not re.fullmatch(r'[A-Za-z0-9_.-]+\.zip', filename):
            raise ValueError('unexpected plugin filename')
        plugins.append(dict(name=item['name'], version=item['version'], filename=filename,
                            installed_verified=False, loaded_verified=False, execution_allowed=False))
    motor_pattern = re.compile(r'motor|joint|calibrat|offset|gain|torque|current.?limit|^kp$|^kd$|零位|标定', re.I)
    keys = []
    def visit(value):
        if isinstance(value, dict):
            for key, child in value.items():
                keys.append(key)
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)
    visit(parsed)
    return dict(root_sections=sorted(parsed),
                schema_without_values=schema(parsed),
                category_counts={k: len(items[k]) for k in sorted(categories)},
                declared_plugins=plugins,
                declared_binaries=[dict(name=i['name'], version=i['version']) for i in items['bins']],
                motor_field_name_matches=sum(bool(motor_pattern.search(k)) for k in keys),
                content_classification='组件、插件及平台连接配置；不是逐电机参数表',
                authority='本机配置声明；不证明插件安装、加载、当前可用或动作安全',
                private_sections_omitted=True)


def analyze(folder, addresses):
    from elftools.elf.elffile import ELFFile
    from Crypto.Cipher import AES

    data = {name: verified_file(folder, name) for name in FILES}
    if data['master_service.json'][:4] != b'FMX\x01':
        raise ValueError('unexpected FMX magic')
    with (folder/'MegaFlowRuntimeDaemon').open('rb') as stream:
        elf = ELFFile(stream)
        # Addresses confirmed by disassembling this fingerprinted installed program.
        prefix = read_va(elf, stream, 0x573a30, 11)
        suffix = read_va(elf, stream, 0x573a40, 4)
        iv = read_va(elf, stream, 0x573a48, 16)
    successes = []
    for interface, mac in addresses.items():
        if not re.fullmatch(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}', mac):
            continue
        key = prefix + mac.upper().encode('ascii') + suffix
        try:
            parsed, plaintext = parse_padded_json(AES.new(key, AES.MODE_CBC, iv).decrypt(data['config.bin']))
        except ValueError:
            continue
        if hashlib.sha256(plaintext).hexdigest() != PLAINTEXT_SHA256:
            raise ValueError('unexpected plaintext fingerprint; manual review required')
        successes.append(dict(matching_interface=interface, plaintext_bytes=len(plaintext),
                              plaintext_sha256=PLAINTEXT_SHA256, **safe_inventory(parsed)))
    if len(successes) != 1:
        raise ValueError('expected exactly one verified decode')
    return dict(schema_version=1, current=False, offline_only=True,
                command_writes=0, rpc_calls=0, configuration_calls=0, calibration_calls=0,
                motor_configuration_values_obtained=0,
                files=[dict(name=n, bytes=v[0], sha256=v[1]) for n, v in FILES.items()],
                megaflow=dict(status='decoded_inventory_only', format='AES-256-CBC → 完整 JSON（本文件为 PKCS#7 填充）',
                    loader='decrypt_and_parse_config_with_all_macs', loader_address='0x4632c0',
                    aes_wrapper_address='0x45c6d0',
                    call_chain=['get_all_mac_addresses', 'read_file_into_vector',
                                'uppercase MAC + embedded key material', 'aes_decrypt', 'cJSON_Parse'],
                    aes_key_setup_call='0x45c74c', aes_cbc_decrypt_call='0x45c768',
                    json_parse_call='0x4635ac', **successes[0]),
                master=dict(status='container_identified_payload_unknown', format='FMX v1 容器标记；内部字段未核定',
                    magic_hex='464d5801', payload_decoded=False, motor_configuration_present=None,
                    load_file_stub='0x4beb80', load_stub='0x1eaa08', parse_stub='0x4dd490',
                    expansion_helper='0x1e61e8', expanded_target='0xa3ee2c',
                    target_section='SHT_NOBITS / .bss',
                    conclusion='加载入口跳向文件中没有指令字节的运行时区域；符号名不能证明内部算法或电机字段',
                    unconfirmed=['头部长度/种子语义', '完整加解码流程', '内部字段及单位', '与运动主机标定的对应关系']),
                limitations=['分析对象来自 Orin；未取得运动主机当前控制器配置或工厂标定',
                    'MegaFlow 全字段结构已检查，未见逐电机参数；不外推到其他文件或嵌套插件包',
                    'FMX 仅确认容器标记和受保护的加载入口；未启动、转储或修改官方进程',
                    '静态文件与日志属于历史证据，不能证明实时加载状态或动作可用性',
                    '私有地址、密钥材料、原始明文和平台凭据不进入基础模型、前端或发布包'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--private-addresses', type=Path, required=True)
    args = parser.parse_args()
    # stdout is sanitized evidence only. No write path for plaintext exists.
    print(json.dumps(analyze(args.folder, json.loads(args.private_addresses.read_text())), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
