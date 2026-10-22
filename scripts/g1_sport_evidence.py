"""Derive historical service readiness from paired, fingerprinted observations.

Network attribution is deliberately separate from executable/board identity,
installed firmware, physical support and permission to move.
"""
import json


def node_inventory(raw):
    """Bind self-reported metadata to the responder, not an advertised bridge."""
    if 'response_participants' not in raw and 'known_read_paths' not in raw:
        return None  # Earlier observations did not collect these fields.
    sport_keys = {p['participant_key'] for q in raw['queries'] if q['service'] == 'sport'
                  for r in q['replies'] for p in r['matching_publications']}
    assert len(sport_keys) == 1
    matches = [p for p in raw['response_participants'] if p['key'] in sport_keys]
    assert len(matches) == 1, 'missing or ambiguous responder metadata'
    participant = matches[0]
    props = participant['properties']
    assert len({p['name'] for p in props}) == len(props), 'ambiguous property names'
    allowed = {'__Hostname', '__ProcessName', '__Pid', '__ProcessId', '__ExecName'}
    assert all('value' not in p or p['name'] in allowed and isinstance(p['value'], str)
               and len(p['value']) <= 256 for p in props), 'unapproved property value'
    values = {p['name']: p['value'] for p in props if 'value' in p}
    paths = raw['known_read_paths']
    assert paths['target'] == '192.168.123.161'
    assert all(p['observed_rtps_source_ips'] == [paths['target']]
               for q in raw['queries'] if q['service'] == 'sport'
               for r in q['replies'] for p in r['matching_publications'])
    assert paths['peer_ota_queries'] == paths['ota_mutations'] == paths['configuration_writes'] == 0
    assert {p['port'] for p in paths['known_ports']} == {22, 1026} and len(paths['known_ports']) == 2
    assert all(p['application_bytes_sent'] == 0 and p['result'] == 'ConnectionRefusedError'
               and p['errno'] == 111 for p in paths['known_ports'])
    assert paths['ota_hosts'] == {'default': 'tcp://192.168.123.164:1026'}
    return dict(participant_key=participant['key'], self_reported=values,
        property_names=[p['name'] for p in props],
        executable_verified=False, component_version=None, version_interface_verified=False,
        checked_at_utc=paths['checked_at_utc'], known_ports=paths['known_ports'],
        ota_hosts=paths['ota_hosts'], peer_ota_queries=0,
        limitation=paths['limitation'])


def service_readiness(raw):
    assert raw['current'] is False
    assert raw['command_writes'] == raw['lease_requests'] == 0
    assert raw['device_motion_firmware'] is None
    assert raw['official_processes_unchanged'] is True
    assert raw['official_processes_before'] == raw['official_processes_after']
    assert all(r['owner'] is None and r['armed'] is False for r in (raw['before'], raw['after']))
    capture = raw['passive_rtps']
    assert capture['payloads_retained'] is False and capture['errors'] == []
    senders = {(r['guid_prefix'], r['source_ip']) for r in capture['senders'] if r['packets'] > 0}
    rows = []
    for q in raw['queries']:
        # A single reply within each bounded window; no claim about all time.
        assert len(q['replies']) == 1
        reply = q['replies'][0]
        assert q['api_id'] == reply['api_id']
        assert len(reply['matching_publications']) == 1
        pub = reply['matching_publications'][0]
        assert pub['topic'] == f"rt/api/{q['service']}/response"
        assert pub['instance_handle'] == reply['publication_handle']
        assert pub['key'].replace('-', '')[:24] == pub['participant_key'].replace('-', '')[:24]
        prefix = pub['participant_key'].replace('-', '')[:24]
        ips = pub['observed_rtps_source_ips']
        assert ips and all((prefix, ip) in senders for ip in ips), 'unpaired RTPS source'
        assert reply['code'] == 0 or reply['raw'] is None
        rows.append(dict(service=q['service'], api_id=q['api_id'], code=reply['code'],
                         raw=reply['raw'], observed_sender_ips=ips))
    by_api = {(r['service'], r['api_id']): r for r in rows}
    expected = {('sport', n) for n in (1, 7001, 7002, 7003, 7004, 7005, 7006)} | {
        ('motion_switcher', 1001), ('robot_state', 1003), ('robot_state', 1006)}
    assert len(rows) == 10 and set(by_api) == expected
    assert all(by_api['sport', n]['code'] == 7301 for n in (7003, 7004, 7005, 7006))
    assert all(by_api['sport', n]['code'] == 0 and json.loads(by_api['sport', n]['raw']) == {'data': 0}
               for n in (7001, 7002))
    assert by_api['sport', 1]['code'] == 0 and by_api['sport', 1]['raw'] == '1.0.0.0'
    assert by_api['robot_state', 1006]['code'] == 3203
    assert by_api['motion_switcher', 1001]['code'] == 0
    assert json.loads(by_api['motion_switcher', 1001]['raw']) == {'form': '0', 'name': 'ai'}
    service_reply = by_api['robot_state', 1003]
    assert service_reply['code'] == 0
    services = json.loads(service_reply['raw'])
    assert isinstance(services, list) and len(services) <= 128
    assert all(set(r) == {'name', 'status', 'protect'} and isinstance(r['name'], str)
               and type(r['status']) is int and type(r['protect']) is int for r in services)
    assert len({r['name'] for r in services}) == len(services)
    assert {r['name']: r['status'] for r in services}.get('ai_sport') == 0
    inventory = node_inventory(raw)
    inventory_findings = []
    if inventory is not None:
        reported = inventory['self_reported']
        inventory_findings = [
            dict(name='应答参与者自报信息',
                evidence=f"进程 {reported.get('__ProcessName', '未知')}；主机 {reported.get('__Hostname', '未知')}；PID {reported.get('__Pid', '未知')}。已匹配到本轮 sport 应答参与者 GUID。",
                meaning='DDS 自报属性，不是可执行文件身份核验；这些属性没有组件版本，不作为 ai_sport 固件。'),
            dict(name='运动节点已知读取路径', evidence='192.168.123.161 的 SSH 22、OTA 已知端口 1026 均拒绝 TCP 连接；未发送应用数据。',
                meaning='仅为两处已知端口的历史快照；不能据此断言该节点没有其他版本读取接口。'),
            dict(name='Orin OTA 查询范围', evidence='本机 OTA 别名只有 default → tcp://192.168.123.164:1026；本轮未向 .161 发出 OTA 查询或修改配置。',
                meaning='默认入口仍是 Orin/PC4；未验证可跨节点读取，PC4 模块清单不能替代运动固件。')]
    return dict(current=False, checked_at_utc=raw['checked_at_utc'],
        completed_at_utc=raw['completed_at_utc'],
        summary='主运动服务开启与 LocoState 可读取是两个条件；本轮前者有证据，后者仍返回 7301。',
        observed_sender_ips=sorted({ip for r in rows for ip in r['observed_sender_ips']}),
        attribution='请求 ID → 应答 publication_handle → 查询进程发现的 DDS 发布者/参与者 GUID → 被动 RTPS 头部的源 IP。只标识观测到的网络发送节点，未核定物理控制板或 ai_sport 可执行程序。',
        services=services, queries=rows, node_inventory=inventory, device_motion_firmware=None,
        exact_7301_predicate=None, balance_ready=None, execution_allowed=False,
        findings=[
            dict(name='服务列表', evidence=f'robot_state / 1003 成功，返回 {len(services)} 项 name/status/protect；没有版本字段。', meaning='官方 G1 文档定义 status 0 为开启、1 为关闭；protect 原值保留，语义未核定。'),
            dict(name='运动状态', evidence='sport / 7001、7002 成功；7003–7006 均明确返回 7301。', meaning='与 FSM=0 同时观测到，仅是相关性；未证明 FSM=0 是 7301 的充分或必要触发条件。'),
            dict(name='版本读取', evidence='sport / 1 是协议版本；robot_state / 1006 返回 3203。', meaning='没有取得实际 ai_sport 运动固件；不从 Orin 组件、App 或协议版本推导。'),
            dict(name='触发条件缺口', evidence='官方公开 SDK 只给出 LocoState not available；未找到对应服务端判定实现。', meaning='状态发布未就绪、内部状态/版本兼容性等仍是待证假设；不通过切换模式或服务试错。')] + inventory_findings,
        limits=raw['limitations'])
