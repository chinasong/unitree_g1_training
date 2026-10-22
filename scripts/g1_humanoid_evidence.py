"""Validate the two humanoid DDS paths without inferring firmware or readiness."""
import json
import math


def _metadata(participant):
    props = participant['properties']
    assert len({p['name'] for p in props}) == len(props)
    allowed = {'__Hostname', '__ProcessName', '__Pid', '__ProcessId', '__ExecName'}
    assert all('value' not in p or p['name'] in allowed and isinstance(p['value'], str)
               and len(p['value']) <= 256 for p in props)
    return {p['name']: p['value'] for p in props if 'value' in p}


def _publication(row, topic, type_name):
    assert len(row['matching_publications']) == 1, 'ambiguous publication'
    pub = row['matching_publications'][0]
    assert pub['topic'] == topic and pub['type'] == type_name
    assert pub['instance_handle'] == row['publication_handle'], 'unpaired publication handle'
    assert pub['key'].replace('-', '')[:24] == pub['participant_key'].replace('-', '')[:24]
    return pub


def humanoid_review(raw, files):
    assert raw['current'] is False and raw['device_motion_firmware'] is raw['exact_7301_predicate'] is None
    assert raw['command_writes'] == raw['lease_requests'] == 0
    assert raw['official_processes_unchanged'] is True
    assert raw['official_processes_before'] == raw['official_processes_after']
    assert all(r['owner'] is None and r['armed'] is False and r['navigation'] == 'idle'
               for r in (raw['before'], raw['after']))
    assert raw['passive_rtps']['errors'] == [] and not raw['passive_rtps']['payloads_retained']
    assert len(raw['queries']) == 2 and {q['api_id'] for q in raw['queries']} == {7001, 7003}
    queries, responder_keys = [], set()
    for query in raw['queries']:
        assert query['service'] == 'sport' and len(query['replies']) == 1
        reply = query['replies'][0]
        assert reply['api_id'] == query['api_id']
        pub = _publication(reply, 'rt/api/sport/response', 'unitree_api::msg::dds_::Response_')
        prefix = pub['participant_key'].replace('-', '')[:24]
        assert any(s['guid_prefix'] == prefix and s['source_ip'] == '192.168.123.161'
                   and s['packets'] > 0 for s in raw['passive_rtps']['senders']), 'unpaired sender'
        assert reply['code'] == 0 or reply['raw'] is None, 'error payload is not a reading'
        responder_keys.add(pub['participant_key'])
        queries.append(dict(api_id=query['api_id'], code=reply['code'], raw=reply['raw'],
            elapsed_ms=reply['elapsed_ms'], sent_at_receipt_clock_s=query['sent_at_receipt_clock_s']))
    assert len(responder_keys) == 1
    responder_key = next(iter(responder_keys))
    matches = [p for p in raw['response_participants'] if p['key'] == responder_key]
    assert len(matches) == 1
    responder = _metadata(matches[0])
    by_api = {q['api_id']: q for q in queries}
    assert by_api[7001]['code'] == 0 and json.loads(by_api[7001]['raw']) == {'data': 0}
    assert by_api[7003]['code'] == 7301 and by_api[7003]['raw'] is None
    state = raw['hg_state']
    assert state['errors'] == [] and len(state['streams']) == 2
    assert {s['topic'] for s in state['streams']} == {'rt/sportmodestate', 'rt/lf/sportmodestate'}
    peers = {p['key']: p for p in raw['other_node161_participants']}
    channels, participants = [], {responder_key: responder}
    for stream in state['streams']:
        pub = _publication(stream, stream['topic'], 'unitree_hg::msg::dds_::SportModeState_')
        peer = peers[pub['participant_key']]
        assert peer['observed_rtps_source_ips'] == ['192.168.123.161']
        assert pub in peer['publications'], 'state endpoint absent from discovery'
        metadata = _metadata(peer)
        assert metadata.get('__ProcessName') == responder.get('__ProcessName') == 'humanoid'
        assert metadata.get('__Pid') == responder.get('__Pid') and metadata.get('__Pid')
        participants[peer['key']] = metadata
        assert type(stream['samples']) is int and stream['samples'] > 1
        assert 0 <= stream['first_receipt_s'] < by_api[7003]['sent_at_receipt_clock_s']
        assert stream['last_receipt_s'] > by_api[7003]['sent_at_receipt_clock_s']+by_api[7003]['elapsed_ms']/1000
        assert stream['last_receipt_s'] <= state['duration_s']
        values = stream['distinct_values']
        assert values and stream['first'] in values and stream['last'] in values
        for value in values:
            assert set(value) == {'fsm_id', 'fsm_mode', 'task_id', 'task_time'}
            assert all(type(value[k]) is int and 0 <= value[k] <= 0xffffffff for k in ('fsm_id', 'fsm_mode', 'task_id'))
            assert type(value['task_time']) in (int, float) and math.isfinite(value['task_time'])
        # Report this captured state, never extrapolate it to a future pose.
        assert all(v['fsm_id'] == 0 for v in values)
        channels.append(dict(topic=stream['topic'], participant_key=peer['key'], samples=stream['samples'],
            first_receipt_s=stream['first_receipt_s'], last_receipt_s=stream['last_receipt_s'],
            values=values, source_ip='192.168.123.161'))
    assert len(participants) == 2, 'do not conflate the RPC and state participants'
    assert files['candidates'] == [] and files['errors'] == [] and files['current'] is False
    assert files['boot_id'] == raw['boot_id']
    assert files['rpc_calls'] == files['command_writes'] == files['lease_requests'] == files['vendor_program_executions'] == 0
    return dict(current=False, checked_at_utc=raw['checked_at_utc'], completed_at_utc=raw['completed_at_utc'],
        summary='同一采样窗口内，humanoid 的两条 HG 状态流收到数据，GetBalanceMode 仍返回 7301；状态广播存在不等于内部 LocoState 查询可用。',
        queries=queries, channels=channels,
        participants=[dict(key=k, self_reported=v, path='sport RPC 应答' if k == responder_key else 'HG 状态广播')
                      for k, v in participants.items()],
        executable_verified=False, device_motion_firmware=None, exact_7301_predicate=None,
        balance_ready=None, foot_load_n=None, cabinet_load_n=None, execution_allowed=False,
        findings=[
            dict(name='两条 humanoid 链路', evidence=f"RPC 与 HG 状态发布者的 GUID 不同；均自报 humanoid / PID {responder['__Pid']}，观测节点为 .161。",
                 meaning='保留两条来源，不把相同 PID 当作可执行文件或固件版本证明。'),
            dict(name='7301 的范围', evidence='HG 状态接收时间覆盖 7003 请求和 7301 回包；7001 返回 FSM 0。',
                 meaning='该窗口不是 3104 超时，也不是完全缺少 HG 广播；不能确定内部缓存、状态门控或版本兼容中的哪一项触发。'),
            dict(name='原始任务字段', evidence=json.dumps(channels[0]['values'], ensure_ascii=False),
                 meaning='FSM 0 对应公开 SDK ZeroTorque 编号；task_id=4 的本机语义未核定，不称作起身、站稳或正在执行动作。'),
            dict(name='Orin 有界文件检查', evidence=f"{len(files['roots'])} 个目录根、{files['files_seen']} 项文件、{files['text_files_read']} 项文本/日志尾部、{files['bytes_read']} 字节；无匹配数值版本候选。",
                 meaning=f"有 {len(files['skipped'])} 项跳过；只覆盖 Orin 可读范围，不代表已检查 .161 文件系统或所有缓存。"),
            dict(name='载荷与版本', evidence='官方 HG SportModeState 仅含 4 个模式/任务字段。',
                 meaning='没有运动固件、足底接触、双脚载荷或柜体支撑力字段；这些项目保持未知。')],
        read_routes=[
            dict(name='GetFsmId / GetFsmMode', route='sport 7001 / 7002', status='已接入', condition='固定官方 getter；模式编号不能独立证明姿态或平衡。'),
            dict(name='GetBalanceMode / Swing / StandHeight / Phase', route='sport 7003–7006', status='已接入 · 7301', condition='错误返回不作为数值；本轮只重查 7003，其他为此前有时间戳的证据。'),
            dict(name='ServiceList / CheckMode', route='robot_state 1003 / motion_switcher 1001', status='已接入', condition='服务开启与模式名均不证明平衡已就绪。'),
            dict(name='GetPkgVersion', route='robot_state 1006', status='此前返回 3203', condition='公开读取接口已覆盖；当前应答端不支持此调用，不推导运动固件版本。'),
            dict(name='GetMimicMotion', route='sport 7100 · 固定 fsm_id=550 / api_id=1', status='条件未满足 · 未调用', condition='官方 C++ 明确要求机器人处于稳定状态（例如站立）；靠柜坐地且承重未知时不调用，也不开放任意 FSM API。'),
            dict(name='basic_softlimit / basic_clearoip', route='DDS 主题发现', status='只读契约未知 · 未调用', condition='仅发现主题名，未找到官方 G1 getter 定义、参数及副作用说明，不猜测读写命令。')],
        limits=['这是带时间戳的历史采样，页面打开时不自动变成实时结论。',
                '状态接收时间不是传感器采样时间；帧数不证明内部控制健康或无丢包。',
                '尚无适用于本机固件和靠柜坐地起点的慢速起身路径、独立平衡或防摔范围。'])
