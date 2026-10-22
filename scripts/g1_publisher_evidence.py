"""Project a captured DDS source audit without granting motion authority."""


def publisher_review(raw):
    assert raw['current'] is raw['motion_authority_verified'] is False
    assert raw['device_motion_firmware'] is raw['exact_7301_predicate'] is None
    assert raw['command_writes'] == raw['lease_requests'] == 0
    assert raw['boot_unchanged'] and raw['official_processes_unchanged']
    assert raw['official_processes_before'] == raw['official_processes_after']
    assert raw['before']['run_id'] == raw['after']['run_id']
    assert all(r['owner'] is None and not r['armed'] and r['navigation'] == 'idle'
               for r in (raw['before'], raw['after']))
    assert raw['errors'] == []
    assert raw['passive_rtps']['headers_only'] and not raw['passive_rtps']['raw_payloads_retained']

    def origin(row, topic, type_name):
        assert len(row['matching_publications']) == 1, 'ambiguous publication'
        pub = row['matching_publications'][0]
        assert pub['instance_handle'] == row['publication_handle'], 'unpaired publication handle'
        assert pub['topic'] == topic and pub['type'] == type_name
        prefix = pub['participant_key'].replace('-', '')[:24]
        assert pub['key'].replace('-', '')[:24] == prefix
        peers = [p for p in raw['participants'] if p['key'] == pub['participant_key']]
        assert len(peers) == 1, 'missing participant'
        peer = peers[0]
        ips = sorted({s['source_ip'] for s in raw['passive_rtps']['senders']
                      if s['guid_prefix'] == prefix and s['packets'] > 0})
        assert len(ips) == 1 and ips == peer['observed_rtps_source_ips'], 'unpaired sender'
        return dict(topic=topic, participant_key=peer['key'], source_ip=ips[0],
                    self_reported=peer['self_report'])

    advertisements = []
    for frame in raw['frames']:
        advertisements.append(dict(**origin(frame, 'rt/servicestate', 'std_msgs::msg::dds_::String_'),
            received_at_utc=frame['received_at_utc'], rows=frame['rows'], sha256=frame['sha256']))
    queries = []
    ids = set()
    allowed = {('sport', n) for n in (7001, 7002, 7003, 7005)} | {('motion_switcher', 1005)}
    for query in raw['queries']:
        assert (query['service'], query['api_id']) in allowed
        assert query['lease_id'] == 0 and query['parameter'] == ''
        assert query['request_id'] not in ids
        ids.add(query['request_id'])
        for reply in query['replies']:
            assert reply['code'] == 0 or reply['data'] is None, 'error payload is not a reading'
            queries.append(dict(**origin(reply, f"rt/api/{query['service']}/response",
                'unitree_api::msg::dds_::Response_'), service=query['service'],
                phase=query['phase'], api_id=query['api_id'], code=reply['code'],
                data=reply['data'], elapsed_ms=reply['elapsed_ms']))
    states = [dict(**origin(row, 'rt/lf/sportmodestate', 'unitree_hg::msg::dds_::SportModeState_'),
                   samples=row['samples'], values=row['distinct_values']) for row in raw['hg_state']]
    return dict(current=False, checked_at_utc=raw['checked_at_utc'],
        completed_at_utc=raw['completed_at_utc'], duration_s=raw['duration_s'], boot_id=raw['boot_id'],
        advertisements=advertisements, queries=queries, states=states,
        device_motion_firmware=None, exact_7301_predicate=None,
        motion_authority_verified=False, execution_allowed=False,
        summary='版本公告已配对到运动板的 robot_state_service；运动查询和 HG 状态流配对到 humanoid。公告来源与运动程序不同，不能据此确认运行构建。',
        readiness='公告前后 GetFsmId/GetFsmMode 均为 0，GetBalanceMode/GetStandHeight 均为 7301；GetSilent=false，状态流持续发布。这些条件仍不足以证明内部 LocoState 或动作控制已就绪。',
        limits=['进程名、PID 和主机名为 DDS 自报；源 IP 是同 GUID 前缀的 RTPS 报文来源，不是二进制签名。',
                '没有读取 humanoid 可执行文件版本、申请 SDK 运动租约或测试动作接受。',
                'task_id=4 的本机语义未核定，不能当作已在执行 StandUp 或已站稳。'])
