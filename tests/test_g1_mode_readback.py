"""Fixed getter wire boundary, correlated replies, real spawned IPC and withdrawal."""
import json
import time
from types import SimpleNamespace as NS

import pytest

from g1_console.mode_readback import READS, QUERY_READS, ReadOnlyModeClient, IsolatedModeReadback


def client(monkeypatch, response, write_ok=True):
    from unitree_sdk2py.core.channel import ChannelFactory
    channels = {}
    class Channel:
        def SetReader(self): pass
        def SetWriter(self): pass
        def Write(self, request, timeout):
            self.sent = request
            return write_ok
        def Read(self, timeout):
            assert 0 < timeout <= .03
            service = self.name.split('/')[2]
            return response(channels[f'rt/api/{service}/request'].sent)
        def CloseReader(self): self.closed = True
        def CloseWriter(self): self.closed = True
    def create(self, name, typ):
        assert name in {f'rt/api/{s}/{d}' for s in ('sport','motion_switcher','robot_state','arm','voice') for d in ('request','response')}
        channels[name] = Channel()
        channels[name].name = name
        return channels[name]
    monkeypatch.setattr(ChannelFactory, 'CreateChannel', create)
    return ReadOnlyModeClient(timeout=.03), channels


DEFAULT_REPLY = object()


def reply(request, *, offset=0, api=None, code=0, raw=DEFAULT_REPLY):
    if raw is DEFAULT_REPLY:
        raw = {'form':'0','name':'ai'} if request.header.identity.api_id == 1001 else {'data':500}
    return NS(header=NS(identity=NS(id=request.header.identity.id+offset, api_id=api or request.header.identity.api_id),
                        status=NS(code=code)), data=json.dumps(raw))


def test_only_three_fixed_official_getters_no_lease_and_other_ids_are_ignored(monkeypatch):
    count = 0
    def response(r):
        nonlocal count
        count += 1
        return reply(r, offset=-1 if count % 2 else 0)
    c, channels = client(monkeypatch, response)
    for name, (service, api, parameter, _) in READS.items():
        assert c.read(name)[0] == 0
        r = channels[f'rt/api/{service}/request'].sent
        assert r.header.identity.api_id == api
        assert r.header.lease.id == r.header.policy.priority == 0 and not r.header.policy.noreply
        assert r.parameter == parameter and r.binary == []
    assert count == 6
    with pytest.raises(KeyError):
        c.read('StandUp')
    assert count == 6
    c.close(); c.close()
    assert all(ch.closed for ch in channels.values())
    assert c.read('fsm_id') == (3102,None)


def test_bounded_timeout_busy_traffic_send_failure_wrong_api_and_server_rejection(monkeypatch):
    for response, write_ok, code in [
        (lambda r: reply(r,offset=-1), True, 3104),
        (lambda r: None, True, 3104),
        (lambda r: reply(r), False, 3102),
        (lambda r: reply(r,api=7101), True, 3105),
        (lambda r: reply(r,code=7301), True, 7301),
    ]:
        c, _ = client(monkeypatch, response, write_ok)
        start = time.monotonic()
        assert c.read('fsm_id') == (code,None)
        assert time.monotonic()-start < .2
        c.close()


@pytest.mark.parametrize('name,raw', [
    ('fsm_id', {'data':True}), ('fsm_id', {'data':-1}), ('fsm_mode', {'data':2**32}),
    ('fsm_mode', {'data':'500'}), ('motion_mode', {'form':0,'name':'ai'}),
    ('motion_mode', {'form':'0'}), ('fsm_id', 'x'*4100),
])
def test_invalid_feedback_never_becomes_current(monkeypatch, name, raw):
    c, _ = client(monkeypatch, lambda r: reply(r,raw=raw))
    assert c.read(name) == ('INVALID_FEEDBACK',None)
    c.close()


def fake_modes(send, stop, interface):
    assert interface == 'eth0'
    stamp = time.monotonic()-.3
    for key in READS:
        send.send((key, dict(code=0, raw={'form':'0','name':'ai'} if key=='motion_mode' else {'data':500},
            received=stamp, received_at=1234., request_ms=5,
            diagnostics=dict(total=dict(attempts=2,successes=1,timeouts=1,errors=0),
                             attempts=2,successes=1,timeouts=1,errors=0,window_s=60))))
    stop.wait(5)
    send.close()


def failed_modes(send, stop, interface):
    send.close()


def test_spawned_modes_keep_receive_time_actual_rpc_counts_and_withdraw_on_ttl_error_stall():
    reader = IsolatedModeReadback(worker=fake_modes)
    try:
        deadline = time.monotonic()+5
        while not reader.snapshot()['motion_mode']['current'] and time.monotonic()<deadline:
            time.sleep(.02)
        row = reader.snapshot()['fsm_id']
        assert row['current'] and row['age_ms'] >= 300 and row['received_at']==1234.
        assert row['diagnostics']['attempts']==2 and row['samples']==1
        stamp = row['received_monotonic']
        assert not reader.snapshot(stamp+1.51)['fsm_id']['current']
        assert reader.snapshot(stamp+1.51)['motion_mode']['current']
        with reader.lock:
            reader.values['fsm_id'].update(code=3104)
        row = reader.snapshot()['fsm_id']
        assert not row['current'] and row['raw'] is None and row['code']==3104
        with reader.lock:
            reader.received -= 5
        assert all(not r['current'] and r['raw'] is None and r['code']=='MODE_WORKER_STALLED' for r in reader.snapshot().values())
    finally:
        reader.close(); reader.close()
    assert not reader.process.is_alive() and not reader.thread.is_alive()
    assert all(not r['current'] and r['raw'] is None for r in reader.snapshot().values())


def test_worker_exit_clears_feedback():
    reader = IsolatedModeReadback(worker=failed_modes)
    try:
        reader.thread.join(timeout=5)
        assert all(not r['current'] and r['raw'] is None and r['code']=='MODE_WORKER_EXITED' for r in reader.snapshot().values())
    finally:
        reader.close()


@pytest.mark.parametrize('name,api,value', [('get_balance',7003,0), ('get_swing_height',7004,.123),
                                         ('get_stand_height',7005,.456), ('get_phase',7006,[.1,.9])])
def test_on_demand_wire_contract_and_nonzero_code_discards_placeholder(monkeypatch, name, api, value):
    c, channels = client(monkeypatch, lambda r: reply(r,raw={'data':value}))
    assert c.read(name) == (0, {'data':value})
    r = channels['rt/api/sport/request'].sent
    assert r.header.identity.api_id == api and r.parameter == ''
    assert r.header.lease.id == 0 and r.binary == []
    c.close()
    c, _ = client(monkeypatch, lambda r: reply(r,code=7301,raw={'data':value}))
    assert c.read(name) == (7301, None)
    c.close()


@pytest.mark.parametrize('name,value', [('get_balance',True),('get_balance',-1),('get_balance',1.5),
    ('get_stand_height',None),('get_stand_height',True),('get_stand_height',float('nan')),
    ('get_swing_height','0.1'),('get_phase',[]),('get_phase',[1]*33),('get_phase',[True]),('get_phase',.5)])
def test_query_values_cannot_use_default_or_unbounded_feedback(monkeypatch, name, value):
    c, _ = client(monkeypatch, lambda r: reply(r,raw={'data':value}))
    assert c.read(name) == ('INVALID_FEEDBACK', None)
    c.close()


@pytest.mark.parametrize('name,api,value', [
    ('get_services',1003,[{'name':'sport','status':0,'protect':1}]),
    ('get_versions',1006,{'packageVersion':'test','moduleVersionMap':{'sport':'test'}}),
])
def test_metadata_queries_use_fixed_read_api_without_mode_or_service_write(monkeypatch, name, api, value):
    c, channels = client(monkeypatch, lambda r: reply(r,raw=value))
    assert c.read(name) == (0, {'data':value})
    request = channels['rt/api/robot_state/request'].sent
    assert request.header.identity.api_id == api and request.parameter == '{}'
    assert request.header.lease.id == 0 and request.binary == []
    c.close()
    c, _ = client(monkeypatch, lambda r: reply(r,code=3203,raw=value))
    assert c.read(name) == (3203,None)
    c.close()


@pytest.mark.parametrize('name,value', [
    ('get_services',{}),('get_services',[{'name':'sport','status':False,'protect':0}]),
    ('get_services',[{'name':'sport','status':0}]),
    ('get_versions',{'packageVersion':'test','moduleVersionMap':[]}),
    ('get_versions',{'packageVersion':1,'moduleVersionMap':{}}),
])
def test_invalid_metadata_cannot_become_version_or_service_evidence(monkeypatch, name, value):
    c, _ = client(monkeypatch, lambda r: reply(r,raw=value))
    assert c.read(name) == ('INVALID_FEEDBACK',None)
    c.close()


def test_silent_getter_uses_official_empty_parameter_no_lease_and_keeps_false(monkeypatch):
    c, channels = client(monkeypatch, lambda r: reply(r, raw={'silent': False}))
    assert c.read('get_silent') == (0, {'data': {'silent': False}})
    request = channels['rt/api/motion_switcher/request'].sent
    assert request.header.identity.api_id == 1005 and request.parameter == ''
    assert request.header.lease.id == request.header.policy.priority == 0 and request.binary == []
    c.close()
    c, _ = client(monkeypatch, lambda r: reply(r, code=7301, raw={'silent': False}))
    assert c.read('get_silent') == (7301, None)
    c.close()


@pytest.mark.parametrize('raw', [{'silent': 0}, {'silent': 'false'}, {'data': False}, {}, False])
def test_silent_requires_explicit_boolean_never_defaults(monkeypatch, raw):
    c, _ = client(monkeypatch, lambda r: reply(r, raw=raw))
    assert c.read('get_silent') == ('INVALID_FEEDBACK', None)
    c.close()


@pytest.mark.parametrize('name,service,version', [
    ('get_loco_api_version','sport','1.0.0.0'),
    ('get_motion_api_version','motion_switcher','1.0.0.1'),
    ('get_state_api_version','robot_state','1.0.0.2'),
])
def test_protocol_query_is_plain_string_fixed_api_and_never_firmware(monkeypatch, name, service, version):
    def response(request, code=0):
        r = reply(request, code=code)
        r.data = version  # Official API 1 is not a JSON document.
        return r
    c, channels = client(monkeypatch, response)
    assert c.read(name) == (0, {'data': {'service':service, 'api_version':version, 'firmware_version':None}})
    r = channels[f'rt/api/{service}/request'].sent
    assert r.header.identity.api_id == 1 and r.parameter == '{}'
    assert r.header.lease.id == r.header.policy.priority == 0 and not r.header.policy.noreply and r.binary == []
    c.close()
    c, _ = client(monkeypatch, lambda r: response(r, 3203))
    assert c.read(name) == (3203, None)
    c.close()


@pytest.mark.parametrize('value', ['"1.0.0.0"', '', '1', '1.0.0.0.0', '1.0\n',
    '<html>1.0</html>', '1.'+'0'*11, 'a.1', None, {'version':'1.0.0.0'}])
def test_invalid_protocol_version_stays_unknown(monkeypatch, value):
    def response(request):
        r = reply(request)
        r.data = value
        return r
    c, _ = client(monkeypatch, response)
    assert c.read('get_loco_api_version') == ('INVALID_FEEDBACK', None)
    c.close()


@pytest.mark.parametrize('name,service,api,value', [
    ('get_volume','voice',1005,{'volume':41}),
    ('get_actions','arm',7107,[[{'id':99,'name':'release_arm'}],[{'name':'named','time':1.}]]),
])
def test_audio_and_action_getters_are_only_fixed_reads_and_discard_errors(monkeypatch, name, service, api, value):
    c, channels = client(monkeypatch, lambda r: reply(r,raw=value))
    assert c.read(name) == (0, {'data':value})
    request = channels[f'rt/api/{service}/request'].sent
    assert request.header.identity.api_id == api and request.parameter == '{}'
    assert request.header.lease.id == 0 and request.binary == []
    c.close()
    c, _ = client(monkeypatch, lambda r: reply(r,code=3203,raw=value))
    assert c.read(name) == (3203,None)
    c.close()


@pytest.mark.parametrize('name,value', [('get_volume',{'volume':True}),('get_volume',{'volume':-1}),
    ('get_volume',{'volume':101}),('get_volume',{'volume':'41'}),('get_volume',{}),
    ('get_actions',None),('get_actions','malformed')])
def test_invalid_audio_and_action_feedback_stays_unknown(monkeypatch, name, value):
    c, _ = client(monkeypatch, lambda r: reply(r,raw=value))
    assert c.read(name) == ('INVALID_FEEDBACK',None)
    c.close()


def query_worker(pipe, stop, interface):
    while not stop.is_set():
        if pipe.poll(.02):
            token, name = pipe.recv()
            assert name in QUERY_READS
            meta = (time.monotonic(), time.time(), 5.)
            pipe.send(('query_result', (token-1, 0, {'data':999}, *meta)))
            pipe.send(('query_result', (token, 7301, {'data':0}, *meta)))
    pipe.close()


def test_spawned_on_demand_queries_correlate_and_never_fallback_to_stale_success():
    reader = IsolatedModeReadback(worker=query_worker)
    try:
        with pytest.raises(KeyError):
            reader.read('SetFsmId')
        for name in QUERY_READS:
            assert reader.read(name) == (7301, None)
    finally:
        reader.close()
    assert reader.read('get_balance') == ('MODE_WORKER_STOPPED', None)


def changing_query_worker(pipe, stop, interface):
    count = 0
    while not stop.is_set():
        if pipe.poll(.02):
            token, name = pipe.recv()
            assert name == 'get_stand_height'
            count += 1
            if count == 1:
                code, raw = 0, {'data': .765}
            else:
                # A late success cannot overwrite this query or renew its time.
                pipe.send(('query_result', (token-1, 0, {'data': 999}, time.monotonic(), 9000., 1.)))
                code, raw = 7301, {'data': 0}
            pipe.send(('query_result', (token, code, raw, time.monotonic()-.1, 1234., 5.)))
    pipe.close()


def test_loco_live_evidence_replaces_known_nonzero_with_unknown_and_expires_without_polling():
    reader = IsolatedModeReadback(worker=changing_query_worker)
    try:
        assert reader.read('get_stand_height') == (0, {'data': .765})
        row = reader.snapshot()['get_stand_height']
        assert row['current'] and row['response_recent'] and row['raw']['data'] == .765
        assert row['age_ms'] >= 100 and row['received_at'] == 1234.
        assert reader.read('get_stand_height') == (7301, None)
        row = reader.snapshot()['get_stand_height']
        assert row['response_recent'] and not row['current'] and row['raw'] is None
        assert row['code'] == 7301 and row['received_at'] == 1234.
        future = row['received_monotonic'] + 15.01
        with reader.lock:
            reader.received = future  # Keep worker health separate from result TTL.
        row = reader.snapshot(future)['get_stand_height']
        assert not row['response_recent'] and not row['current'] and row['raw'] is None
        assert row['code'] == 7301
        assert not reader.snapshot()['get_balance']['response_recent']
    finally:
        reader.close()
    assert not reader.snapshot()['get_stand_height']['response_recent']
