"""No hardware: expiry, raw-field authority and receive-only endpoint boundaries."""
from dataclasses import dataclass
from pathlib import Path

from g1_console.body import KNOWLEDGE
from g1_console.safety_feedback import FeedbackStore, PassiveSafetyFeedback, CHANNELS
from g1_console.state_learning import interpret
from test_g1_state_safety import feedback


def test_raw_zero_and_unknown_task_never_become_health_support_or_activity():
    store = FeedbackStore()
    store.record('bms', {'soc': 0, 'bmsstate': [0]*5, 'cell_vol': [0]*40})
    store.record('sport', {'fsm_id': 0, 'fsm_mode': 0, 'task_id': 4, 'task_time': 0.})
    store.record('motion_mode', {'form': '0', 'name': 'ai'})
    t = feedback();t['safety_feedback'] = store.snapshot()
    camera = dict(status='live', age_ms=25)
    result = interpret(t, True, True, KNOWLEDGE, mode='robot', camera=camera)
    checks = {c['id']: c for c in result['safety']['checks']}
    assert result['feedback']['bms']['raw']['soc'] == 0
    assert checks['health']['acquired'] and checks['terrain']['acquired'] and checks['recovery']['acquired']
    assert len([c for c in checks.values() if c['id'] in ('support','balance','terrain','calibration','health','recovery','action_contract') and c['status']=='unknown']) == 7
    assert result['control_mode']['active_capability_id'] is None
    assert not result['control_mode']['activity_verified'] and not result['safety']['execution_allowed']
    assert result['posture']['support'] == 'unknown'


def test_error_and_expiry_remove_raw_evidence_including_identical_zero_values():
    store = FeedbackStore();store.record('bms', {'soc': 34})
    live = store.snapshot();assert live['bms']['current']
    last = store.values['bms']['received']
    assert not store.snapshot(last-1)['bms']['current']
    stale = store.snapshot(last+1.501)
    assert not stale['bms']['current'] and stale['bms']['raw'] is None
    store.record('bms', None, 3104)
    assert store.snapshot()['bms']['code'] == 3104 and store.snapshot()['bms']['raw'] is None
    for invalid in ({'v':float('nan')}, {'v':'x'*4097}, None):
        store.record('bms', invalid)
        assert not store.snapshot()['bms']['current']
    store.record('bms', {'soc':0});assert store.snapshot()['bms']['current']
    for fresh, mode in [(False,'robot'), (True,'demo')]:
        t=feedback();t['safety_feedback']=store.snapshot()
        a=interpret(t,fresh,True,KNOWLEDGE,mode=mode,camera=dict(status='live',age_ms=1))
        assert not a['feedback'] and not a['camera_evidence']['current']
        assert all(not c.get('acquired') for c in a['safety']['checks'])


def test_only_passive_subscribers_created_and_closed(monkeypatch):
    from unitree_sdk2py.core import channel
    calls=[]
    @dataclass
    class Sample:
        soc:int
    class Subscriber:
        def __init__(self, topic, cls):
            self.topic=topic;calls.append(('subscribe',topic,cls.__idl_typename__))
        def Init(self, cb, queue):
            assert queue==1
            if self.topic=='rt/lf/bmsstate':cb(Sample(34))
        def Close(self):calls.append(('close',self.topic))
    monkeypatch.setattr(channel, 'ChannelSubscriber', Subscriber)
    def forbidden(*a,**kw):raise AssertionError('A passive reader must not create any writer')
    monkeypatch.setattr(channel, 'ChannelPublisher', forbidden)
    store=PassiveSafetyFeedback()
    assert store.snapshot()['bms']['raw']=={'soc':34}
    assert {c[1] for c in calls}=={v[0] for k,v in CHANNELS.items() if k not in ('motion_mode','fsm_mode')}
    assert all('unitree_go' not in c[2] for c in calls)
    store.close();store.record('bms', {'soc':99})
    assert store.snapshot()['bms']['raw'] is None
    assert len([c for c in calls if c[0]=='close'])==8


def test_additional_posture_channels_expire_and_never_prove_support_or_activity():
    store = FeedbackStore()
    store.record('secondary_imu', {'rpy': [0., 0., 0.]})
    store.record('pelvis_odom', {'header': {'frame_id': 'odom'}, 'pose': {'position': {'z': .31}}})
    store.record('torso_odom', {'header': {'frame_id': 'odom'}})
    store.record('fsm_mode', {'data': 0})
    t = feedback();t['safety_feedback'] = store.snapshot()
    result = interpret(t, True, True, KNOWLEDGE, mode='robot')
    assert result['posture']['secondary_imu_rpy_deg'] == [0., 0., 0.]
    assert result['posture']['feet'] == {'left': {'contact': None, 'load_n': None}, 'right': {'contact': None, 'load_n': None}}
    assert result['posture']['support'] == result['posture']['whole_body_pose'] == 'unknown'
    checks = {c['id']:c for c in result['safety']['checks']}
    assert len(checks['balance']['acquired']) == 4 and checks['balance']['status'] == 'unknown'
    assert not result['control_mode']['activity_verified'] and not result['safety']['execution_allowed']
    t['safety_feedback'] = store.snapshot(max(v['received'] for v in store.values.values()) + 6.001)
    expired = interpret(t, True, True, KNOWLEDGE, mode='robot')
    assert expired['posture']['secondary_imu_rpy_deg'] is None
    assert all(v['raw'] is None and not v['current'] for v in expired['feedback'].values())
    assert len(next(c for c in expired['safety']['checks'] if c['id']=='balance')['acquired']) == 1


def test_fsm_mode_uses_only_dedicated_official_getter_without_defaults():
    from g1_console.safety_feedback import read_fsm_mode
    class Client:
        def _Call(self, api, parameter):
            assert (api, parameter) == (7002, '')
            return self.result
    client = Client()
    client.result = (0, '{"data":0}')
    assert read_fsm_mode(client) == (0, {'data': 0})
    for invalid in ('{}', 'null', '[]', '{"data":true}', '{"data":-1}', '{"data":4294967296}', 'bad'):
        client.result = (0, invalid)
        assert read_fsm_mode(client) == ('INVALID_FEEDBACK', None)
    client.result = (7301, '{"data":0}')
    assert read_fsm_mode(client) == (7301, None)


def test_pinned_evidence_and_binding_reproduce_foundation_without_importing_hardware():
    from scripts.build_g1_motor_knowledge import build
    assert build()==KNOWLEDGE
    audit=KNOWLEDGE['safety_evidence_audit']
    assert audit['current'] is False and not audit['execution_allowed']
    assert len(audit['areas'])==7 and all(not x['safety_verified'] for x in audit['areas'])
    assert len([v for m in KNOWLEDGE['motors'] for v in m['actual_config'].values() if v is None])==253
    posture = audit['posture_interface_audit']
    assert not posture['current'] and not posture['execution_allowed']
    assert posture['foot_contact'] is None and posture['left_load_n'] is None and posture['right_load_n'] is None
    assert posture['whole_body_pose'] == 'unknown' and not posture['activity_verified']
    assert posture['active_capability_id'] is None
    getters = {r['api_id']: r for r in posture['official_getters']}
    assert set(getters) == {1001, 7001, 7002, 7003, 7004, 7005}
    for api in (7003, 7004, 7005):
        assert getters[api]['code'] == 7301 and getters[api]['payload'] is None
        assert getters[api]['error_symbol'] == 'UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE'
    assert '四路 foot_force=[0,0,0,0]' in ' '.join(posture['limitations'])
    # Match field order, scalar types and DDS namespace to the pinned official C++ definition.
    from g1_console.hg_sport_state import SportModeState_
    assert list(SportModeState_.__annotations__)==['fsm_id','fsm_mode','task_id','task_time']
    assert SportModeState_.__idl_typename__=='unitree_hg.msg.dds_.SportModeState_'
    source=Path('g1_console/assets/safety_evidence_sources/SportModeState_.hpp').read_text()
    for field in ['uint32_t fsm_id_', 'uint32_t fsm_mode_', 'uint32_t task_id_', 'float task_time_']:
        assert field in source
    assert 'foot_force' not in source
