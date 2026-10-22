"""State evidence and final SDK boundary; all adapters below are in-process fakes."""
import asyncio
from copy import deepcopy
import json
import math
import time
from types import SimpleNamespace

import pytest

from g1_console.body import KNOWLEDGE
from g1_console.capabilities import CATALOG, READ_OPERATIONS
from g1_console.config import ConsoleError, Settings
from g1_console.service import ConsoleService
from g1_console.state_learning import interpret, history_sample


def feedback(fsm=0):
    return dict(joints=[dict(index=m['index'], q=math.radians(160) if m['index'] in (3,9) else 0., dq=0., temperature=[35,40]) for m in KNOWLEDGE['motors']],
                imu=dict(rpy=[.1,-.05,0.]), fsm=fsm, mode_machine=4, mode_pr=0,
                received=time.monotonic(), fsm_received=time.monotonic(), tick=123)


@pytest.mark.parametrize('fsm', [0,500,702,999])
def test_fsm_and_quiet_folded_joints_do_not_identify_action_support_or_safe_range(fsm):
    t=feedback(fsm)
    history=[history_sample(t, n/20) for n in range(240)]
    a=interpret(t, True, True, KNOWLEDGE, history, 'robot')
    assert a['control_mode']['fsm']==fsm and a['control_mode']['active_capability_id'] is None
    assert not a['control_mode']['activity_verified']
    assert a['posture']['description']=='双膝显著屈曲'
    assert a['posture']['knee_deg']==pytest.approx([160,160])
    assert a['posture']['whole_body_pose']==a['posture']['support']=='unknown'
    assert a['observation_window']['samples']==240 and a['observation_window']['max_abs_dq_rad_s']==0
    assert a['observation_window']['max_joint_span_deg']==0
    assert a['observation_window']['learned_safe_range'] is None
    assert a['observation_window']['certified_safe'] is False
    assert a['safety']['execution_allowed'] is a['safety']['fall_free_guarantee'] is False
    assert a['safety']['validated_action_envelopes']==[]
    assert {c['id']:c['status'] for c in a['safety']['checks']}['walking_fsm']==('matched' if fsm==500 else 'violated')
    assert all(c['status']=='unknown' for c in a['safety']['checks'] if c['id'] in ('support','balance','terrain','calibration','health','recovery','action_contract'))


@pytest.mark.parametrize('fresh,mode', [(False,'robot'),(True,'demo'),(False,'disconnected')])
def test_missing_real_current_evidence_invalidates_live_interpretation(fresh,mode):
    t=feedback()
    a=interpret(t,fresh,True,KNOWLEDGE,[history_sample(t,1)],mode)
    assert not a['current'] and a['control_mode']['fsm'] is None
    assert a['posture']['knee_deg'] is a['posture']['imu_rpy_deg'] is None
    assert a['observation_window']['samples']==0
    assert all(c['status']=='unknown' for c in a['safety']['checks'])


def test_nonfinite_missing_feedback_never_passes_guards_or_enters_json():
    t=feedback(500)
    t['joints'][3]['q']=float('nan');t['joints'][0]['dq']=float('inf')
    t['joints'][1]['temperature']=[float('nan'),20]
    t['imu']['rpy']=[0,float('nan'),0]
    a=interpret(t,True,True,KNOWLEDGE,[history_sample(t,1)],'robot')
    checks={c['id']:c['status'] for c in a['safety']['checks']}
    assert all(checks[k]=='unknown' for k in ('real_feedback','tilt_guard','thermal_guard'))
    assert a['posture']['knee_deg'] is None
    json.dumps(a,allow_nan=False)
    t=feedback();t['joints'].pop()
    assert interpret(t,True,True,KNOWLEDGE,mode='robot')['safety']['checks'][0]['status']=='unknown'


def test_project_guard_passes_still_do_not_grant_motion_and_thresholds_remain_references():
    t=feedback(500)
    a=interpret(t,True,True,KNOWLEDGE,mode='robot')
    assert all(c['status']=='matched' for c in a['safety']['checks'][:4])
    assert not a['safety']['execution_allowed']
    t['imu']['rpy'][0]=.36;t['joints'][0]['temperature'][0]=70
    checks={c['id']:c['status'] for c in interpret(t,True,True,KNOWLEDGE,mode='robot')['safety']['checks']}
    assert checks['tilt_guard']==checks['thermal_guard']=='violated'
    policy=KNOWLEDGE['state_safety']
    assert len(policy['sdk_helpers'])==7 and not policy['validated_action_envelopes']
    assert all(h['verified_device_threshold'] is None and not h['execution_allowed'] for h in policy['sdk_helpers'])
    assert policy['learning_policy']['passive_samples_can_expand_boundary'] is False
    values=[v for m in KNOWLEDGE['motors'] for v in m['actual_config'].values()]
    assert len(values)==253 and all(v is None for v in values)


def test_device_indices_are_noncontiguous_and_right_arm_changes_are_included():
    t=feedback();index=t['joints'][-1]['index']
    assert index>22 and len(t['joints'])==23
    history=[history_sample(t,1)]
    t['joints'][-1]['q']=.1
    history.append(history_sample(t,2))
    a=interpret(t,True,True,KNOWLEDGE,history,'robot')
    assert a['safety']['checks'][0]['status']=='matched'
    assert a['observation_window']['max_joint_span_deg']==pytest.approx(math.degrees(.1))
    t['joints'][-1]['index']=100
    a=interpret(t,True,True,KNOWLEDGE,mode='robot')
    assert a['safety']['checks'][0]['status']=='unknown'


def test_quiet_straight_legs_camera_and_agreeing_fsm_still_do_not_prove_standing():
    t=feedback(500)
    for joint in t['joints']:
        joint['q']=0.
    t['safety_feedback']={
        'sport':dict(current=True,raw=dict(fsm_id=500,fsm_mode=0)),
        'fsm_mode':dict(current=True,raw=dict(data=0))}
    a=interpret(t,True,True,KNOWLEDGE,mode='robot',camera=dict(status='live',age_ms=5))
    assert a['control_mode']['feedback_consistency']=='matched'
    standing=a['standing']
    assert standing['already_standing'] is standing['selected_transition'] is None
    assert standing['status']=='unknown' and not standing['execution_allowed']
    assert all(standing['slow_stand'][k] is None for k in ('speed_parameter','duration_s','trajectory','firmware_support_verified'))
    assert {r['id']:r['status'] for r in standing['evidence']}['load']=='unknown'
    assert all(not c['execution_allowed'] and c['firmware_applicability_verified'] is None for c in standing['candidates'])
    t['safety_feedback']['sport']['raw']['fsm_id']=0
    assert interpret(t,True,True,KNOWLEDGE,mode='robot')['control_mode']['feedback_consistency']=='violated'
    t['safety_feedback']['fsm_mode']['current']=False
    assert interpret(t,True,True,KNOWLEDGE,mode='robot')['control_mode']['feedback_consistency']=='unknown'
    stale=interpret(t,False,True,KNOWLEDGE,mode='robot')['standing']
    assert all(r['status']=='unknown' for r in stale['evidence'])
    assert stale['current_findings']==['双膝形态缺少新鲜反馈','缺少新鲜 FSM 回读']


def test_final_dispatch_blocks_every_motion_even_with_writable_switch_owner_and_floor(tmp_path):
    async def check():
        called=[]
        def execute(op,p):
            called.append(op)
            return 0, {'fake':True}
        s=ConsoleService(Settings(data_dir=tmp_path,read_only=False))
        s.adapter=SimpleNamespace(mode='robot',execute=execute)
        s.owner='operator';s.lease_until=time.monotonic()+60;s.floor_verified=True
        for cap in CATALOG:
            if cap['risk'] in ('motion','guarded'):
                with pytest.raises(ConsoleError) as error:
                    await s._rpc(cap['id'],{})
                assert error.value.code=='SAFETY_ENVELOPE_UNVERIFIED'
        assert not called
        # Known positive controls prove that the actual SDK dispatch path is reachable.
        for op in READ_OPERATIONS | {'stop'}:
            assert await s._rpc(op,{})=={'fake':True}
        assert set(called)==READ_OPERATIONS | {'stop'}
        readonly=ConsoleService(Settings(data_dir=tmp_path/'readonly',read_only=True))
        readonly.adapter=s.adapter
        with pytest.raises(ConsoleError) as error:
            await readonly._rpc('stop',{})
        assert error.value.code=='READ_ONLY'
    asyncio.run(check())


@pytest.mark.parametrize('fresh,mode', [(True,'robot'),(False,'robot'),(True,'demo'),(False,'disconnected')])
def test_human_pose_confirmation_is_preserved_but_never_becomes_live_load_or_motion_authority(fresh,mode):
    a=interpret(feedback(),fresh,True,KNOWLEDGE,mode=mode)
    report=a['standing']['human_observation']
    assert report['source']=='user_direct_report' and report['current'] is False
    assert report['reported_pose']=='seated_on_floor' and report['reported_external_support']=='cabinet'
    assert not report['reported_person_support'] and report['cabinet_load_n'] is None
    assert report['supersedes']['previous']=='person' and report['supersedes']['replacement']=='cabinet'
    assert not report['claim_provenance']['pose_feet_and_digital_view']['reconfirmed_in_latest_message']
    assert report['reported_left_sole_contact'] and report['reported_right_sole_contact']
    assert report['reported_digital_pose_matches'] and not report['reported_independent_standing']
    assert report['observed_at_utc'] is report['left_load_n'] is report['right_load_n'] is None
    assert report['recorded_at_utc']==KNOWLEDGE['standing_knowledge']['support_audit']['human_observation']['recorded_at_utc']
    assert all(foot['contact'] is foot['load_n'] is None for foot in a['posture']['feet'].values())
    assert a['standing']['already_standing'] is a['standing']['selected_transition'] is None
    assert not a['standing']['execution_allowed'] and not a['safety']['execution_allowed']
    assert all(row['status']=='unknown' for row in a['standing']['conditional_plan'][1:])
    # Even an apparent straight-leg pose never refreshes the historical human report.
    t=feedback(500)
    for joint in t['joints']:joint['q']=0.
    b=interpret(t,True,True,KNOWLEDGE,mode='robot')
    assert b['standing']['human_observation']==report
    assert b['posture']['whole_body_pose']=='unknown'


def test_observations_persist_as_history_without_learning_authority_and_reset_coordinates(tmp_path):
    async def check():
        t=feedback()
        s=ConsoleService(Settings(data_dir=tmp_path,read_only=True))
        s.adapter=SimpleNamespace(mode='robot',sample=lambda:deepcopy(t))
        await s._sample();await s._sample()
        assert len(s.body_history)==1
        events=[json.loads(line) for line in s.journal.read_text().splitlines()]
        assert len(events)==1 and events[0]['kind']=='state_observed'
        assert not events[0]['current'] and not events[0]['execution_allowed']
        assert events[0]['posture']['support']=='unknown'
        t.update(mode_pr=1,received=time.monotonic())
        await s._sample()
        assert len(s.body_history)==1 and s.body_history[0]['mode_pr']==1
        t['received']=time.monotonic()-2
        await s._sample()
        assert not s.body_history
        # A historical record never restores current evidence after a new process starts.
        restarted=ConsoleService(Settings(data_dir=tmp_path,read_only=True))
        assert not restarted.fresh() and not restarted.body_history
        assert restarted.owner is None
    asyncio.run(check())
