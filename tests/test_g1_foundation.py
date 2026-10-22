"""Pinned knowledge vs live evidence; unknown is never converted to motion authority."""
import asyncio
import ast
from copy import deepcopy
import hashlib
import json
import re
import time

import pytest
from g1_console.assessment import assess
from g1_console.body import KNOWLEDGE, JOINTS
from g1_console.capabilities import CATALOG
from g1_console.config import Settings
from g1_console.service import ConsoleService
from scripts.build_g1_motor_knowledge import build, ROOT, board_audit, standing_support_audit


def telemetry():
    return dict(joints=[dict(index=j['index'], q=0., dq=0., temperature=[32,35], voltage=48., mode=1, status=0)
                        for j in JOINTS], fsm=0, mode_machine=4, mode_pr=0, imu=None)


FIRMWARE = [dict(id=1,name='turn_back_wave',fsm=[500,501]),
            dict(id=31,name='extend_right_arm_forward',mode_machine=[4]),dict(id=99,name='release_arm')]


def test_protocol_success_and_single_responder_do_not_establish_motion_firmware_or_unlock_modes():
    review = build()['motion_control_review']
    protocol = review['protocol_review']
    assert protocol['current'] is False
    assert {r['service']:r['api_version'] for r in protocol['services']} == {
        'sport':'1.0.0.0', 'motion_switcher':'1.0.0.1', 'robot_state':'1.0.0.2'}
    assert all(r['firmware_version'] is None for r in protocol['services'])
    assert review['device_motion_firmware'] is review['firmware']['selected_manual_branch'] is None
    assert all(r['device_applicable'] is None and not r['execution_allowed'] for r in review['modes'])
    evidence = json.loads((ROOT/'g1_console/assets/safety_evidence_sources/loco_readiness_20261021_v27.json').read_text())
    failed = [r for r in evidence['reads'] if r['api_id'] in (7003,7004,7005,7006)]
    assert len(failed) == 12 and all(r['code'] == 7301 and r['raw'] is None for r in failed)
    responders = json.loads((ROOT/'g1_console/assets/safety_evidence_sources/loco_responders_20261021_v27.json').read_text())
    assert all(len(r['replies']) == 1 for r in responders['queries'])
    assert len({r['replies'][0]['publication_handle'] for r in responders['queries']}) == 1
    assert all(r['samples'] == 0 for r in responders['string_states'].values())


def test_foundation_is_reproducible_and_sources_are_fingerprinted():
    assert build() == KNOWLEDGE
    assert len(KNOWLEDGE['motors']) == 23
    assert len(KNOWLEDGE['feedback_fields']) == 10 and len(KNOWLEDGE['command_fields']) == 7
    for s in KNOWLEDGE['sources'].values():
        assert hashlib.sha256((ROOT/s['path']).read_bytes()).hexdigest() == s['sha256']
        assert s['commit'] in s['url']
    knee = next(m for m in KNOWLEDGE['motors'] if m['index'] == 3)
    assert knee['example_only']['kp'] == 100
    assert all(v is None for v in knee['actual_config'].values())
    assert KNOWLEDGE['identity']['firmware_version'] is None


def test_firmware_conditions_do_not_grant_execution():
    a = assess(telemetry(), True, 50, CATALOG, FIRMWARE, True)
    rows = {r['id']:r['compatibility'] for r in a['firmware_actions']}
    assert [rows[i]['status'] for i in (1,31,99)] == ['violated','matched','unknown']
    assert a['checks'][1]['status'] == 'violated'  # FSM 0 is not the project's walking state.
    assert a['checks'][2]['status'] == a['checks'][3]['status'] == 'unknown'
    assert all(not c['execution_allowed'] for c in a['capabilities'])
    assert all(not r['execution_allowed'] for r in rows.values())


def test_documented_motor_facts_never_replace_device_configuration():
    motors = {m['index']:m for m in KNOWLEDGE['motors']}
    assert {i:m['documented_variant']['gear_ratio'] for i,m in motors.items()
            if m['documented_variant']['gear_ratio'] is not None} == {0:14.3,1:22.5,6:14.3,7:22.5}
    assert motors[1]['documented_variant']['model']['effort_nm'] == 139
    assert motors[1]['model']['effort_nm'] == 88  # Preserve the conflicting legacy reference.
    assert motors[10]['documented_variant']['model']['effort_nm'] == 35
    assert all(v is None for m in motors.values() for v in m['actual_config'].values())
    assert [(c['index'],c['component']) for c in KNOWLEDGE['temperature_channels']] == [(0,'casing'),(1,'winding')]
    assert [c['sdk_helper_default_c'] for c in KNOWLEDGE['temperature_channels']] == [85,120]
    assert '不是' in KNOWLEDGE['rejected_inferences'][-1]['reason']
    assert KNOWLEDGE['device_audit']['telemetry_snapshot']['read_only']


def test_board_readings_and_installed_model_do_not_become_motor_configuration():
    audit = KNOWLEDGE['board_audit']
    assert audit['current'] is False and audit['physical_board_count'] is None
    assert audit['motor_configuration_values_obtained'] == audit['command_writes'] == audit['rpc_calls'] == 0
    topics = {r['topic']:r for r in audit['topic_readings']}
    assert topics['rt/lf/mainboardstate']['samples'] > 0
    assert topics['rt/lf/bmsstate']['raw_fields']['version_high'] == 1
    assert topics['rt/lowcmd']['raw_fields']['last_payload_all_kp_kd_zero']
    assert not any(r['publisher_device_verified'] for r in topics.values())
    assert {d['index'] for d in audit['installed_model']['differences']} == {4,5,10,11}
    assert not audit['installed_model']['active_controller_use_verified']
    assert all(v is None for m in KNOWLEDGE['motors'] for v in m['actual_config'].values())
    a = assess(telemetry(),True,20,CATALOG,[],True)
    assert a['checks'][3]['status'] == 'unknown' and not a['execution_allowed']


def test_changed_board_source_is_rejected(tmp_path,monkeypatch):
    import scripts.build_g1_motor_knowledge as builder
    asset = tmp_path/'g1_console/assets/g1_board_parameter_audit.json'
    asset.parent.mkdir(parents=True)
    evidence = json.loads((ROOT/'g1_console/assets/g1_board_parameter_audit.json').read_text())
    asset.write_text(json.dumps(evidence))
    raw = tmp_path/evidence['passive_read']['path']
    raw.parent.mkdir(parents=True,exist_ok=True)
    raw.write_text('{}')  # A replaced sample must never silently enter the persisted foundation.
    monkeypatch.setattr(builder,'ROOT',tmp_path)
    with pytest.raises(AssertionError,match='passive_read'):
        board_audit(KNOWLEDGE['motors'])


def test_changed_human_or_read_evidence_cannot_silently_replace_foundation(tmp_path,monkeypatch):
    import scripts.build_g1_motor_knowledge as builder
    asset=tmp_path/'g1_console/assets/g1_standing_support_audit.json'
    asset.parent.mkdir(parents=True)
    audit=json.loads((ROOT/'g1_console/assets/g1_standing_support_audit.json').read_text())
    asset.write_text(json.dumps(audit))
    for source in audit['sources']:
        target=tmp_path/source['path'];target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes((ROOT/source['path']).read_bytes())
    report=tmp_path/audit['human_source']
    data=json.loads(report.read_text());data['current']=True
    report.write_text(json.dumps(data))
    monkeypatch.setattr(builder,'ROOT',tmp_path)
    with pytest.raises(AssertionError,match='human_posture'):
        standing_support_audit()


def test_failed_official_getters_and_manual_nominal_time_do_not_become_device_parameters():
    a=KNOWLEDGE['standing_knowledge']['support_audit']
    assert not a['current'] and not a['execution_allowed']
    reads={r['method']:r for r in a['official_reads']['readings']}
    for name in ('GetBalanceMode','GetStandHeight'):
        assert reads[name]['code']==7301 and reads[name]['payload'] is None
    assert reads['GetServerApiVersion']['service']=='arm' and reads['GetServerApiVersion']['payload'] is None
    groups=reads['GetActionList']['payload']
    assert [len(group) for group in groups]==[20,4]
    nominal=next(r for r in a['manual_review']['control_meanings'] if 'documented_nominal_duration_s' in r)
    assert nominal['documented_nominal_duration_s']==5 and nominal['device_duration_s'] is None
    assert not nominal['device_verified']
    assert all(v is None for m in KNOWLEDGE['motors'] for v in m['actual_config'].values())


def test_standing_probe_is_limited_to_official_service_specific_getters():
    from scripts.audit_g1_standing_reads import READS
    loco=(ROOT/'g1_console/assets/capability_sources/g1_loco_api.hpp').read_text()
    arm=ast.parse((ROOT/'externals/unitree_sdk2_python/unitree_sdk2py/g1/arm/g1_arm_action_api.py').read_text())
    arm_ids={node.targets[0].id:ast.literal_eval(node.value) for node in arm.body
             if isinstance(node,ast.Assign) and isinstance(node.value,ast.Constant)}
    expected={('sport',int(re.search(r'ROBOT_API_ID_LOCO_GET_'+name+r'\s*=\s*(\d+)',loco)[1]))
              for name in ('BALANCE_MODE','STAND_HEIGHT')}
    expected.add(('arm',arm_ids['ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST']))
    assert {(service,api) for service,api,_,_ in READS if api!=1}==expected
    assert {(service,api) for service,api,_,_ in READS if api==1}=={
        ('sport',1),('motion_switcher',1),('arm',1)}
    assert len(READS)==6
    assert ('sport',7107) not in {(service,api) for service,api,_,_ in READS}  # Speed setter shares arm getter ID.


def test_cabinet_correction_keeps_prior_claims_and_raw_versions_without_inventing_values():
    audit=KNOWLEDGE['standing_knowledge']['support_audit']
    human=audit['human_observation'];prior=human['supersedes']
    old=json.loads((ROOT/prior['path']).read_text())
    assert old['reported_external_support']=='person'
    assert human['reported_external_support']=='cabinet' and not human['reported_person_support']
    provenance=human['claim_provenance']['pose_feet_and_digital_view']
    assert provenance['recorded_at_utc']==old['recorded_at_utc']
    assert not provenance['reconfirmed_in_latest_message']
    review=audit['firmware_review']
    assert review['device_motion_firmware'] is review['lowstate_version']['decoded_version'] is None
    assert review['lowstate_version']['raw']==[0,0] and review['lowstate_version']['samples']>0
    assert all(review['foot_force'][key] is None for key in ('left_load_n','right_load_n','cabinet_load_n'))
    assert not review['current'] and review['rpc_calls']==review['command_writes']==0
    tree=ast.parse((ROOT/'scripts/audit_g1_firmware_support.py').read_text())
    imported={a.name for n in ast.walk(tree) if isinstance(n,(ast.Import,ast.ImportFrom)) for a in n.names}
    assert {'DataReader','BuiltinDataReader'} <= imported
    assert not imported & {'DataWriter','Publisher','ChannelPublisher','Client','LocoClient','MotionSwitcherClient'}


def test_board_audit_cannot_import_command_publishers_or_rpc_clients():
    tree = ast.parse((ROOT/'scripts/audit_g1_boards.py').read_text())
    imported = {a.name for n in ast.walk(tree) if isinstance(n,(ast.Import,ast.ImportFrom)) for a in n.names}
    assert not imported & {'DataWriter','Publisher','ChannelPublisher','Client','LocoClient','MotionSwitcherClient'}
    assert 'DataReader' in imported and 'BuiltinDataReader' in imported


def test_ota_cache_and_contact_reports_never_prove_firmware_or_standing():
    audit=KNOWLEDGE['standing_knowledge']['support_audit']
    review=audit['applicability_review']
    assert review['current'] is False
    assert review['ota_history']['package_directory_label']=='1.5.3.8_G1_Edu+'
    assert review['ota_history']['device_motion_firmware'] is KNOWLEDGE['identity']['firmware_version'] is None
    assert {c['id'] for c in review['contacts']}=={'left_foot','right_foot','seat','cabinet'}
    assert all(c['contact_verified'] is c['load_n'] is None for c in review['contacts'])
    assert all(c['applicable_to_current_pose'] is None and not c['execution_allowed'] for c in review['capabilities'])
    assert all(v is None for k,v in review['slow_control'].items() if k!='meaning')
    assert len(audit['conditional_plan'])==6 and all(p['completion_evidence'] for p in audit['conditional_plan'])
    assert review['passive_status']['active_capability'] is None
    assert all(row['samples']==0 for row in review['passive_status']['topics'])
    assert all(value is None for motor in KNOWLEDGE['motors'] for value in motor['actual_config'].values())


def test_extra_status_probe_is_passive_and_only_uses_matching_string_topics():
    tree=ast.parse((ROOT/'scripts/audit_g1_passive_status.py').read_text())
    imported={a.name for n in ast.walk(tree) if isinstance(n,(ast.Import,ast.ImportFrom)) for a in n.names}
    assert {'DataReader','BuiltinDataReader','String_'} <= imported
    assert not imported & {'DataWriter','Publisher','ChannelPublisher','Client','LocoClient','MotionSwitcherClient'}
    topics=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(
        isinstance(t,ast.Name) and t.id=='TOPICS' for t in n.targets))
    review=KNOWLEDGE['standing_knowledge']['support_audit']['applicability_review']
    assert set(topics)=={r['topic'] for r in review['passive_status']['topics']}
    assert not any(isinstance(n,ast.Attribute) and n.attr in ('Write','_Call','SetWriter') for n in ast.walk(tree))


@pytest.mark.parametrize('fresh,mode,missing,status', [(True,4,False,'matched'),(True,10,False,'violated'),
    (True,None,False,'unknown'),(False,4,False,'unknown'),(True,4,True,'violated')])
def test_variant_documentation_requires_current_matching_robot(fresh,mode,missing,status):
    t=telemetry();t['mode_machine']=mode
    if missing:t['joints'].pop()
    a=assess(t,fresh,50,CATALOG,FIRMWARE,True)
    assert a['hardware_variant']['status'] == status
    assert a['checks'][3]['status'] == 'unknown'  # SDK thermal defaults never diagnose health.
    assert a['execution_allowed'] is False


@pytest.mark.parametrize('mode_pr', [None,1])
def test_ab_or_unknown_coordinates_cannot_claim_ankle_model_match(mode_pr):
    t=telemetry(); t['mode_pr']=mode_pr
    a=assess(t,True,100,CATALOG,[],True)
    assert [j['index'] for j in a['joint_limits'] if j['status']=='unknown'] == [4,5,10,11]


def test_tiny_deviation_is_preserved_without_fault_diagnosis():
    t=telemetry(); j=next(j for j in t['joints'] if j['index']==10); j['q']=.5244
    a=assess(t,True,50,CATALOG,[],True)
    result=next(j for j in a['joint_limits'] if j['index']==10)
    assert result['status']=='violated' and result['deviation_rad']==pytest.approx(.0008)
    assert '不能据此判定故障' in result['detail']


def test_expired_evidence_never_claims_current_match():
    a=assess(telemetry(),False,50,CATALOG,FIRMWARE,True)
    assert all(j['status']=='unknown' for j in a['joint_limits'])
    assert all(r['compatibility']['status']=='unknown' for r in a['firmware_actions'])
    assert all(c['status']=='unknown' for c in a['capabilities'])
    assert a['temperature_range_c'] is None and not a['current']
    b=assess(telemetry(),True,1600,CATALOG,FIRMWARE,True)
    assert b['checks'][1]['status']=='unknown'
    assert b['firmware_actions'][0]['compatibility']['status']=='unknown'


def test_history_only_counts_distinct_fresh_received_samples_and_resets(tmp_path):
    async def check():
        s=ConsoleService(Settings(data_dir=tmp_path))  # Only the in-process synthetic adapter.
        await s.connect('demo','eth0')
        value=deepcopy(s.telemetry)
        value.update(received=time.monotonic(),fsm_received=time.monotonic())
        s.body_history.clear()
        s.adapter.sample=lambda:deepcopy(value)
        await s._sample(); await s._sample()
        assert len(s.body_history)==1
        value['received']=time.monotonic()+.0001
        await s._sample()
        assert len(s.body_history)==2
        value['received']=time.monotonic()-2
        await s._sample()
        assert not s.body_history
        await s.disconnect()
        assert not s.body_history
    asyncio.run(check())


def test_only_local_media_may_be_absent_and_present_files_must_still_match(tmp_path):
    from scripts import build_g1_motor_knowledge as builder
    builder.pinned('doc/evidence/not_public/frame.jpg', '0' * 64)  # Local-only photo, not in this checkout.
    with pytest.raises(FileNotFoundError):
        builder.pinned('doc/evidence/not_public/receipt.json', '0' * 64)
    with pytest.raises(AssertionError):
        builder.pinned('README.md', '0' * 64)
