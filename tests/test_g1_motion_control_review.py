"""Documented mode numbers and stop names must never become motion authority."""
import hashlib
import gzip
import json

import pytest

from g1_console.body import KNOWLEDGE
from scripts import build_g1_motor_knowledge as builder


def test_installed_sdk_init_and_correlated_reads_do_not_fill_private_readiness_or_load():
    audit = builder.motion_control_review()['installed_sdk_audit']
    assert audit['constructor_rpc_requests'] == audit['init_rpc_requests'] == 0
    assert len(audit['sdk_getters']) == 7
    for row in audit['sdk_getters']:
        assert row['replies'] and all(r['code'] == row['sdk_code'] for r in row['replies'])
        if row['api_id'] in range(7003, 7007):
            assert row['sdk_code'] == 7301 and row['sdk_value'] is None
            assert all(r['data'] is None for r in row['replies'])
    odom = audit['streams']['rt/odommodestate']
    assert odom['samples'] == 619 and odom['last']['foot_force'] == [0, 0, 0, 0]
    assert audit['bilateral_foot_load_n'] is audit['external_support_load_n'] is None
    assert audit['device_motion_firmware'] is audit['exact_7301_predicate'] is None
    assert not any(audit[k] for k in ('current', 'execution_allowed', 'motion_authority_verified',
                                     'command_writes', 'lease_requests'))


def test_modified_installed_sdk_audit_rejected_before_entering_foundation(monkeypatch):
    source = builder.ROOT/'g1_console/assets/safety_evidence_sources/installed_loco_sdk_20261022_v36.json'
    original = type(source).read_bytes
    monkeypatch.setattr(type(source), 'read_bytes', lambda p: b'{}' if p == source else original(p))
    with pytest.raises(AssertionError, match='installed_loco_sdk_20261022_v36.json'):
        builder.motion_control_review()


def test_reboot_read_errors_remain_unknown_and_do_not_grant_motion_authority():
    review = builder.motion_control_review()
    audit = review['reboot_readonly_audit']
    reads = audit['reads']
    assert len(reads) == 33 and {r['trial'] for r in reads} == {0, 1, 2}
    assert {(r['service'], r['api_id']) for r in reads} == {
        ('sport', 1), ('motion_switcher', 1), ('robot_state', 1),
        *(('sport', i) for i in range(7001, 7007)),
        ('motion_switcher', 1001), ('robot_state', 1006)}
    assert sum(r['code'] == 7301 for r in reads) == 12
    assert sum(r['code'] == 3203 for r in reads) == 3
    assert not any(r['code'] == 3104 for r in reads)
    assert all(r['code'] == 7301 and r['raw'] is None for r in reads
               if r['service'] == 'sport' and r['api_id'] in range(7003, 7007))
    assert audit['device_motion_firmware'] is audit['exact_7301_predicate'] is None
    assert audit['before'] == audit['after'] == dict(fsm=0, control_owner=None, armed=False)
    assert not any(audit[k] for k in ('current', 'execution_allowed', 'motion_authority_verified',
                                     'command_writes', 'lease_requests'))
    assert review['firmware']['selected_manual_branch'] is None


def test_successful_pc4_query_and_sdk_constants_do_not_prove_motion_host_support():
    review = KNOWLEDGE['motion_control_review']
    assert review['device_motion_firmware'] is None
    assert review['firmware']['ota_query_returncode'] == 0
    assert review['firmware']['diagnosis']['historical_returncode'] == 104
    assert review['firmware']['diagnosis']['internal_codes_mapped_to_104'] == [120003, 120004]
    assert review['firmware']['diagnosis']['timeout_us'] == 3_000_000
    assert review['firmware']['pkg_version_retry']['code'] == 3203
    assert review['firmware']['selected_manual_branch'] is None
    assert all(b['device_applicable'] is None for b in review['firmware']['manual_branches'])
    assert {m['name'] for m in review['firmware']['queried_modules']} == {
        'master_service_pc4', 'unitree_patch_pc4', 'video_hub_pc4'}
    assert review['motion_writes'] == review['motor_configuration_values_obtained'] == 0
    assert not review['current'] and not review['execution_allowed']
    assert len(review['modes']) == 9
    assert all(m['device_applicable'] is None and not m['execution_allowed'] for m in review['modes'])
    assert all(v is None for m in KNOWLEDGE['motors'] for v in m['actual_config'].values())


def test_replaced_query_result_is_rejected_by_foundation(tmp_path, monkeypatch):
    asset = tmp_path/'g1_console/assets/g1_motion_control_review.json'
    asset.parent.mkdir(parents=True)
    evidence = tmp_path/'result.json'
    evidence.write_text('{"returncode":0}')
    review = json.loads((builder.ROOT/'g1_console/assets/g1_motion_control_review.json').read_text())
    review['sources'] = [dict(path='result.json', sha256=hashlib.sha256(b'{"returncode":104}').hexdigest())]
    asset.write_text(json.dumps(review))
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    with pytest.raises(AssertionError, match='result.json'):
        builder.motion_control_review()


def test_received_explicit_service_version_is_archived_without_granting_firmware_or_motion_authority():
    review = builder.motion_control_review()
    ad = review['service_advertisement']
    assert [r for r in ad['rows'] if r['name'] == 'ai_sport'] == [
        dict(name='ai_sport', version='8.7.5.212', path='$[0]', binding='object_fields')]
    assert ad['manual_branch_candidate'] == '23_gt_8_6'
    assert ad['manual_branch_selected'] is review['firmware']['selected_manual_branch'] is None
    assert ad['device_motion_firmware'] is review['device_motion_firmware'] is None
    assert not any(ad[k] for k in ('current', 'execution_allowed', 'running_build_verified',
                                  'physical_publisher_verified', 'raw_frame_retained'))
    assert ad['motion_writes'] == ad['lease_requests'] == 0
    assert all(m['device_applicable'] is None and not m['execution_allowed'] for m in review['modes'])


def test_app_software_and_imu_snapshot_do_not_fill_motion_calibration_or_support():
    built = builder.build()
    review = built['motion_control_review']
    app = review['app_observation']
    assert built['identity']['app_device_software_version'] == 'v1.5.6'
    assert built['identity']['app_reported_model'] == 'G1 EDU+'
    assert built['identity']['firmware_version'] is review['device_motion_firmware'] is None
    assert review['firmware']['selected_manual_branch'] is None
    assert built['robot'] == 'G1 EDU 23 DOF' and not built['hands']
    assert not app['current'] and app['captured_at_utc'] is None
    assert [i['temperature_c'] for i in app['imu_snapshot']] == [80, 79]
    assert app['motor_snapshot']['all_23_motors_covered'] is None
    assert app['motor_snapshot']['calibration_values'] is None
    assert all(v is None for m in built['motors'] for v in m['actual_config'].values())
    assert built['state_safety']['verified_motor_safety_limits'] is None
    assert built['state_safety']['validated_action_envelopes'] == []
    assert not app['execution_allowed'] and not review['execution_allowed']


def test_modified_app_evidence_is_rejected_before_entering_foundation(tmp_path, monkeypatch):
    asset = tmp_path/'g1_console/assets/g1_motion_control_review.json'
    asset.parent.mkdir(parents=True)
    review = json.loads((builder.ROOT/'g1_console/assets/g1_motion_control_review.json').read_text())
    original = (builder.ROOT/review['app_evidence_source']).read_bytes()
    review['app_evidence_source'] = 'app.json'
    review['sources'] = [dict(path='app.json', sha256=hashlib.sha256(original).hexdigest())]
    tampered = json.loads(original)
    tampered['device']['motion_firmware'] = '8.6.0.1'
    (tmp_path/'app.json').write_text(json.dumps(tampered))
    asset.write_text(json.dumps(review))
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    with pytest.raises(AssertionError, match='app.json'):
        builder.motion_control_review()


def test_verified_manual_and_single_damping_result_keep_rising_conditions_unknown():
    built = builder.build()
    review = built['motion_control_review']
    manual = review['official_manual']
    result = review['motor_test_contract']['device_result']
    assert manual['edition'] == '2026-04-07' and manual['branch_selected'] is None
    seat_route = next(r for r in manual['routes'] if r['start'].startswith('座椅'))
    assert '人工扶直站立' in seat_route['condition'] and '靠柜坐地' in seat_route['condition']
    assert result['motion_writes'] == review['motor_test_contract']['max_attempts'] == 1
    assert result['sdk_code'] == 3104 and result['observed_attempt']['status'] == 'outcome_unknown'
    assert result['dual_readback']['fsm_id']['raw']['data'] == 1
    assert result['dual_readback']['sport']['raw']['fsm_id'] == 1
    assert [r['fsm_id'] for r in result['transitions']] == [0, 1]
    for rows in (result['queries_before'], result['queries_after']):
        assert all(r['result']['error']['sdk_code'] == 7301 for r in rows
                   if r['operation'] in ('get_balance', 'get_stand_height'))
    assert not any(result[k] for k in ('current', 'execution_allowed', 'support_verified', 'motion_authority_verified'))
    assert result['device_motion_firmware'] is result['exact_7301_predicate'] is None
    assert built['state_safety']['validated_action_envelopes'] == []
    assert all(v is None for m in built['motors'] for v in m['actual_config'].values())


def test_full_trace_reproduces_reported_joint_motion_instead_of_only_initial_quiet_window():
    result = builder.motion_control_review()['motor_test_contract']['device_result']
    trace = result['trace']
    raw = gzip.decompress((builder.ROOT/trace['compressed_path']).read_bytes())
    assert hashlib.sha256(raw).hexdigest() == trace['raw_sha256']
    rows = [json.loads(line) for line in raw.splitlines()]
    initial = {j['index']: j['q'] for j in rows[0]['joints']}
    assert len(rows) == trace['samples'] == 792
    assert rows[-1]['at']-rows[0]['at'] == trace['duration_s']
    assert max(abs(j['dq']) for r in rows for j in r['joints']) == trace['max_abs_dq_rad_s']
    assert max(abs(j['q']-initial[j['index']]) for r in rows for j in r['joints']) == trace['max_abs_q_delta_rad']
    assert trace['max_abs_dq_rad_s'] > 1 and trace['max_abs_q_delta_rad'] > .4


def test_modified_damping_archive_rejected_before_entering_foundation(monkeypatch):
    source = builder.ROOT/'doc/evidence/damping_test_20261022/observed_attempt.json'
    original = type(source).read_bytes
    monkeypatch.setattr(type(source), 'read_bytes', lambda p: b'{}' if p == source else original(p))
    with pytest.raises(AssertionError, match='observed_attempt.json'):
        builder.motion_control_review()
