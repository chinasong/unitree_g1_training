"""Test real non-zero attribution and failure paths, without loading DDS."""
from copy import deepcopy
import json

import pytest

from scripts.audit_g1_sport_host import discovery_metadata, rtps_header
from scripts.build_g1_motor_knowledge import build, ROOT
from scripts.g1_sport_evidence import service_readiness


def raw_evidence():
    review = json.loads((ROOT/'g1_console/assets/g1_motion_control_review.json').read_text())
    return json.loads((ROOT/review['service_evidence_source']).read_text())


def test_rtps_header_recognizes_known_nonzero_guid_without_retaining_payload():
    prefix = bytes.fromhex('0110201814eee68280247c60')
    packet = b'RTPS'+bytes([2, 1, 1, 16])+prefix+b'private application payload'
    assert rtps_header(packet) == dict(guid_prefix=prefix.hex(), protocol=[2, 1], vendor_id='0110')
    for invalid in (packet[:19], b'NOPE'+packet[4:], b'RTPS'+bytes([1])+packet[5:], b''):
        assert rtps_header(invalid) is None


def test_real_network_attribution_and_service_on_do_not_unlock_motion():
    review = service_readiness(raw_evidence())
    assert review['observed_sender_ips'] == ['192.168.123.161']
    services = {r['name']: r for r in review['services']}
    assert len(services) == 28 and services['ai_sport']['status'] == services['state_estimator']['status'] == 0
    assert review['exact_7301_predicate'] is review['balance_ready'] is review['device_motion_firmware'] is None
    assert not review['current'] and not review['execution_allowed']
    assert len(review['queries']) == 10
    assert all(r['raw'] is None for r in review['queries'] if r['code'] != 0)
    foundation = build()
    assert foundation['motion_control_review']['service_readiness'] == review
    assert foundation['identity']['firmware_version'] is None
    assert all(not m['execution_allowed'] for m in foundation['motion_control_review']['modes'])
    assert all(v is None for m in foundation['motors'] for v in m['actual_config'].values())


def test_metadata_is_bound_to_actual_reply_and_does_not_become_firmware():
    review = service_readiness(raw_evidence())
    node = review['node_inventory']
    assert node['self_reported']['__ProcessName'] == 'humanoid'
    assert node['self_reported']['__Hostname'] == 'Unitree'
    assert '__NetworkAddresses' in node['property_names']
    assert '__NetworkAddresses' not in node['self_reported']
    assert not node['executable_verified'] and not node['version_interface_verified']
    assert node['component_version'] is review['device_motion_firmware'] is None
    assert node['peer_ota_queries'] == 0
    # Another advertised sport publisher is a bridge, not the matched responder.
    raw = raw_evidence()
    raw['response_participants'].reverse()
    assert service_readiness(raw)['node_inventory'] == node
    # Missing names remain unknown; the property order is not a source of defaults.
    participant = next(p for p in raw['response_participants'] if p['key'] == node['participant_key'])
    participant['properties'] = []
    missing = service_readiness(raw)
    assert missing['node_inventory']['self_reported'] == {}
    assert '进程 未知' in next(f['evidence'] for f in missing['findings'] if f['name'] == '应答参与者自报信息')
    assert missing['device_motion_firmware'] is None and not missing['execution_allowed']


@pytest.mark.parametrize('change', ['guid', 'duplicate', 'property_value', 'port_payload', 'peer_query', 'alias'])
def test_unbound_or_unsafe_node_inventory_is_rejected(change):
    raw = deepcopy(raw_evidence())
    key = raw['queries'][0]['replies'][0]['matching_publications'][0]['participant_key']
    participant = next(p for p in raw['response_participants'] if p['key'] == key)
    if change == 'guid':
        participant['key'] = '00000000-0000-0000-0000-000000000000'
    elif change == 'duplicate':
        raw['response_participants'].append(deepcopy(participant))
    elif change == 'property_value':
        participant['properties'].append(dict(name='access_token', value='must-not-be-retained'))
    elif change == 'port_payload':
        raw['known_read_paths']['known_ports'][0]['application_bytes_sent'] = 1
    elif change == 'peer_query':
        raw['known_read_paths']['peer_ota_queries'] = 1
    else:
        raw['known_read_paths']['ota_hosts']['default'] = 'tcp://192.168.123.161:1026'
    with pytest.raises(AssertionError):
        service_readiness(raw)


def test_discovery_metadata_retains_only_approved_values():
    # Match the policy interface without requiring CycloneDDS on the Mac.
    prop = type('Property', (), {})
    prop.__qualname__ = 'Policy.Property'
    policies = []
    for key, value in [('__ProcessName', 'humanoid'), ('access_token', 'private'), ('__Pid', 'x'*257)]:
        p = prop()
        p.key, p.value = key, value
        policies.append(p)
    metadata = discovery_metadata(policies)
    assert metadata['properties'] == [dict(name='__ProcessName', value='humanoid'),
                                       dict(name='access_token'), dict(name='__Pid')]
    assert 'private' not in json.dumps(metadata)


@pytest.mark.parametrize('change', ['ip', 'guid', 'handle', 'duplicate', 'placeholder', 'lease', 'firmware'])
def test_unpaired_sources_errors_and_control_activity_cannot_be_promoted(change):
    raw = deepcopy(raw_evidence())
    reply = raw['queries'][0]['replies'][0]
    if change == 'ip':
        reply['matching_publications'][0]['observed_rtps_source_ips'] = ['203.0.113.99']
    elif change == 'guid':
        reply['matching_publications'][0]['participant_key'] = '00000000-0000-0000-0000-000000000000'
    elif change == 'handle':
        reply['matching_publications'][0]['instance_handle'] += 1
    elif change == 'duplicate':
        raw['queries'][0]['replies'].append(deepcopy(reply))
    elif change == 'placeholder':
        raw['queries'][3]['replies'][0]['raw'] = '{"data":0}'
    elif change == 'lease':
        raw['lease_requests'] = 1
    else:
        raw['device_motion_firmware'] = '1.0.0.0'
    with pytest.raises(AssertionError):
        service_readiness(raw)
