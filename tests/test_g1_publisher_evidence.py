"""Only paired captures establish provenance; they never establish authority."""
import copy
import json

import pytest

from scripts import build_g1_motor_knowledge as builder
from scripts.g1_publisher_evidence import publisher_review


def captured():
    path = builder.ROOT/'g1_console/assets/safety_evidence_sources/service_publisher_20261022_v33.json'
    return json.loads(path.read_text())


def test_actual_publication_handles_separate_version_publisher_from_motion_responder():
    review = publisher_review(captured())
    ad = review['advertisements'][0]
    assert ad['self_reported']['__ProcessName'] == 'robot_state_service'
    assert next(r['version'] for r in ad['rows'] if r['name'] == 'ai_sport') == '8.7.5.212'
    assert len(review['queries']) == 10
    assert {q['code'] for q in review['queries'] if q['api_id'] in (7003, 7005)} == {7301}
    assert all(q['self_reported']['__ProcessName'] == 'humanoid'
               for q in review['queries'] if q['api_id'] in (7003, 7005))
    assert review['states'][0]['samples'] == 4930
    assert review['states'][0]['self_reported']['__Pid'] == '3359'
    assert not review['current'] and not review['execution_allowed'] and not review['motion_authority_verified']
    assert review['device_motion_firmware'] is review['exact_7301_predicate'] is None
    assert builder.motion_control_review()['publisher_review'] == review


@pytest.mark.parametrize('mutation', ['handle', 'duplicate', 'sender', 'participant', 'lease', 'authority', 'error_data'])
def test_modified_binding_or_authority_is_rejected(mutation):
    raw = copy.deepcopy(captured())
    frame = raw['frames'][0]
    if mutation == 'handle':
        frame['publication_handle'] += 1
    elif mutation == 'duplicate':
        frame['matching_publications'] *= 2
    elif mutation == 'sender':
        for sender in raw['passive_rtps']['senders']:
            sender['source_ip'] = '192.168.123.164'
    elif mutation == 'participant':
        raw['participants'] = []
    elif mutation == 'lease':
        raw['queries'][0]['lease_id'] = 123
    elif mutation == 'authority':
        raw['motion_authority_verified'] = True
    else:
        raw['queries'][2]['replies'][0]['data'] = '{"data":0}'
    with pytest.raises(AssertionError):
        publisher_review(raw)
