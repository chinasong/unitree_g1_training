"""Cross-channel state reception must not become motion or version authority."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from scripts.build_g1_motor_knowledge import build
from scripts.g1_humanoid_evidence import humanoid_review

ROOT = Path(__file__).resolve().parents[1]


def evidence():
    base = ROOT/'g1_console/assets/safety_evidence_sources'
    return (json.loads((base/'humanoid_state_20261021_v30.json').read_text()),
            json.loads((base/'humanoid_files_20261021_v30.json').read_text()))


def test_publishing_hg_state_and_7301_coexist_without_proving_balance_or_firmware():
    raw, files = evidence()
    result = humanoid_review(raw, files)
    assert [c['samples'] for c in result['channels']] == [1559, 240]
    assert len({p['key'] for p in result['participants']}) == 2
    assert all(p['self_reported']['__Pid'] == '3396' for p in result['participants'])
    assert result['device_motion_firmware'] is result['exact_7301_predicate'] is result['balance_ready'] is None
    assert result['foot_load_n'] is result['cabinet_load_n'] is None
    assert not result['current'] and not result['executable_verified'] and not result['execution_allowed']
    foundation = build()
    assert foundation['motion_control_review']['humanoid_review'] == result
    assert foundation['motion_control_review']['firmware']['selected_manual_branch'] is None
    assert all(not mode['execution_allowed'] for mode in foundation['motion_control_review']['modes'])
    assert all(v is None for m in foundation['motors'] for v in m['actual_config'].values())


@pytest.mark.parametrize('bad', ['handle', 'type', 'sender', 'endpoint', 'pid', 'receipt', 'error_payload', 'nan', 'extra_rpc'])
def test_unpaired_or_invalid_evidence_cannot_enter_the_foundation(bad):
    raw, files = evidence()
    raw = deepcopy(raw)
    stream = raw['hg_state']['streams'][0]
    if bad == 'handle':
        stream['publication_handle'] += 1
    elif bad == 'type':
        stream['matching_publications'][0]['type'] = 'unitree_go::msg::dds_::SportModeState_'
    elif bad == 'sender':
        raw['passive_rtps']['senders'][0]['source_ip'] = '192.168.123.164'
    elif bad == 'endpoint':
        key = stream['matching_publications'][0]['participant_key']
        next(p for p in raw['other_node161_participants'] if p['key'] == key)['publications'] = []
    elif bad == 'pid':
        key = stream['matching_publications'][0]['participant_key']
        peer = next(p for p in raw['other_node161_participants'] if p['key'] == key)
        next(p for p in peer['properties'] if p['name'] == '__Pid')['value'] = '9999'
    elif bad == 'receipt':
        stream['last_receipt_s'] = 1
    elif bad == 'error_payload':
        raw['queries'][1]['replies'][0]['raw'] = '{"data":0}'
    elif bad == 'nan':
        stream['distinct_values'][0]['task_time'] = float('nan')
    elif bad == 'extra_rpc':
        raw['queries'].append(deepcopy(raw['queries'][0]))
    with pytest.raises((AssertionError, KeyError)):
        humanoid_review(raw, files)


def test_discovery_names_do_not_create_new_callable_remote_controls():
    from g1_console.capabilities import CATALOG
    result = humanoid_review(*evidence())
    assert len(CATALOG) == 61
    names = {row['name']: row for row in result['read_routes']}
    assert names['GetMimicMotion']['status'] == '条件未满足 · 未调用'
    assert names['basic_softlimit / basic_clearoip']['status'] == '只读契约未知 · 未调用'
    assert all('basic_' not in str(row) and 'GetMimicMotion' not in str(row) for row in CATALOG)
