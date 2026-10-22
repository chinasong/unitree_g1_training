"""Offline format verification cannot promote manifests into device motor facts."""
from copy import deepcopy
import hashlib
import json

import pytest

from g1_console.body import KNOWLEDGE
from scripts import analyze_g1_opaque_configs as analyzer
from scripts import build_g1_capability_knowledge as builder


def inventory():
    return dict(baseInfo={'devId': 'private-device-identity'}, uuid='private-uuid-value',
        platform={'mqttServerPassword': 'private-mqtt-password'},
        items=dict(bins=[dict(name='daemon', version='1.0', filePath='private/path')],
            deps=[], libs=[], weights=[], plugins=[dict(name='plugin', version='2.0',
                filePath='/private/g1edu_motion.zip', secret='private-plugin-secret', uid='private-plugin-uid')]))


def test_only_inventory_allowlist_can_leave_offline_decoder():
    result = analyzer.safe_inventory(inventory())
    encoded = json.dumps(result)
    for secret in ('private-device-identity', 'private-uuid-value', 'private-mqtt-password',
                   'private-plugin-secret', 'private-plugin-uid', 'private/path'):
        assert secret not in encoded
    assert result['declared_plugins'] == [dict(name='plugin', version='2.0', filename='g1edu_motion.zip',
        installed_verified=False, loaded_verified=False, execution_allowed=False)]
    assert result['motor_field_name_matches'] == 0 and result['private_sections_omitted']
    changed = inventory()
    changed['items']['plugins'].append({'calibration': 'private-value'})
    assert analyzer.schema(changed)['items']['plugins'] != analyzer.schema(inventory())['items']['plugins']
    changed = inventory(); changed['motor_config'] = {'kp': 99}
    with pytest.raises(ValueError, match='schema'):
        analyzer.safe_inventory(changed)


def test_invalid_padding_and_trailing_json_are_rejected():
    data = b'{"valid":true}'
    padding = 16 - len(data) % 16
    padded = data + bytes([padding]) * padding
    assert analyzer.parse_padded_json(padded) == ({'valid': True}, data)
    with pytest.raises(ValueError, match='padding'):
        analyzer.parse_padded_json(padded[:-1] + b'\0')
    trailing = b'{}private-extra'; padding = 16-len(trailing)%16
    with pytest.raises(ValueError, match='complete JSON'):
        analyzer.parse_padded_json(trailing + bytes([padding])*padding)


def test_replaced_binary_or_configuration_is_rejected_before_decode(tmp_path):
    (tmp_path/'config.bin').write_bytes(b'0'*analyzer.FILES['config.bin'][0])
    with pytest.raises(ValueError, match='fingerprint mismatch'):
        analyzer.verified_file(tmp_path, 'config.bin')


def test_changed_analysis_receipt_is_rejected(tmp_path, monkeypatch):
    # Keep prior provenance valid so the failure proves the new analysis is checked.
    a = KNOWLEDGE['service_audit']
    paths = [s['path'] for receipt in (a['receipt'], a['configuration_analysis']['receipt'])
             for s in receipt['sources']+receipt['scripts']]
    paths += ['g1_console/assets/service_audit_sources/receipt_20261021.json',
              'g1_console/assets/service_audit_sources/configuration_receipt_20261021.json']
    for name in paths:
        dest = tmp_path/name; dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes((builder.ROOT/name).read_bytes())
    altered = tmp_path/a['configuration_analysis']['source']['path']
    altered.write_text('{}')
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    with pytest.raises(AssertionError, match='configuration_analysis_20261021.json'):
        builder.service_audit()


def test_persisted_result_preserves_fmx_unknown_and_no_motor_values():
    a = KNOWLEDGE['service_audit']['configuration_analysis']
    assert a['current'] is False and a['offline_only'] is True
    assert a['megaflow']['plaintext_bytes'] == 14628
    assert a['megaflow']['category_counts'] == dict(bins=5, deps=85, libs=24, plugins=2, weights=0)
    assert a['megaflow']['motor_field_name_matches'] == 0
    assert a['master']['payload_decoded'] is False and a['master']['motor_configuration_present'] is None
    assert all(a[k] == 0 for k in ('rpc_calls','command_writes','configuration_calls','calibration_calls','motor_configuration_values_obtained'))
    assert all(v is None for m in KNOWLEDGE['motors'] for v in m['actual_config'].values())
    for s in a['receipt']['sources'] + a['receipt']['scripts']:
        assert hashlib.sha256((builder.ROOT/s['path']).read_bytes()).hexdigest() == s['sha256']


def test_exact_private_material_reproduces_sanitized_evidence_when_available():
    folder = builder.ROOT/'.g1-deploy/research/opaque-20261021'
    private = folder/'network-addresses.private.json'
    if not private.exists():
        pytest.skip('private Orin material is intentionally excluded from source and deployment')
    pytest.importorskip('elftools'); pytest.importorskip('Crypto')
    result = analyzer.analyze(folder, json.loads(private.read_text()))
    saved = deepcopy(KNOWLEDGE['service_audit']['configuration_analysis'])
    for k in ('received_at_utc','analyzed_at_utc','provenance','source','receipt'):
        saved.pop(k)
    assert result == saved
    # Also prove actual platform/plugin secrets were not exported (without printing them).
    raw = json.loads((folder/'megaflow-decoded.private.json').read_text())
    exported = json.dumps(KNOWLEDGE, ensure_ascii=False)
    secrets = [raw['platform'][k] for k in raw['platform'] if 'password' in k.lower()]
    secrets += [p['secret'] for p in raw['items']['plugins']]
    assert secrets and all(isinstance(v, str) and len(v) > 5 and v not in exported for v in secrets)
    assert all(mac not in exported for mac in json.loads(private.read_text()).values())
