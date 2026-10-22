"""Large generic libraries and negative searches cannot supply motor parameters."""
from copy import deepcopy
import json

import pytest

from g1_console.body import KNOWLEDGE
from scripts import analyze_g1_large_libraries as review
from scripts import build_g1_capability_knowledge as builder


def test_generic_crypto_and_camera_library_are_not_motor_configuration():
    a = KNOWLEDGE['service_audit']['large_library_review']
    assert not a['current'] and a['offline_only'] and a['read_only']
    assert not a['payload_decoded'] and a['motor_configuration_present'] is None
    assert all(a[k] == 0 for k in ('motor_configuration_values_obtained', 'command_writes', 'rpc_calls',
        'configuration_calls', 'calibration_calls', 'vendor_program_executions', 'runtime_memory_reads'))
    crypto, camera = a['files']
    assert crypto['archive_members'] == 168 and crypto['symbols']['generic_algorithm_line_counts']['Blowfish'] == 31
    assert crypto['symbols']['line_count'] == 28215
    assert camera['symbols']['line_count'] == 125567 and camera['dynamic_symbols']['line_count'] == 84621
    assert camera['has_symtab'] and camera['has_debug_info']
    assert all(not f['implementation_match_confirmed'] and not f['fmx_loader_found'] for f in a['files'])
    assert all(f['unchanged_during_read'] and f['sha256'] == f['sha256_after'] for f in a['files'])
    linkage = a['master_linkage']
    assert not linkage['cryptopp_or_realsense_direct_needed'] and not linkage['static_or_runtime_linkage_excluded']
    assert linkage['named_cryptopp_symbols'] == linkage['named_realsense_symbols'] == 0
    assert [e['name'] for e in linkage['positive_control_loader_entries']] == ['Mixer::Gen', 'Mixer::LoadFile']
    assert all(not e['file_backed'] for e in linkage['positive_control_loader_entries'])
    values = [v for motor in KNOWLEDGE['motors'] for v in motor['actual_config'].values()]
    assert len(values) == 253 and all(v is None for v in values)


def test_changed_library_inventory_is_rejected_before_foundation_build(tmp_path, monkeypatch):
    base = 'g1_console/assets/service_audit_sources/'
    for name in ['receipt_20261021.json', 'configuration_receipt_20261021.json',
                 'fmx_receipt_20261021.json', 'fmx_loader_receipt_20261021.json',
                 'large_library_receipt_20261021.json']:
        path = base+name
        receipt = json.loads((builder.ROOT/path).read_text())
        for item in [{'path':path}]+receipt['sources']+receipt['scripts']:
            dest = tmp_path/item['path']; dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes((builder.ROOT/item['path']).read_bytes())
    changed = base+'large_library_inventory_20261021.json'
    tampered = json.loads((tmp_path/changed).read_text())
    tampered['files'][0]['symbols']['matches'] = ['unitree::security::Mixer::Gen']
    (tmp_path/changed).write_text(json.dumps(tampered))
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    with pytest.raises(AssertionError, match='large_library_inventory_20261021.json'):
        builder.service_audit()


def test_offline_review_reproduces_and_rejects_changed_real_file(tmp_path):
    folder = builder.ROOT/'.g1-deploy/research/opaque-20261021'
    if not (folder/'master_service').exists():
        pytest.skip('private vendor copies intentionally excluded')
    pytest.importorskip('elftools')
    result = review.analyze(folder)
    saved = deepcopy(KNOWLEDGE['service_audit']['large_library_review'])
    saved.pop('receipt'); saved.pop('source')
    assert result == saved
    for name in ('master_service', 'master_service.json'):
        (tmp_path/name).write_bytes((folder/name).read_bytes())
    altered = tmp_path/'master_service.json'
    altered.write_bytes(altered.read_bytes()[:-1]+b'X')
    with pytest.raises(ValueError, match='fingerprint mismatch'):
        review.analyze(tmp_path)
