"""Declared service versions and failed searches never become motor parameters."""
from copy import deepcopy
import hashlib
import json

import pytest

from g1_console.body import KNOWLEDGE
from scripts import analyze_g1_fmx_loader_search as review
from scripts import build_g1_capability_knowledge as builder


def test_module_versions_are_scoped_and_cached_patch_is_not_current():
    a = KNOWLEDGE['service_audit']['fmx_loader_review']
    assert a['current'] is False and a['offline_only'] is True
    assert a['module_identity']['modules'][0]['declared_version'] == '1.0.0.2'
    assert not a['module_identity']['file_list_proves_content_hash']
    assert not a['module_identity']['all_manifest_files_present_verified']
    assert a['module_identity']['manifest_file_recheck']['files'][0]['status'] == 'FileNotFoundError'
    assert all(not m['source_commit_reproduced'] and not m['motor_firmware_version_verified']
               for m in a['module_identity']['modules'])
    assert KNOWLEDGE['identity']['firmware_version'] is a['motor_firmware_version'] is None
    patch = a['cached_packages'][0]
    assert patch['cached_path_version'] == '1.0.0.2' and patch['installed_manifest_version'] == '1.0.0.6'
    assert not patch['path_version_matches_installed']
    assert all(not p['payload_decoded'] and not p['fmx_algorithm_equivalence_verified'] for p in a['cached_packages'])
    assert a['local_search']['unique_elfs'] == 62 and a['local_search']['static_archives'] == 10
    assert a['local_search']['other_ar_containers'] == 1
    assert a['local_search']['mixer_symbol_matches'] == []
    assert a['local_search']['without_defined_symbols'] == 18  # Includes one Debian ar container.
    assert any(e['error'] == 'PermissionError' for e in a['local_search']['errors'])
    assert not a['payload_decoded'] and a['motor_configuration_present'] is None
    assert all(a[k] == 0 for k in ('motor_configuration_values_obtained', 'command_writes', 'rpc_calls',
                                   'configuration_calls', 'calibration_calls', 'vendor_program_executions'))
    assert sum(v is None for m in KNOWLEDGE['motors'] for v in m['actual_config'].values()) == 253


def test_changed_loader_inventory_is_rejected_by_foundation(tmp_path, monkeypatch):
    base = 'g1_console/assets/service_audit_sources/'
    for name in ['receipt_20261021.json', 'configuration_receipt_20261021.json',
                 'fmx_receipt_20261021.json', 'fmx_loader_receipt_20261021.json']:
        path = base+name
        receipt = json.loads((builder.ROOT/path).read_text())
        for item in [{'path':path}] + receipt['sources'] + receipt['scripts']:
            dest = tmp_path/item['path']; dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes((builder.ROOT/item['path']).read_bytes())
    changed = base+'fmx_loader_inventory_20261021.json'
    (tmp_path/changed).write_text('{}')
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    with pytest.raises(AssertionError, match='fmx_loader_inventory_20261021.json'):
        builder.service_audit()


def test_exact_copies_reproduce_sanitized_review_and_fail_on_changed_input(tmp_path):
    folder = builder.ROOT/'.g1-deploy/research/fmx-loader-followup-20261021'
    if not (folder/'master_module.private.json').exists():
        pytest.skip('private vendor copies intentionally excluded')
    pytest.importorskip('elftools')
    inventory = review.ASSETS/'fmx_loader_inventory_20261021.json'
    search = review.ASSETS/'fmx_official_code_search_20261021.json'
    result = review.analyze(folder, builder.ROOT/'.g1-deploy/research/opaque-20261021', inventory, search)
    saved = deepcopy(KNOWLEDGE['service_audit']['fmx_loader_review'])
    saved.pop('receipt'); saved.pop('source')
    assert result == saved
    assert len(result['ota_package_entries']) == 3
    assert all(not s['file_backed'] for s in result['ota_package_entries'])
    public = json.dumps(result, ensure_ascii=False)
    for name in ('master', 'patch', 'video'):
        raw = json.loads((folder/(name+'_module.private.json')).read_text())
        for field in ('Install', 'Remove'):
            for value in raw[field].values():
                if isinstance(value, str) and value:
                    assert value not in public
    for record in KNOWLEDGE['service_audit']['fmx_loader_review']['receipt']['sources']:
        assert hashlib.sha256((builder.ROOT/record['path']).read_bytes()).hexdigest() == record['sha256']
    altered = tmp_path/'inventory.json'; altered.write_text('{}')
    with pytest.raises(ValueError, match='fingerprint mismatch'):
        review.analyze(folder, folder, altered, search)
