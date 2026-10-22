"""FMX hypotheses and container facts never become actuator configuration."""
from copy import deepcopy
import hashlib
import json

import pytest

from g1_console.body import KNOWLEDGE
from scripts import analyze_g1_fmx as fmx
from scripts import build_g1_capability_knowledge as builder


def test_trampoline_rejects_arbitrary_or_modified_code():
    with pytest.raises(ValueError, match='trampoline'):
        fmx.stub_target(bytes(32), 0x4beb80)
    # A BR x17 must not be interpreted as the verified BR x16 pattern.
    words = [0xa9bf7bfd, 0xa9bf07e0, 0x94000000, 0xa8c107e0,
             0xa8c17bfd, 0x90000010, 0x91000210, 0xd61f0220]
    import struct
    with pytest.raises(ValueError, match='trampoline'):
        fmx.stub_target(struct.pack('<8I', *words), 0x4beb80)


def test_structure_does_not_assert_a_header_or_seed():
    a = KNOWLEDGE['service_audit']['fmx_analysis']
    assert a['structure']['file_bytes'] == 468
    assert a['structure']['offset8_u32_le'] == a['structure']['bytes_after_20'] == 448
    assert a['structure']['block8_remainder_after_32'] == 4
    assert a['structure']['header_size_verified'] is None
    assert not a['structure']['seed_semantics_verified']
    assert {r['name'] for r in a['loader_stubs']} == set(fmx.STUBS)
    assert all(not r['file_backed'] and r['section_type'] == 'SHT_NOBITS' for r in a['loader_stubs'])


def test_followup_receipts_and_failed_hypotheses_cannot_supply_motor_values():
    a = KNOWLEDGE['service_audit']['fmx_analysis']
    assert a['current'] is False and a['offline_only'] is True
    assert a['payload_decoded'] is False and a['motor_configuration_present'] is None
    h = a['research_hypotheses']
    assert h['hypotheses_tested'] == 16 and h['complete_json_count'] == 0
    assert not h['applicable_to_installed_loader_verified'] and not h['outer_layer_decode_verified']
    assert a['local_loader_search']['elf_count'] == 7
    assert all(a[k] == 0 for k in ('motor_configuration_values_obtained', 'command_writes',
        'rpc_calls', 'configuration_calls', 'calibration_calls', 'vendor_program_executions'))
    assert sum(v is None for m in KNOWLEDGE['motors'] for v in m['actual_config'].values()) == 253
    for record in a['receipt']['sources'] + a['receipt']['scripts']:
        assert hashlib.sha256((builder.ROOT/record['path']).read_bytes()).hexdigest() == record['sha256']
    original = KNOWLEDGE['service_audit']['processes']
    assert a['robot_recheck']['processes'] == [{k:p[k] for k in ('pid','argv','start_ticks')} for p in original]


def test_changed_followup_evidence_is_rejected(tmp_path, monkeypatch):
    a = KNOWLEDGE['service_audit']
    receipts = ['receipt_20261021.json', 'configuration_receipt_20261021.json', 'fmx_receipt_20261021.json']
    for filename in receipts:
        path = 'g1_console/assets/service_audit_sources/' + filename
        receipt = json.loads((builder.ROOT/path).read_text())
        for item in [{'path':path}] + receipt['sources'] + receipt['scripts']:
            dest = tmp_path/item['path']; dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes((builder.ROOT/item['path']).read_bytes())
    (tmp_path/a['fmx_analysis']['source']['path']).write_text('{}')
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    with pytest.raises(AssertionError, match='fmx_analysis_20261021.json'):
        builder.service_audit()


def test_exact_material_reproduces_followup_without_private_output():
    folder = builder.ROOT/'.g1-deploy/research/opaque-20261021'
    private = builder.ROOT/'.g1-deploy/research/fmx-20261021'
    key = private/'research-key.private.bin'
    if not key.exists():
        pytest.skip('offline vendor copies and candidate material intentionally excluded')
    pytest.importorskip('elftools'); pytest.importorskip('Crypto')
    result = fmx.analyze(folder, private/'remote_inventory.json', key)
    saved = deepcopy(KNOWLEDGE['service_audit']['fmx_analysis'])
    for name in ('analyzed_at_utc','source','receipt'):
        saved.pop(name)
    assert result == saved
    output = json.dumps(result, ensure_ascii=False)
    assert key.read_bytes().hex() not in output
    assert str(int.from_bytes((folder/'master_service.json').read_bytes()[4:8], 'little')) not in output
    # Corrupted input must stop before interpreting any official file or candidate.
    with pytest.raises(ValueError, match='inventory fingerprint'):
        fmx.analyze(folder, key, key)
