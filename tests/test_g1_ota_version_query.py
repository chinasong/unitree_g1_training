"""Validate the actual audit command boundary without invoking vendor code."""
import hashlib
import json
from types import SimpleNamespace

import pytest

from scripts import audit_g1_ota_versions as audit


def setup_files(tmp_path, monkeypatch):
    binary = tmp_path/'ota_pipe_cli'
    binary.write_bytes(b'pinned test binary')
    monkeypatch.setattr(audit, 'BINARY_SHA256', hashlib.sha256(binary.read_bytes()).hexdigest())
    hosts = tmp_path/'hosts.json'
    hosts.write_text(json.dumps(audit.EXPECTED_HOSTS))
    return binary, hosts


def test_query_has_three_second_budget_and_only_list_command(tmp_path, monkeypatch):
    binary, hosts = setup_files(tmp_path, monkeypatch)
    calls = []
    def run(args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(returncode=0, stdout='host alias:pc4, module count:3', stderr='')
    result = audit.query(binary, hosts, run)
    assert calls == [([str(binary), '-a', 'default', '-c', 'listmoduleversion', '-T', '3000000'],
                      dict(cwd='/tmp', capture_output=True, text=True, timeout=12))]
    assert result['returncode'] == 0
    assert result['timeout_unit'] == 'microseconds'
    assert result['motion_writes'] == result['ota_mutations'] == 0
    assert 'motion-host firmware remains unverified' in result['version_result_authority']


@pytest.mark.parametrize('changed', ['binary', 'endpoint'])
def test_identity_change_blocks_vendor_execution(tmp_path, monkeypatch, changed):
    binary, hosts = setup_files(tmp_path, monkeypatch)
    if changed == 'binary':
        binary.write_bytes(b'unexpected binary')
    else:
        hosts.write_text(json.dumps({'default': 'tcp://192.168.123.161:1026'}))
    calls = []
    with pytest.raises(AssertionError):
        audit.query(binary, hosts, lambda *args, **kwargs: calls.append(args))
    assert calls == []
