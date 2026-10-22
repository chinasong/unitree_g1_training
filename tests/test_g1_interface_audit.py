"""Validate actual RPC envelopes against pinned official getters and evidence integrity."""
import ast
import json
import shutil

import pytest

from g1_console.body import KNOWLEDGE
from scripts import audit_g1_interfaces as audit
from scripts import build_g1_motor_knowledge as builder


def constants(path):
    tree = ast.parse(path.read_text())
    return {node.targets[0].id: ast.literal_eval(node.value) for node in tree.body
            if isinstance(node, ast.Assign)}


def test_only_documented_read_envelopes_are_sent_without_lease():
    source = builder.ROOT/'g1_console/assets/interface_audit_sources/unitree_sdk2_python/unitree_sdk2py'
    api = constants(source/'b2/robot_state/robot_state_api.py')
    internal = constants(source/'rpc/internal.py')
    calls, leases, registered, versions = [], [], [], []

    class ProtocolClient:
        def __init__(self, service, lease):
            self.service = service
            leases.append(lease)

        def SetTimeout(self, timeout):
            assert timeout == 2

        def GetServerApiVersion(self):
            calls.append((self.service, internal['RPC_API_ID_INTERNAL_API_VERSION'], '{}'))
            return 0, '1.0.0.1'

        def _SetApiVerson(self, version):
            versions.append(version)

        def _RegistApi(self, method, priority):
            registered.append((method, priority))

        def _Call(self, method, payload):
            calls.append((self.service, method, payload))
            if method == api['ROBOT_STATE_API_ID_SERVICE_LIST']:
                return 0, '[{"name":"robot_state","status":0,"protect":1}]'
            assert method == api['ROBOT_STATE_API_ID_GET_PKG_VERSION']
            return internal['RPC_ERR_SERVER_API_NOT_IMPL'], ''

    result = audit.inspect(ProtocolClient, audit.state_client(ProtocolClient), settle=lambda _: None)
    assert len(calls) == result['rpc_calls'] == 8
    assert all(lease is False for lease in leases)
    assert calls == [(s, 1, '{}') for s in audit.SERVICES] + [
        ('robot_state', api['ROBOT_STATE_API_ID_SERVICE_LIST'], '{}'),
        ('robot_state', api['ROBOT_STATE_API_ID_GET_PKG_VERSION'], '{}')]
    assert registered == [(1003, 0), (1006, 0)]
    assert versions == [api['ROBOT_STATE_API_VERSION']]
    assert result['robot_state']['packages']['package_version'] is None
    assert result['robot_state']['packages']['code'] == 3203
    assert result['configuration_calls'] == result['calibration_calls'] == result['lease_requests'] == 0


def test_live_service_metadata_cannot_become_motor_configuration():
    record = KNOWLEDGE['interface_audit']
    assert record['current'] is False
    assert record['motor_configuration_read_supported'] is record['calibration_read_supported'] is None
    assert record['motor_configuration_values_obtained'] == 0
    query = record['rpc_metadata']
    assert len(query['versions']) == 6 and all(v['code'] == 0 for v in query['versions'])
    assert not any(v['physical_host_verified'] for v in query['versions'])
    assert len(query['robot_state']['services']['entries']) == 28
    assert query['robot_state']['packages']['code'] == 3203
    actual = [v for m in KNOWLEDGE['motors'] for v in m['actual_config'].values()]
    assert len(actual) == 253 and all(v is None for v in actual)
    assert KNOWLEDGE['identity']['firmware_version'] is None


@pytest.mark.parametrize('target', ['source', 'query', 'inventory'])
def test_changed_protocol_or_device_evidence_is_rejected(tmp_path, monkeypatch, target):
    asset = 'g1_console/assets/g1_interface_parameter_audit.json'
    record = json.loads((builder.ROOT/asset).read_text())
    files = record['source_files'] + [record['query_source']] + [r['inventory'] for r in record['repositories']]
    for path in [asset] + [f['path'] for f in files]:
        destination = tmp_path/path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(builder.ROOT/path, destination)
    replaced = {'source': record['source_files'][0], 'query': record['query_source'],
                'inventory': record['repositories'][0]['inventory']}[target]
    (tmp_path/replaced['path']).write_text('{}')
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    with pytest.raises(AssertionError, match=replaced['path']):
        builder.interface_audit()
