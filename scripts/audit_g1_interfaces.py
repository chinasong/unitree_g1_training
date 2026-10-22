#!/usr/bin/env python3
"""One-shot G1 RPC metadata reads on Orin; never calls configuration or calibration APIs.

API 1 is the official SDK's internal GetServerApiVersion, not a service-specific
API ID. RobotStateClient is explicitly reused from B2 by the G1 documentation.
No parameter names, arbitrary API IDs, leases, shell RPC, or control calls accepted.
"""
from datetime import datetime, timezone
import json
import time
from types import SimpleNamespace


SERVICES = ('config', 'basic_demarcate', 'basic_softlimit', 'basic_taumax',
            'robot_type_service', 'robot_state')


def state_client(client_base):
    # The pinned B2 RobotStateClient imports a nonexistent rpc.client_internal.
    # Reproduce only its two documented getters; leave the vendor SDK unchanged.
    class MetadataClient(client_base):
        def __init__(self):
            super().__init__('robot_state', False)

        def Init(self):
            self._SetApiVerson('1.0.0.2')
            self._RegistApi(1003, 0)
            self._RegistApi(1006, 0)

        def ServiceList(self):
            code, data = self._Call(1003, '{}')
            if code:
                return code, None
            return code, [SimpleNamespace(name=row['name'], status=row['status'], protect=row['protect'])
                          for row in json.loads(data)]

        def GetPkgVersion(self):
            code, data = self._Call(1006, '{}')
            if code:
                return code, None, None
            row = json.loads(data)
            return code, row['packageVersion'], row['moduleVersionMap']
    return MetadataClient


def inspect(client_factory, state_factory, settle=time.sleep):
    result = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(), current=False,
        method='Official DDS/RPC metadata getters through Orin eth0; no SSH to PC1',
        command_writes=0, configuration_calls=0, calibration_calls=0, lease_requests=0,
        motor_configuration_values_obtained=0, versions=[], robot_state={})
    for service in SERVICES:
        client = client_factory(service, False)
        client.SetTimeout(2.0)
        settle(.5)
        code, version = client.GetServerApiVersion()
        result['versions'].append(dict(service=service, api_id=1, code=code,
            api_version=version if isinstance(version, str) and len(version) <= 128 else None,
            physical_host_verified=False,
            meaning='Service API version only; not motor firmware, configuration or calibration'))
        del client
    state = state_factory()
    state.SetTimeout(2.0)
    state.Init()
    settle(.5)
    code, services = state.ServiceList()
    result['robot_state']['services'] = dict(api_id=1003, code=code,
        entries=[dict(name=s.name, status=s.status, protect=s.protect) for s in services] if code == 0 else None)
    code, package, modules = state.GetPkgVersion()
    result['robot_state']['packages'] = dict(api_id=1006, code=code,
        package_version=package, module_versions=modules,
        meaning='Package/module metadata only; physical host and motor firmware not verified')
    result['rpc_calls'] = len(SERVICES)+2
    result['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    return result


if __name__ == '__main__':
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.rpc.client import Client
    ChannelFactoryInitialize(0, 'eth0')
    print(json.dumps(inspect(Client, state_client(Client)), ensure_ascii=False, indent=2))
