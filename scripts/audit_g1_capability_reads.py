#!/usr/bin/env python3
"""Fixed public getters only; no lease, setters, multiplexed FSM API or actions."""
from datetime import datetime, timezone
import json
from pathlib import Path
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client

# All IDs, versions and empty-string getter payloads are from pinned C++ G1 clients.
READS = (('sport', '1.0.0.0', 7001, 'GetFsmId'),
         ('sport', '1.0.0.0', 7002, 'GetFsmMode'),
         ('sport', '1.0.0.0', 7003, 'GetBalanceMode'),
         ('sport', '1.0.0.0', 7004, 'GetSwingHeight'),
         ('sport', '1.0.0.0', 7005, 'GetStandHeight'),
         ('arm', '1.0.0.14', 7107, 'GetActionList'),
         ('voice', '1.0.0.0', 1005, 'GetVolume'))


def main():
    result = dict(current=False, checked_at_utc=datetime.now(timezone.utc).isoformat(),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        command_writes=0, lease_requests=0, configuration_calls=0, calibration_calls=0,
        motor_configuration_values_obtained=0, readings=[],
        unqueried=[dict(method='GetMimicMotion', reason='官方要求稳定状态，未核实；不调用复用 7100 的 FSM 入口'),
                   dict(method='GetPhase', reason='官方已标记 deprecated；本轮不查询')])
    ChannelFactoryInitialize(0, 'eth0')
    clients = {}
    for service, version, api_id, method in READS:
        if service not in clients:
            client = Client(service, False)
            client._SetApiVerson(version)
            client.SetTimeout(3.0)
            clients[service] = client
        client = clients[service]
        client._RegistApi(api_id, 0)
        try:
            code, raw = client._Call(api_id, '')
            try:
                payload = json.loads(raw) if code == 0 else None
            except (TypeError, ValueError):
                payload = None
            result['readings'].append(dict(service=service, method=method, api_id=api_id,
                parameter='', code=code, payload=payload, received_at_utc=datetime.now(timezone.utc).isoformat(),
                meaning='历史服务返回；不是逐电机配置或稳定性证明'))
        except Exception as error:
            result['readings'].append(dict(service=service, method=method, api_id=api_id,
                code=None, payload=None, error=type(error).__name__))
    result['rpc_calls'] = len(result['readings'])
    result['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
