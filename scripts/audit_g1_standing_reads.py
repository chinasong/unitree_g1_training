#!/usr/bin/env python3
"""Bounded official metadata/getter audit; no state changes or control lease."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time

# API 1 is GetServerApiVersion. Empty getter payloads follow pinned C++ clients.
READS = (
    ('sport', 1, '{}', 'GetServerApiVersion'),
    ('motion_switcher', 1, '{}', 'GetServerApiVersion'),
    ('arm', 1, '{}', 'GetServerApiVersion'),
    ('sport', 7003, '', 'GetBalanceMode'),
    ('sport', 7005, '', 'GetStandHeight'),
    ('arm', 7107, '{}', 'GetActionList'),
)


def main():
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelFactory
    from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
        Request_, Response_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)

    result = dict(current=False, checked_at_utc=datetime.now(timezone.utc).isoformat(),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        command_writes=0, lease_requests=0, configuration_calls=0, calibration_calls=0,
        motor_configuration_values_obtained=0, readings=[])
    channels = {}
    ChannelFactoryInitialize(0, 'eth0')
    try:
        for service, api, parameter, method in READS:
            if service not in channels:
                writer = ChannelFactory().CreateChannel(f'rt/api/{service}/request', Request_)
                reader = ChannelFactory().CreateChannel(f'rt/api/{service}/response', Response_)
                channels[service] = writer, reader
                reader.SetReader()
                writer.SetWriter()
            writer, reader = channels[service]
            identity = RequestIdentity_(time.monotonic_ns(), api)
            request = Request_(RequestHeader_(identity, RequestLease_(0), RequestPolicy_(0, False)), parameter, [])
            started = time.monotonic()
            deadline = started + 1.5
            code, payload = 3102, None
            if writer.Write(request, 1.5):
                code = 3104
                while (remaining := deadline-time.monotonic()) > 0:
                    response = reader.Read(remaining)
                    if response is None:
                        break
                    if response.header.identity.id != identity.id:
                        continue
                    if response.header.identity.api_id != api:
                        code = 3105
                        break
                    code = response.header.status.code
                    if code == 0:
                        if len(response.data.encode()) > 262144:
                            raise ValueError('oversized read feedback')
                        try:
                            payload = json.loads(response.data)
                        except ValueError:
                            payload = response.data
                    break
            result['readings'].append(dict(service=service, api_id=api, parameter=parameter,
                method=method, code=code, payload=payload,
                elapsed_ms=round((time.monotonic()-started)*1000, 2),
                received_at_utc=datetime.now(timezone.utc).isoformat()))
    finally:
        for writer, reader in channels.values():
            reader.CloseReader()
            writer.CloseWriter()
    result['rpc_calls'] = len(result['readings'])
    result['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    result['meaning'] = 'Historical official read responses. API version is not robot firmware or capability applicability; errors retain unknowns.'
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
