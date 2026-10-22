#!/usr/bin/env python3
"""Bounded, read-only G1 board audit. Run on Orin; never creates a DDS writer."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import time


def inspect(interface='eth0', duration=10):
    from cyclonedds.domain import Domain, DomainParticipant
    from cyclonedds.builtin import (BuiltinDataReader, BuiltinTopicDcpsParticipant,
                                   BuiltinTopicDcpsPublication, BuiltinTopicDcpsSubscription)

    if interface != 'eth0' or not 1 <= duration <= 30:
        raise ValueError('This audit is restricted to Orin eth0 and 1–30 seconds')
    result = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
                  method='SSH reads, one TCP/22 connection, DDS discovery and passive application DataReaders',
                  scope='Orin and the documented PC1 address; no application topic writes or RPC calls',
                  command_writes=0, rpc_calls=0, motor_configuration_values_obtained=0)
    result['network'] = {}
    for name, cmd in [('interfaces', ['ip','-br','-4','addr']),
                      ('routes', ['ip','-4','route','show']),
                      ('neighbors', ['ip','neigh','show','dev',interface])]:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=4)
        result['network'][name] = dict(returncode=p.returncode, output=p.stdout.strip())
    target = '192.168.123.161'
    with socket.socket() as sock:
        sock.settimeout(3)
        code = sock.connect_ex((target, 22))
        result['pc1_ssh'] = dict(address=target, port=22, errno=code,
                                status='open' if code == 0 else 'refused' if code == 111 else 'unavailable',
                                authentication_attempted=False,
                                limitation='A failed TCP/22 connection says nothing about other ports or firmware configuration')
    files = {}
    for name in ['/proc/device-tree/model', '/etc/nv_tegra_release',
                 '/etc/os-release', '/unitree/robot/pkg/version/version.json', '/etc/g1_version']:
        p = Path(name)
        try:
            data = p.read_bytes()
            files[name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest(),
                               text=data.decode('utf-8').rstrip('\x00\n'))
        except (OSError, UnicodeDecodeError) as exc:
            files[name] = dict(error=type(exc).__name__)
    result['orin_files'] = files
    # Configure this process only; do not alter the system network or vendor DDS configuration.
    domain = Domain(0, f'<CycloneDDS><Domain Id="0"><General><Interfaces>'
                    f'<NetworkInterface name="{interface}"/></Interfaces></General></Domain></CycloneDDS>')
    participant = DomainParticipant(0)
    readers = {name: BuiltinDataReader(participant, topic) for name, topic in [
        ('participants', BuiltinTopicDcpsParticipant), ('publications', BuiltinTopicDcpsPublication),
        ('subscriptions', BuiltinTopicDcpsSubscription)]}
    found = {name: {} for name in readers}
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        for name, reader in readers.items():
            for sample in reader.take(500):
                if not sample.sample_info.valid_data:
                    continue
                row = dict(key=str(sample.key))
                if name != 'participants':
                    row.update(participant_key=str(sample.participant_key), topic=sample.topic_name,
                               type=sample.type_name)
                # Do not copy arbitrary user data or private property values.
                row['entity_names'] = [p.name for p in sample.qos if type(p).__name__ == 'EntityName']
                found[name][row['key']] = row
        time.sleep(.1)
    result['dds_discovery'] = dict(domain=0, interface=interface, duration_s=duration,
        local_participant=str(participant.get_guid()),
        **{name: sorted(rows.values(), key=lambda r:r['key']) for name, rows in found.items()},
        limitations=['DDS participants/endpoints are software entities, not a count of physical control boards',
                     'Endpoint discovery alone does not identify its IP, motor serial, firmware, gain or calibration',
                     'An absent endpoint during this bounded observation does not prove the capability is absent'])
    # Subscribe only to observed, SDK-defined types. Even LowCmd is READ here, never written.
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.qos import Qos, Policy
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MainBoardState_, BmsState_, LowCmd_, LowState_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    expected = {'rt/lf/mainboardstate': MainBoardState_, 'rt/lf/bmsstate': BmsState_,
                'rt/lowcmd': LowCmd_, 'rt/lowstate': LowState_, 'rt/selftest': String_}
    advertised = {r['topic']:r['type'] for r in found['publications'].values()}
    data_readers, observed = {}, {}
    for name, cls in expected.items():
        typename = cls.__idl_typename__.replace('.', '::')
        if advertised.get(name) != typename:
            observed[name] = dict(samples=0, status='matching type not discovered')
            continue
        topic = Topic(participant, name, cls)
        data_readers[name] = DataReader(participant, topic,
            qos=Qos(Policy.Reliability.BestEffort, Policy.Durability.Volatile, Policy.History.KeepLast(1)))
        observed[name] = dict(samples=0, status='no sample received', type=advertised[name],
                              advertised_publication_keys=[r['key'] for r in found['publications'].values() if r['topic']==name],
                              publisher_device_verified=False)
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        for name, reader in data_readers.items():
            for sample in reader.take(4):
                if not sample.sample_info.valid_data:
                    continue
                row = observed[name]
                row.update(samples=row['samples']+1, status='sample received',
                           received_at_utc=datetime.now(timezone.utc).isoformat())
                if name == 'rt/lowstate':
                    from unitree_sdk2py.utils.crc import CRC
                    row['crc_valid'] = CRC().Crc(sample) == sample.crc
                    row['last'] = dict(version=list(sample.version), mode_pr=sample.mode_pr,
                                       mode_machine=sample.mode_machine, tick=sample.tick)
                elif name == 'rt/lowcmd':
                    from unitree_sdk2py.utils.crc import CRC
                    row['crc_valid'] = CRC().Crc(sample) == sample.crc
                    row['last'] = dict(mode_pr=sample.mode_pr, mode_machine=sample.mode_machine,
                        motors=[dict(index=i, **asdict(m)) for i,m in enumerate(sample.motor_cmd)])
                    row['meaning'] = 'Observed command payload only; not factory gains or proof that a driver applies it'
                elif name == 'rt/selftest':
                    row['last'] = dict(data=sample.data[:8192], truncated=len(sample.data)>8192)
                    row['meaning'] = 'Vendor text; interpretation requires a matching official schema'
                else:
                    row['last'] = asdict(sample)
                    row['meaning'] = 'Raw official HG IDL fields; array roles, units and firmware semantics remain unverified'
        time.sleep(.05)
    result['passive_samples'] = observed
    result['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--duration', type=float, default=10)
    args = parser.parse_args()
    print(json.dumps(inspect(args.interface, args.duration), ensure_ascii=False, indent=2))
