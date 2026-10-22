"""Read-only envelopes, source coverage and separation from device authority."""
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

from g1_console.body import KNOWLEDGE
from scripts import audit_g1_services as services
from scripts import build_g1_capability_knowledge as builder


def test_every_public_python_business_method_and_default_is_preserved():
    catalog=KNOWLEDGE['official_capabilities']
    for group in ('loco','arm','audio','video'):
        source=catalog['sources'][f'py_{group}_client']
        tree=ast.parse((builder.ROOT/source['path']).read_text())
        expected={n.name for cls in tree.body if isinstance(cls,ast.ClassDef) for n in cls.body
                  if isinstance(n,ast.FunctionDef) and not n.name.startswith('_') and n.name!='Init'}
        rows=[r for r in catalog['methods'] if r['language']=='Python' and r['group']==group]
        assert {r['name'] for r in rows}==expected
    move=next(m for m in catalog['methods'] if m['id']=='py.loco.Move.0')
    assert move['parameters'][-1]['name']=='continous_move'  # Preserve the actual SDK spelling.
    assert move['parameters'][-1]['default']=='False'
    assert move['api_ids']==[7105]
    assert catalog['execution_allowed'] is False


def test_cpp_only_methods_and_incompatible_wire_dialects_stay_visible():
    c=KNOWLEDGE['official_capabilities']; byid={m['id']:m for m in c['methods']}
    assert byid['py.arm.ExecuteAction.0']['wire']=={'data':'action_id'}
    assert '"action_id"' in byid['cpp.arm.ExecuteAction.0']['source_excerpt']
    assert byid['cpp.arm.ExecuteAction.1']['api_ids']==[7108]
    assert byid['cpp.arm.StopCustomAction.0']['api_ids']==[7113]
    assert byid['cpp.loco.GetMimicMotion.0']['wire']=={'fsm_id':550,'api_id':1}
    assert byid['cpp.loco.SwitchMoveMode.0']['kind']=='client_local'
    assert byid['cpp.loco.SwitchMoveMode.0']['api_ids']==[]
    assert byid['py.loco.HighStand.0']['wire']=={'SetStandHeight':4294967295}
    assert byid['cpp.loco.StandUp.0']['wire']!={'SetFsmId':706}
    assert all(not m['executed'] and not m['device_parameter_values_verified'] for m in c['methods'])
    assert all('AGV' in m['support'] for m in c['methods'] if m['group']=='agv')
    assert 'return 0' in byid['py.audio.PlayStop.0']['source_excerpt']


def test_getter_program_cannot_send_write_or_multiplexed_fsm_requests(capsys):
    # Compile the exact executable body with inert transport substitutes. Do not import SDK.
    tree=ast.parse((builder.ROOT/'scripts/audit_g1_capability_reads.py').read_text())
    tree.body=[n for n in tree.body if not (isinstance(n,ast.ImportFrom) and n.module.startswith('unitree_sdk2py'))
               and not isinstance(n,ast.If)]
    calls=[];leases=[]
    class FakeClient:
        def __init__(self,service,lease):self.service=service;leases.append(lease)
        def _SetApiVerson(self,version):pass
        def SetTimeout(self,timeout):pass
        def _RegistApi(self,api,priority):pass
        def _Call(self,api,payload):
            calls.append((self.service,api,payload))
            return (7301,'{"data":999}') if api==7004 else (0,'{"data":0}')
    class BootPath:
        def __init__(self,path):assert path=='/proc/sys/kernel/random/boot_id'
        def read_text(self):return 'test-boot'
    namespace={}
    exec(compile(tree,'audit_getters','exec'),namespace)
    namespace.update(Client=FakeClient,Path=BootPath,ChannelFactoryInitialize=lambda domain,interface:None)
    namespace['main']()
    result=json.loads(capsys.readouterr().out)
    assert calls==[(s,i,'') for s,i in [('sport',7001),('sport',7002),('sport',7003),('sport',7004),('sport',7005),('arm',7107),('voice',1005)]]
    assert leases==[False,False,False]
    assert next(r for r in result['readings'] if r['api_id']==7004)['payload'] is None
    assert all(i!=7100 for _,i,_ in calls)
    assert result['command_writes']==result['lease_requests']==0


def test_actual_audit_preserves_opaque_files_errors_and_unknown_motor_values():
    a=KNOWLEDGE['service_audit']; c=KNOWLEDGE['official_capabilities']
    assert not a['current'] and len(a['processes'])==4
    opaque={f['path']:f for f in a['files'] if f['format']=='opaque'}
    assert opaque['/unitree/module/master_service/master_service.json']['bytes']==468
    assert opaque['/home/unitree/megaflow_sdk/ws/bin/config.bin']['bytes']==14640
    assert all('text' not in f for f in opaque.values())
    assert all(not f['loaded_by_active_motor_controller_verified'] for f in a['files'])
    assert any(l.get('matching_lines',0)>0 and l.get('search_bytes',0)>65536 for l in a['logs'])
    readings=c['historical_reads']['readings']
    assert all(r['payload'] is None for r in readings if r['code']!=0)
    assert all(v is None for motor in KNOWLEDGE['motors'] for v in motor['actual_config'].values())
    assert a['command_writes']==a['rpc_calls']==a['motor_configuration_values_obtained']==0


def test_changed_official_capability_source_is_rejected(tmp_path,monkeypatch):
    source=deepcopy(KNOWLEDGE['official_capabilities']['sources']['cpp_loco_client'])
    path=tmp_path/source['path'];path.parent.mkdir(parents=True);path.write_text('int32_t Fake() {}')
    monkeypatch.setattr(builder,'ROOT',tmp_path)
    with pytest.raises(AssertionError,match='g1_loco_client.hpp'):
        builder.read_source(source)


def test_sanitization_and_service_inspector_have_no_vendor_execution():
    assert 'secret-value' not in services.sanitize('token=secret-value\nnormal config')
    assert 'normal config' in services.sanitize('token=secret-value\nnormal config')
    assert 'pass@' not in services.sanitize('http://user:pass@localhost/path?auth=secret')
    tree=ast.parse(Path(services.__file__).read_text())
    imports={n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
    assert not any(n and 'unitree' in n for n in imports)
    commands=[n.args[0] for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
              and isinstance(n.func.value,ast.Name) and n.func.value.id=='subprocess' and n.func.attr=='run']
    assert len(commands)==2
    assert {n.elts[0].value for n in commands}=={'systemctl','journalctl'}
    assert not any(isinstance(n,ast.keyword) and n.arg=='shell' for n in ast.walk(tree))
    for source in KNOWLEDGE['service_audit']['receipt']['sources']:
        assert hashlib.sha256((builder.ROOT/source['path']).read_bytes()).hexdigest()==source['sha256']
