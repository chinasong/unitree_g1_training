"""Prove the two allowed writes reach dispatch while every motor route stays locked."""
import asyncio
import time

import pytest
from fastapi.testclient import TestClient

from g1_console.adapters import UnitreeAdapter
from g1_console.app import create_app
from g1_console.capabilities import CATALOG, READ_OPERATIONS, ACCESSORY_TEST_OPERATIONS
from g1_console.config import ConsoleError, Settings
from scripts.connect_orin_console import verify_configuration
from scripts.orin_console_recovery import restore
from test_g1_manual_remote import setup, OWNER, OTHER, command, arm


def test_exact_scope_is_required_by_settings_and_reconnector(tmp_path):
    Settings(manual_remote=True, accessory_test=True).validate()
    for kwargs in ({}, {'manual_remote':True,'verify_only':True}, {'manual_remote':True,'read_only':True}):
        with pytest.raises(ValueError):
            Settings(accessory_test=True, **kwargs).validate()
    state=dict(read_only=False, manual_remote=dict(enabled=True, verify_only=False,
               accessory_test=True, allowed_write_operations=['led','tts']))
    verify_configuration(state, manual_remote=True, accessory_test=True)
    for change in ({'accessory_test':False}, {'verify_only':True}, {'allowed_write_operations':['led','tts','damp']}, {'allowed_write_operations':None}):
        with pytest.raises(RuntimeError):
            verify_configuration({**state,'manual_remote':{**state['manual_remote'],**change}}, True, accessory_test=True)
    for manual, verify in ((False,False),(True,True)):
        with pytest.raises(ValueError):
            restore(tmp_path, manual=manual, verify_only=verify, accessory_test=True)


def test_allowed_writes_reach_consumer_without_arm_or_standing_and_are_idempotent(tmp_path):
    async def check():
        s=await setup(tmp_path, accessory_test=True)
        s.adapter.fsm=0
        await s._sample()
        for sequence,(op,params) in enumerate((('tts',{'text':'连接测试'}),('led',{'r':0,'g':12,'b':0})),1):
            c=command(op,seq=sequence,params=params)
            result=await s.command(c)
            assert result['status']=='accepted'
            assert await s.command(c)==result
        assert [op for op,_ in s.adapter.calls]==['tts','led']
        snap=s.snapshot()
        assert snap['manual_remote']['allowed_write_operations']==['led','tts']
        assert not snap['manual_remote']['armed'] and snap['telemetry']['fsm']==0
        await s.heartbeat(OWNER)
        await s.release(OWNER)
        assert [op for op,_ in s.adapter.calls]==['tts','led']
    asyncio.run(check())


def test_every_other_write_route_blocked_even_when_arm_is_forged(tmp_path):
    async def check():
        s=await setup(tmp_path, accessory_test=True)
        with pytest.raises(ConsoleError) as exc:
            await arm(s)
        assert exc.value.code=='ACCESSORY_TEST_ONLY'
        s.manual_permit=dict(owner=OWNER,until=time.monotonic()+60,observed_at=time.time())
        for cap in CATALOG:
            op=cap['id']
            if op in READ_OPERATIONS|ACCESSORY_TEST_OPERATIONS:continue
            assert s.execution_gate(op)['code']=='ACCESSORY_TEST_ONLY'
            for route in (s.command(command(op)),s._rpc(op,{},manual_owner=OWNER)):
                with pytest.raises(ConsoleError) as exc:
                    await route
                assert exc.value.code=='ACCESSORY_TEST_ONLY'
        assert not s.adapter.calls
        await s._rpc('get_fsm',{})
        assert s.adapter.calls==[('get_fsm',{})]
        s.motion_active=True
        await s._stop('forced stale fixture')
        assert s.adapter.calls==[('get_fsm',{})]
    asyncio.run(check())


def test_accessory_requires_owner_freshness_and_has_no_retry_after_timeout(tmp_path):
    async def check():
        s=await setup(tmp_path, accessory_test=True)
        for owner in (None,OTHER):
            with pytest.raises(ConsoleError):await s._rpc('tts',{'text':'测试'},manual_owner=owner)
        s.lease_until=time.monotonic()-1
        assert not s.execution_gate('tts')['allowed']
        with pytest.raises(ConsoleError):await s.command(command('tts',params={'text':'测试'}))
        assert not s.adapter.calls
        await s.acquire(OWNER,False)
        c=command('tts',params={'text':'测试'}).model_copy(update={'issued_at':time.time()-10})
        with pytest.raises(ConsoleError) as exc:await s.command(c)
        assert exc.value.code=='EXPIRED_COMMAND' and not s.adapter.calls
        calls=[]
        def timeout(op,params):calls.append((op,params));return 3104,None
        s.adapter.execute=timeout
        c=command('tts',seq=2,params={'text':'测试'})
        for _ in range(2):
            with pytest.raises(ConsoleError) as exc:await s.command(c)
            assert exc.value.code=='SDK_TIMEOUT'
        assert len(calls)==1
        assert s.snapshot()['command_history'][0]['status']=='outcome_unknown'
    asyncio.run(check())


def test_adapter_rejects_motor_writes_before_loading_any_sdk():
    adapter=object.__new__(UnitreeAdapter)
    adapter.read_only = adapter.verify_only = adapter.accessory_test = adapter.damping_test = False
    adapter.accessory_test=True
    for cap in CATALOG:
        if cap['id'] not in READ_OPERATIONS|ACCESSORY_TEST_OPERATIONS:
            with pytest.raises(ConsoleError) as exc:adapter.execute(cap['id'],{})
            assert exc.value.code=='ACCESSORY_TEST_ONLY'


def test_http_scope_owner_origin_and_positive_write(tmp_path):
    with TestClient(create_app(Settings(manual_remote=True, accessory_test=True,data_dir=tmp_path))) as client:
        client.get('/api/session');headers={'X-G1-Intent':'operator'}
        client.post('/api/connect',json={'mode':'demo'},headers=headers).raise_for_status()
        data=command('tts',params={'text':'测试'}).model_dump()
        assert client.post('/api/command',json=data,headers=headers).json()['error']['code']=='NO_CONTROL_LEASE'
        client.post('/api/control/acquire',json={'owner':OWNER},headers=headers).raise_for_status()
        data=command('tts',seq=2,params={'text':'测试'}).model_dump()
        assert client.post('/api/command',json=data).status_code==403
        assert client.post('/api/command',json=data,headers={**headers,'Origin':'http://evil.test'}).status_code==403
        assert client.post('/api/command',json=data,headers=headers).json()['status']=='accepted'
        data=command('damp',seq=3,acknowledge_risk=True).model_dump()
        assert client.post('/api/command',json=data,headers=headers).json()['error']['code']=='ACCESSORY_TEST_ONLY'
        assert not client.get('/api/state').json()['manual_remote']['armed']
