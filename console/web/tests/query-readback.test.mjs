import test from 'node:test';
import assert from 'node:assert/strict';
import {latestQueryReadback,queryReadbackLabel} from '../src/queryReadback.ts';

const row=(overrides={})=>({request_id:'receipt',capability:'get_balance',label:'平衡状态',at:100,updated_at:100,mode:'robot',status:'read_result',message:'已读取',data:0,...overrides});

test('a newer failed query supersedes an older success, regardless of array order',()=>{
 const success=row(), failure=row({at:102,status:'read_unavailable',sdk_code:7301,data:null});
 for(const history of [[success,failure],[failure,success]]){
  assert.equal(latestQueryReadback(history,'get_balance','robot'),failure);
  assert.match(queryReadbackLabel(failure),/7301.*值未知/);
 }
 assert.equal(queryReadbackLabel(success),'已读取 · 历史样本');
});

test('demo, other capabilities, invalid timestamps and control acknowledgements cannot replace real query evidence',()=>{
 const misleading=[row({mode:'demo'}),row({capability:'get_fsm'}),row({at:NaN}),row({status:'sdk_ack'})];
 assert.equal(latestQueryReadback(misleading,'get_balance','robot'),null);
 assert.equal(latestQueryReadback([row()],'get_balance','disconnected'),null);
 assert.equal(latestQueryReadback([],'get_balance','robot'),null);
});

test('all errors retain their raw code and never turn a placeholder zero into a value',()=>{
 for(const code of [7301,3203,3104,9999,'SDK_REJECTED']){
  const failure=row({status:'read_unavailable',sdk_code:code,data:0});
  assert.match(queryReadbackLabel(failure),new RegExp(String(code)));
  assert.match(queryReadbackLabel(failure),/值未知/);
 }
 assert.match(queryReadbackLabel(row({status:'outcome_unknown',sdk_code:null})),/值未知/);
});
