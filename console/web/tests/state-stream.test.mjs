import test from 'node:test';
import assert from 'node:assert/strict';
import {followState} from '../src/stateStream.ts';

const flush=()=>new Promise(resolve=>setImmediate(resolve));
function fixture(refreshSession=async()=>{}){
 const sockets=[],values=[],timers=[],calls=[];let offline=0;
 const close=followState({refreshSession:async()=>{calls.push('GET /api/session');await refreshSession();},
  createSocket:()=>{calls.push('WS /api/live');const s={close(){s.onclose?.();}};sockets.push(s);return s;},
  onValue:v=>values.push(v),onOffline:()=>offline++,onInvalid:()=>{},
  schedule:callback=>{timers.push(callback);return timers.length;},cancel:()=>{}});
 return {sockets,values,timers,calls,close,get offline(){return offline;}};
}

test('reboot rotates the cookie: renew session before reopening feedback, never replay a command',async()=>{
 let cookie='old-boot',boot='old-boot';
 const f=fixture(async()=>{cookie=boot;});await flush();
 f.sockets[0].onmessage({data:JSON.stringify({run_id:'old',connected:true})});
 boot='new-boot';f.sockets[0].onclose();assert.equal(f.offline,1);
 f.timers.shift()();await flush();assert.equal(cookie,'new-boot');
 f.sockets[0].onmessage({data:JSON.stringify({run_id:'old-stale'})});
 f.sockets[1].onmessage({data:JSON.stringify({run_id:'new',connected:false})});
 assert.deepEqual(f.values,[{run_id:'old',connected:true},{run_id:'new',connected:false}]);
 assert.deepEqual(f.calls,['GET /api/session','WS /api/live','GET /api/session','WS /api/live']);f.close();
});

test('offline session retries; closing a view while authentication is pending opens no socket',async()=>{
 let available=false;
 const f=fixture(async()=>{if(!available)throw Error('503 offline');});await flush();
 assert.equal(f.sockets.length,0);assert.equal(f.offline,1);
 available=true;f.timers.shift()();await flush();assert.equal(f.sockets.length,1);f.close();
 let finish;const pending=fixture(()=>new Promise(resolve=>{finish=resolve;}));
 pending.close();finish();await flush();assert.equal(pending.sockets.length,0);
});
