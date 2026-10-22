import test from 'node:test';
import assert from 'node:assert/strict';
import {setTimeout as delay} from 'node:timers/promises';
import {Deadman} from '../src/deadman.ts';
const velocity={vx:.15,vy:0,omega:0};
const deferred=()=>{let resolve,reject;const promise=new Promise((a,b)=>{resolve=a;reject=b;});return {promise,resolve,reject};};

test('release while velocity is in flight sends stop and a late acknowledgement never resumes',async()=>{
 const pending=deferred(),writes=[];
 const d=new Deadman(async()=>{writes.push('velocity');await pending.promise;},async()=>{writes.push('stop');},assert.fail);
 d.start(velocity);d.release();pending.resolve();await delay(150);
 assert.deepEqual(writes,['velocity','stop']);
});

test('held input renews sequentially; duplicate start cannot add a second stream',async()=>{
 let inFlight=0,maxFlight=0;const writes=[];
 const d=new Deadman(async()=>{++inFlight;maxFlight=Math.max(maxFlight,inFlight);writes.push('velocity');await delay(20);--inFlight;},async()=>writes.push('stop'),assert.fail);
 d.start(velocity);d.start(velocity);await delay(155);d.release();await delay(140);
 assert.equal(maxFlight,1);assert.equal(writes.filter(v=>v==='velocity').length,2);assert.equal(writes.at(-1),'stop');
});

test('rejection stops without retry or automatic restart',async()=>{
 const writes=[],errors=[];
 const d=new Deadman(async()=>{writes.push('velocity');throw new Error('rejected');},async()=>writes.push('stop'),m=>errors.push(m));
 d.start(velocity);await delay(150);assert.deepEqual(writes,['velocity','stop']);assert.deepEqual(errors,['rejected']);
});

test('stalled request expires, reports and stops; resolving afterwards never continues',async()=>{
 const pending=deferred(),writes=[],errors=[];
 const d=new Deadman(async()=>{writes.push('velocity');await pending.promise;},async()=>writes.push('stop'),m=>errors.push(m));
 d.start(velocity);await delay(340);pending.resolve();await delay(120);
 assert.deepEqual(writes,['velocity','stop']);assert.equal(errors.length,1);
});

test('an old failed request cannot cancel a newly held input',async()=>{
 const old=deferred(),writes=[],errors=[];let count=0;
 const d=new Deadman(async v=>{writes.push(v.vx);if(++count===1)await old.promise;},async()=>writes.push('stop'),m=>errors.push(m));
 d.start(velocity);d.release();d.start({...velocity,vx:-.15});old.reject(new Error('late'));await delay(20);d.release();await delay(120);
 assert.deepEqual(writes,[.15,'stop',-.15,'stop']);assert.deepEqual(errors,[]);
});

test('update swaps the renewed velocity in place; all-zero input releases with one stop',async()=>{
 const writes=[];
 const d=new Deadman(async v=>{writes.push(v.vx);},async()=>writes.push('stop'),assert.fail);
 d.update({vx:.1,vy:0,omega:0});await delay(50);d.update({vx:.2,vy:0,omega:0});await delay(120);
 d.update({vx:0,vy:0,omega:0});d.update({vx:0,vy:0,omega:0});await delay(140);
 assert.deepEqual(writes,[.1,.2,'stop']);
});
