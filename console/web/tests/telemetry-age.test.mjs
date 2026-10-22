import test from 'node:test';
import assert from 'node:assert/strict';
import {telemetryAgeMs} from '../src/telemetryAge.ts';

test('last receipt age advances throughout a disconnection and resets on new feedback',()=>{
 assert.equal(telemetryAgeMs(45,10000,10000),45);
 assert.equal(telemetryAgeMs(45,10000,13500),3545);
 assert.equal(telemetryAgeMs(12,13500,13510),22);
 assert.equal(telemetryAgeMs(0,13500,13500),0);
});

test('missing or invalid feedback age never becomes a zero or invented measurement',()=>{
 for(const age of [null,undefined,NaN,Infinity,-1])assert.equal(telemetryAgeMs(age,10000,11000),null);
 assert.equal(telemetryAgeMs(45,0,11000),null);
 assert.equal(telemetryAgeMs(45,NaN,11000),null);
 assert.equal(telemetryAgeMs(45,10000,NaN),null);
 assert.equal(telemetryAgeMs(45,10000,9900),45);
});
