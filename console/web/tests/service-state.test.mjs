import test from 'node:test';
import assert from 'node:assert/strict';
import {serviceRows,serviceStatus} from '../src/serviceState.ts';

test('service status uses the documented enum instead of the switch-input enum',()=>{
 assert.equal(serviceStatus(0),'开启');
 assert.equal(serviceStatus(1),'关闭');
 assert.equal(serviceStatus(2),'未知枚举 2');
 assert.equal(serviceStatus(-1),'未知枚举 -1');
});

test('missing, boolean or error-placeholder service state stays unknown',()=>{
 for(const data of [null,{},[{name:'ai_sport',status:false,protect:0}],
  [{name:'ai_sport',protect:0}],[{name:'ai_sport',status:0,protect:null}]]){
  assert.equal(serviceRows(data),null);
 }
 assert.deepEqual(serviceRows([{name:'ai_sport',status:0,protect:0}]),[{name:'ai_sport',status:0,protect:0}]);
 // No invented version, support, balance or safety fields are added.
 assert.deepEqual(serviceRows([{name:'state_estimator',status:9,protect:2}]),[{name:'state_estimator',status:9,protect:2}]);
});
