import {useState} from 'react';
import {ApiError} from './api';
import type {Snapshot} from './types';

const READS = [
 ['get_balance',7003,'平衡模式'],
 ['get_swing_height',7004,'摆脚高度'],
 ['get_stand_height',7005,'站立高度'],
 ['get_phase',7006,'相位'],
] as const;

export default function LocoReadback({state,fresh,command}:{state:Snapshot;fresh:boolean;command:(id:string)=>Promise<unknown>}){
 const [busy,setBusy]=useState(false);
 const [message,setMessage]=useState('');
 const feedback=state.telemetry.safety_feedback;
 const current=(id:string)=>fresh&&state.mode==='robot'&&feedback?.[id]?.response_recent===true;
 const allRecent=READS.every(([id])=>current(id));
 const rejected=READS.filter(([id])=>current(id)&&feedback?.[id]?.code===7301).length;
 async function read(){
  if(busy)return;
  setBusy(true);setMessage('正在逐项读取；不会自动重试或切换模式。');
  try{
   for(const [id] of READS){
    try{await command(id);}
    catch(e){
     // A rejected getter is evidence: continue with the other fixed getters.
     // Connection/authentication errors stop the group immediately.
     if(!(e instanceof ApiError)||!['SDK_REJECTED','SDK_TIMEOUT'].includes(e.code))throw e;
    }
   }
   setMessage('四项查询已返回，请核对表中接收时间。错误应答没有有效参数值。');
  }catch(e){setMessage(e instanceof Error?e.message:'查询中断，缺失项保持未知');}
  finally{setBusy(false);}
 }
 return <div className="version-listener" aria-label="LocoState 实时只读诊断">
  <div className="panel-title"><h3>LocoState 四项只读诊断</h3><button className="secondary" disabled={busy||!fresh||state.mode!=='robot'} onClick={()=>void read()}>{busy?'正在读取…':'读取四项 LocoState'}</button></div>
  <p>{!allRecent?'尚无完整的新鲜查询；不能判断当前四项接口是否可读。':rejected?`${rejected} / 4 项返回 7301；LocoState 相关参数不可读。`:READS.every(([id])=>feedback?.[id]?.current)?'四项读取成功；这仍不证明控制权、起身适用或身体已站稳。':'查询有失败项；参数保持未知。'}</p>
  <div className="table-scroll"><table><thead><tr><th>官方接口</th><th>本次回读</th><th>有效值</th><th>接收时间</th></tr></thead><tbody>{READS.map(([id,api,label])=>{
   const row=feedback?.[id];const recent=current(id);
   return <tr key={id}><td>{label}<small>sport / API {api}</small></td><td>{!recent?'未知 / 未查询或已过期':row?.code===0?'读取成功':`${String(row?.code)} · ${row?.code===7301?'LocoState 不可用':row?.code===3104?'接收超时':'查询失败'}`}</td><td>{recent&&row?.current?(JSON.stringify(row.raw?.data)??'未知').slice(0,180):'未知'}</td><td>{row?.received_at?`${new Date(row.received_at*1000).toLocaleTimeString('zh-CN',{hour12:false})} · ${row.age_ms} ms 前`:'尚未接收'}</td></tr>;
  })}</tbody></table></div>
  <p className="panel-note">{message||'按需读取，不自动轮询。'} 结果 15 秒后过期；断连或读取进程异常会撤回有效值。公开状态流持续更新、SDK Init 完成，均不等于服务内部 LocoState 已就绪。</p>
 </div>;
}
