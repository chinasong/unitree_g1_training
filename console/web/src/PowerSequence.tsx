import {useEffect,useRef,useState} from 'react';
import {CheckCircleIcon,CircleIcon,PowerIcon,ShieldCheckIcon,SpinnerGapIcon,WarningCircleIcon,XIcon} from '@phosphor-icons/react';
import {api} from './api';
import type {Snapshot} from './types';

type Route='seat'|'squat';
type Kind='start'|'stop';
type Props={state:Snapshot;held:boolean;owner:string;takeControl:()=>Promise<unknown>|void;command:(id:string,params?:Record<string,unknown>,ack?:boolean)=>Promise<unknown>;report:(text:string)=>void;onStarted:()=>void};
// Mirrors g1_console/mode_sequence.py ROUTES; the service re-checks every precondition.
// Which directions may run, and why not, comes from the service (mode_sequence.EVIDENCE).
const ROUTES:Record<Route,{label:string;start:string;stop:string;stopFrom:number[]}>={
 seat:{label:'座椅',start:'机器人端坐在椅子上，双脚着地',stop:'椅子已放回机器人身后，落座位置已清空',stopFrom:[500,4]},
 squat:{label:'蹲姿',start:'机器人双脚着地呈蹲姿',stop:'机器人下方与周围的下蹲空间已清空',stopFrom:[500,706]},
};
const COMMON=['吊架或专人保护已就位','机器人周围与手臂活动范围已清空','实体遥控器在手，可随时 L2+B 阻尼'];
const HOLD_MS=800;
const loadRoute=():Route=>{try{return localStorage.getItem('g1-route')==='squat'?'squat':'seat';}catch{return 'seat';}};

export default function PowerSequence({state,held,owner,takeControl,command,report,onStarted}:Props){
 const [route,setRoute]=useState<Route>(loadRoute);
 const [asking,setAsking]=useState<Kind|null>(null),[checks,setChecks]=useState([false,false,false,false]),[busy,setBusy]=useState(false);
 const [damping,setDamping]=useState(0);
 const holdTimer=useRef<ReturnType<typeof setInterval>|undefined>(undefined);
 const seq=state.manual_remote?.sequence??null;
 const live=seq?.status==='running'||seq?.status==='waiting_operator';
 const fsm=state.telemetry.fsm;
 const closed=state.manual_remote?.routes?.[route]??{start:'等待服务回报路线状态',stop:'等待服务回报路线状态'};
 const thermal=state.manual_remote?.thermal;
 const canStart=!live&&!closed.start&&!thermal?.hot&&(fsm===0||fsm===1);
 const canStop=!live&&!closed.stop&&fsm!==null&&ROUTES[route].stopFrom.includes(fsm);
 const announced=useRef<number|null>(null);

 useEffect(()=>{try{localStorage.setItem('g1-route',route);}catch{/* per-viewer convenience only */}},[route]);
 useEffect(()=>{  // A finished start chain leads straight into the walking confirmation.
  if(seq?.kind==='start'&&seq.status==='done'&&seq.finished_at&&announced.current!==seq.finished_at){
   const first=announced.current===null;announced.current=seq.finished_at;
   if(!first||Date.now()/1000-seq.finished_at<5)onStarted();
  }
 },[seq,onStarted]);
 useEffect(()=>()=>clearInterval(holdTimer.current),[]);

 const fail=(value:unknown,fallback:string)=>report(value instanceof Error?value.message:fallback);
 const ask=(kind:Kind)=>{setChecks([false,false,false,false]);setAsking(kind);};
 async function submit(){
  if(!asking)return;
  setBusy(true);
  try{
   if(!held)await takeControl();
   await api('/api/control/sequence',{owner,route,kind:asking,posture_observed:true,protection_ready:true,area_clear_observed:true,remote_in_hand:true});
   setAsking(null);
  }catch(value){fail(value,'一键启停未能开始');}
  finally{setChecks([false,false,false,false]);setBusy(false);}
 }
 const post=(path:string,body:Record<string,unknown>,fallback:string)=>void api(path,{owner,...body}).catch(value=>fail(value,fallback));
 const pressDamp=()=>{
  if(!held)return;
  const began=Date.now();clearInterval(holdTimer.current);
  holdTimer.current=setInterval(()=>{
   const progress=Math.min(1,(Date.now()-began)/HOLD_MS);setDamping(progress);
   if(progress>=1){clearInterval(holdTimer.current);setDamping(0);void command('damp',{},true).catch(value=>fail(value,'阻尼保护回执未确认；请用实体遥控器'));}
  },30);
 };
 const releaseDamp=()=>{clearInterval(holdTimer.current);setDamping(0);};

 return <div className="game-power" aria-label="一键启停">
  <div className="game-route" role="radiogroup" aria-label="启停路线">{(Object.keys(ROUTES) as Route[]).map(id=><button key={id} role="radio" aria-checked={route===id} disabled={live} className={route===id?'active':''} onClick={()=>setRoute(id)}>{ROUTES[id].label}</button>)}</div>
  <div className="game-power-actions">
   <button className="game-power-start" disabled={!canStart} title={closed.start??undefined} onClick={()=>ask('start')}><PowerIcon size={18} weight="bold"/>一键启动</button>
   <button className="game-power-stop" disabled={!canStop} title={closed.stop??undefined} onClick={()=>ask('stop')}><PowerIcon size={18}/>一键停止</button>
   <button className="game-damp" disabled={!held} onPointerDown={pressDamp} onPointerUp={releaseDamp} onPointerLeave={releaseDamp} onPointerCancel={releaseDamp} style={{'--hold':damping} as React.CSSProperties} title="按住 0.8 秒；对应手册 L2+B 阻尼保护，不是断电急停">阻尼保护</button>
  </div>
  {thermal?.hot&&<p className="game-power-note hot" role="alert">关节 {thermal.joint} 温度 {thermal.max_c} ℃，达到项目拦截线 {thermal.limit_c} ℃：不能一键启动或驾驶。现场扶稳后用实体遥控器 L2+B 阻尼卸力，再关机降温</p>}
  {!live&&[closed.start,closed.stop].filter((text,index,all)=>text&&all.indexOf(text)===index).map(text=><p key={text} className="game-power-note">{text}</p>)}
  {seq&&(live||Date.now()/1000-(seq.finished_at??0)<20)&&<div className={'game-sequence '+seq.status}>
   <header><b>{seq.route_label} · {seq.kind==='start'?'启动':'停止'}</b>{live&&<button aria-label="中止" onClick={()=>post('/api/control/sequence/abort',{},'中止请求失败；请按停止或用实体遥控器')}><XIcon size={14}/>中止</button>}</header>
   <ol>{seq.steps.map((step,index)=><li key={index} className={step.status}>{step.status==='done'?<CheckCircleIcon size={14} weight="fill"/>:step.status==='active'?<SpinnerGapIcon size={14} className="spin"/>:step.status==='aborted'?<WarningCircleIcon size={14} weight="fill"/>:<CircleIcon size={14}/>}<span>{step.label}</span>{step.target!==null&&<small>{step.target}</small>}</li>)}</ol>
   {seq.message&&<p>{seq.message}</p>}
   {seq.status==='waiting_operator'&&<button className="game-hold-continue" onClick={()=>post('/api/control/sequence/continue',{upright_held:true},'继续失败；序列保持锁定站立')}>已扶直，进入行走</button>}
  </div>}
  {asking&&<div className="game-confirm-backdrop" role="presentation"><section className="game-confirm" role="dialog" aria-modal="true" aria-labelledby="power-confirm-title"><header><div><span>{ROUTES[route].label}路线 · 官方手册</span><h2 id="power-confirm-title">{asking==='start'?'一键启动前请逐项确认':'一键停止前请逐项确认'}</h2></div><button aria-label="关闭" onClick={()=>setAsking(null)}><XIcon size={20}/></button></header><p>{asking==='start'?(route==='seat'?'阻尼 → 锁定站立 → 人工扶直 → 进入行走。每步回读确认后自动进入下一步。':'阻尼 → 蹲站起立 → 进入行走。每步回读确认后自动进入下一步。'):(route==='seat'?'停止行走 → 落座。':'停止行走 → 站姿下蹲。')}任一步异常即停在原地，不自动切换其他模式。</p><div className="game-confirm-list">{[asking==='start'?ROUTES[route].start:ROUTES[route].stop,...COMMON].map((text,index)=><label key={text}><input type="checkbox" checked={checks[index]} disabled={busy} onChange={event=>setChecks(current=>current.map((value,item)=>item===index?event.target.checked:value))}/><span><ShieldCheckIcon size={18}/>{text}</span></label>)}</div><button className="game-confirm-submit" disabled={busy||!checks.every(Boolean)} onClick={()=>void submit()}>{busy?'正在开始…':asking==='start'?'确认并启动':'确认并停止'}</button></section></div>}
 </div>;
}
