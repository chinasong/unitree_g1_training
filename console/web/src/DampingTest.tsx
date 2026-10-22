import {useState} from 'react';
import type {Snapshot} from './types';

type Props={state:Snapshot;fresh:boolean;held:boolean;command:(id:string,params?:Record<string,unknown>,ack?:boolean)=>Promise<unknown>;report:(message:string)=>void};

export default function DampingTest({state,fresh,held,command,report}:Props){
 const [busy,setBusy]=useState(false);
 const attempt=state.manual_remote?.damping_attempt;
 const feedback=state.manual_remote?.damping_feedback;
 const gate=state.capabilities.find(c=>c.id==='damp')?.execution_gate;
 const labels:Record<string,string>={outcome_unknown:'已尝试 · 应答未确认执行结果',accepted:'官方接口接受',rejected:'官方接口拒绝',fsm_verified:'接口接受后曾回读 FSM 1'};
 const feedbackLabels:Record<string,string>={target_observed:'双路当前回读 FSM 1 · 官方阻尼状态',other_state_observed:'双路当前回读其他状态',conflict:'双路状态冲突，不能确认',unknown:'当前证据缺失或过期，保持未知',not_attempted:'尚未测试'};
 async function execute(){
  setBusy(true);
  try{await command('damp',{},true);}
  catch(e){report(e instanceof Error?e.message:'阻尼测试结果未知；不会重发');}
  finally{setBusy(false);}
 }
 return <section className="surface remote-verification" aria-label="单次官方阻尼电机测试">
  <div className="panel-title"><h2>单次官方阻尼电机测试</h2><span className="tag">仅 FSM 0 → 1</span></div>
  <p>只在零力矩起点切换官方被动阻尼，不设置关节位置目标。此操作不会让机器人站起或自行站稳。</p>
  <p className="panel-note">提交前重新核验 23 关节、IMU、低速观察窗口、双路状态回读及实时相机。足底与外部支撑仍未知；这些检查不是防摔保证。</p>
  <div className="settings-buttons"><button className="primary" disabled={busy||!fresh||!held||!gate?.allowed||!!attempt} onClick={execute}>{busy?'等待官方应答…':attempt?'本次测试已占用，不允许重发':'执行一次官方阻尼 · FSM 0 → 1'}</button><span className="minor">{attempt?labels[attempt.status]||attempt.status:!held?'先取得本窗口控制权':gate?.message}</span></div>
  {attempt&&<><p><b>指令应答：</b>SDK {attempt.sdk_code??'未知'} · 源 FSM {attempt.source_fsm} → 目标 FSM {attempt.target_fsm}。{attempt.sdk_code===3104?'接收超时，未重发；回读结果另行判断。':''}</p><p><b>实时状态：</b>{fresh&&feedback?feedbackLabels[feedback.status]:'当前证据过期，保持未知'}。GetFsmId {fresh?feedback?.fsm_id??'未知':'未知'} / HG sport {fresh?feedback?.sport_fsm_id??'未知':'未知'}；不证明承重、起身能力或已站稳。</p></>}
  <p className="panel-note">一次尝试后全部写入锁定；超时、断连和恢复连接均不自动重发，不自动切回零力矩。起身、姿态保持和平衡切换尚未开放。</p>
 </section>;
}
