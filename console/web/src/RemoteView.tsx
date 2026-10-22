import {useState} from 'react';
import {StethoscopeIcon} from '@phosphor-icons/react';
import type {Capability,Snapshot,MotionControlReview} from './types';
import CurrentPosture from './CurrentPosture';
import DampingTest from './DampingTest';
import OfficialRemoteManual from './OfficialRemoteManual';
import MotionModeReview from './MotionModeReview';
import RemoteOperations from './RemoteOperations';
import ControllerReadiness from './ControllerReadiness';
import Disclosure from './Disclosure';

type Props={review:MotionControlReview;state:Snapshot;fresh:boolean;held:boolean;owner:string;clock:number;takeControl:()=>void;arm:()=>Promise<unknown>;disarm:()=>Promise<unknown>;command:(id:string,params?:Record<string,unknown>,ack?:boolean)=>Promise<unknown>;choose:(cap:Capability)=>void;report:(text:string)=>void};

export default function RemoteView({review,state,fresh,held,takeControl,command,choose,report}:Props){
 const [busy,setBusy]=useState(false);
 const verifyOnly=!!state.manual_remote?.verify_only;
 const accessoryTest=!!state.manual_remote?.accessory_test;
 const dampingTest=!!state.manual_remote?.damping_test;
 const remote=state.manual_remote;
 return <div className="remote-workspace diagnostic-workspace">
  <section className="surface diagnostic-intro"><StethoscopeIcon size={26}/><div><h2>只读诊断</h2><p>驾驶请用『驾驶』页；本页只读诊断。</p></div></section>
  <ControllerReadiness state={state} fresh={fresh} review={review} choose={choose} command={command}/>

  <RemoteOperations state={state} fresh={fresh} held={held} choose={choose} showOperations={false}/>
  <Disclosure title="当前身体判断" summary="查看姿态、支撑与反馈证据；未知项继续保持未知。"><CurrentPosture state={state} fresh={fresh}/></Disclosure>
  <Disclosure title="官方遥控手册 · 23 自由度" summary="查看官方按键、固件核对与起身适用条件。"><OfficialRemoteManual review={review}/><MotionModeReview review={review}/></Disclosure>
  <Disclosure title="遥控功能对应关系" summary="查看遥控功能、软件接口和已知安全边界。"><section className="surface remote-reference"><div className="panel-title"><h2>遥控功能对应关系</h2><a href="https://support.unitree.com/home/en/G1_developer/remote_control" target="_blank" rel="noreferrer">官方遥控说明 ↗</a></div><div className="table-scroll"><table><thead><tr><th>功能</th><th>软件接口 / 状态</th><th>适用边界</th></tr></thead><tbody><tr><td>摇杆移动、转向与停止</td><td>SetVelocity / StopMove</td><td>需人工授权与新鲜 FSM 500；不负责起身</td></tr><tr><td>手臂交互手势</td><td>ExecuteAction</td><td>限本机声明且已核查 SDK ID 的子集</td></tr><tr><td>音量、语音、灯光</td><td>官方 AudioClient</td><td>取得控制权后按需执行</td></tr><tr><td>起身、锁定站立、阻尼、零力矩</td><td>阻尼已完成一次状态切换测试；其余未实测</td><td>一次测试凭据禁止重发；其他模式适用条件尚未核实</td></tr><tr><td>自适应起身、舞蹈、步态切换</td><td>新版遥控手册已收录；SDK 映射未齐</td><td>不能将遥控按键或示例值直接当成实机调用</td></tr><tr><td>遥控阻尼保护 / 硬件急停</td><td>网页停止仅发送零速度</td><td>遥控阻尼保护不是已核定的硬件断电急停；失联策略未知</td></tr></tbody></table></div></section></Disclosure>
  {(verifyOnly||accessoryTest||dampingTest)&&<Disclosure title="专项验证与测试工具" summary="仅在对应启动模式展示；控制权、门控和实机确认规则保持不变。"><section className="surface remote-verification"><div className="panel-title"><h2>人工遥控链路验证</h2><span className="minor">{dampingTest?'仅开放单次官方阻尼':accessoryTest?'动作锁定 · 仅开放语音和 RGB 灯':'不下发实机指令'}</span></div><p>先取得本窗口控制权，再打开接口填写参数并核对。控制权仅限本项目窗口互斥，官方控制器的运动权限尚未验证。</p><div className="settings-buttons"><button className={held?'secondary':'primary'} disabled={!fresh||!!state.control.owner&&!held} onClick={takeControl}>{held?'释放本窗口控制权':'取得本窗口控制权'}</button><span className="minor">{held?`心跳有效 · 租约剩余 ${state.control.expires_in.toFixed(1)} 秒`:state.control.owner?'其他窗口持有控制权':'尚无窗口控制权'}</span></div><div className="verification-cards">{[['起身接口',['squat_to_stand','lie_to_stand']],['姿态保持接口',['prepare','stand_height']],['平衡切换接口',['balance','start']]].map(([title,ids])=><div className="verification-card" key={title as string}><h3>{title}</h3>{(ids as string[]).map(id=>{const cap=state.capabilities.find(c=>c.id===id)!;return <button className="secondary" disabled={!held||!fresh} key={id} onClick={()=>choose(cap)}>{cap.label} · 核对</button>;})}</div>)}</div><p className="panel-note">当前姿态的适用条件、承重与实际运动固件仍有未知；接口核对通过不代表固件接受或动作完成。</p>{!!remote?.preflights?.length&&<details><summary>本次服务接口核对记录 · 未下发</summary><div className="table-scroll"><table><thead><tr><th>接口</th><th>官方请求</th><th>核对结果</th></tr></thead><tbody>{remote.preflights.slice(0,8).map((p,i)=><tr key={p.at+':'+i}><td>{p.label}<small>{new Date(p.at*1000).toLocaleTimeString('zh-CN',{hour12:false})} · 历史样本</small></td><td>API {p.request.api_id}<code>{JSON.stringify(p.request.parameter)}</code></td><td>参数 / 映射通过<small>固件接受未知 · 未执行</small></td></tr>)}</tbody></table></div></details>}</section>
   {dampingTest&&<DampingTest state={state} fresh={fresh} held={held} command={command} report={report}/>}
   {accessoryTest&&<section className="surface remote-verification" aria-label="官方声光实机测试"><div className="panel-title"><h2>官方声光实机测试</h2><span className="tag">电机与模式切换保持锁定</span></div><p>取得本窗口控制权后，可发送一次短语音或设置 RGB 灯。官方接口应答与实际播放、亮灯分开记录；发生超时不会自动重发。</p><div className="settings-buttons"><button className="primary" disabled={busy||!fresh||!held||!state.capabilities.find(c=>c.id==='tts')?.execution_gate?.allowed} onClick={async()=>{setBusy(true);try{const result=await command('tts',{text:'软件遥控器连接测试，机器人保持原状。'});report((result as {message?:string})?.message||'语音请求已返回，实际播放结果待确认');}catch(error){report(error instanceof Error?error.message:'请求失败');}finally{setBusy(false);}}}>{busy?'等待官方应答…':'播放一次连接测试语音'}</button><button className="secondary" disabled={!fresh||!held} onClick={()=>{const cap=state.capabilities.find(c=>c.id==='led');if(cap)choose(cap);}}>设置 RGB 灯…</button></div><p className="panel-note">无需确认站立；不会因此开放起身、阻尼、平衡切换、音量修改或音频流。</p></section>}
  </Disclosure>}
 </div>;
}
