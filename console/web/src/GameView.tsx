import {useCallback,useEffect,useMemo,useRef,useState} from 'react';
import {GameControllerIcon,ShieldCheckIcon,SquareIcon,XIcon} from '@phosphor-icons/react';
import CameraView from './CameraView';
import {Deadman} from './deadman';
import {watchGamepad,type GameInput} from './gamepad';
import PowerSequence from './PowerSequence';
import type {Snapshot} from './types';

type Props={state:Snapshot;fresh:boolean;held:boolean;owner:string;clock:number;takeControl:()=>Promise<unknown>|void;arm:()=>Promise<unknown>;disarm:()=>Promise<unknown>;command:(id:string,params?:Record<string,unknown>,ack?:boolean)=>Promise<unknown>;report:(text:string)=>void};
type Source='joystick'|'keyboard'|'gamepad';
const ZERO:GameInput={vx:0,vy:0,omega:0};
const GEARS=[.5,1,2] as const;
const CONFIRMATIONS=['我在现场确认：机器人已自行站立','身体没有靠柜、被人托住或其他外部支撑','身体四周、手臂活动范围和移动方向均已清空','地面平整，已检查台阶、边缘和绊倒物'];
const GESTURES=[['wave','挥手'],['shake_start','伸手'],['shake_end','收手'],['wave_turn','挥手转身']] as const;
// Plain-language wording for the drive HUD; unknown codes fall back to the service's own message.
const BLOCKERS:Record<string,string>={NOT_WALKING_MODE:'机器人还没进入行走模式：先摆好起始姿态，点右侧“一键启动”'};
const magnitude=(input:GameInput)=>Math.hypot(input.vx,input.vy,input.omega);
const sameInput=(a:GameInput,b:GameInput)=>a.vx===b.vx&&a.vy===b.vy&&a.omega===b.omega;
const typing=(target:EventTarget|null)=>target instanceof HTMLElement&&(target.matches('input,textarea,select')||target.isContentEditable);

export default function GameView({state,fresh,held,owner,clock,takeControl,arm,disarm,command,report}:Props){
 const remote=state.manual_remote;
 const restricted=remote?.enabled!==true||!!remote.verify_only||!!remote.accessory_test||!!remote.damping_test;
 const authorized=!!remote?.armed&&remote.owner===owner&&held&&fresh&&!restricted;
 const canMove=authorized&&state.capabilities.find(cap=>cap.id==='velocity')?.execution_gate?.allowed===true;
 const remaining=Math.max(0,Math.ceil((remote?.expires_in||0)-Math.max(0,clock/1000-state.at)));
 const [gear,setGear]=useState(0),[checks,setChecks]=useState([false,false,false,false]);
 const [confirming,setConfirming]=useState(false),[busy,setBusy]=useState(false),[gestureBusy,setGestureBusy]=useState<string|null>(null);
 const [error,setError]=useState<string|null>(null),[sources,setSources]=useState<Record<Source,GameInput>>({joystick:ZERO,keyboard:ZERO,gamepad:ZERO});
 const [knob,setKnob]=useState({x:0,y:0});
 const callbacks=useRef({command,report});callbacks.current={command,report};
 const disarmRef=useRef(disarm);disarmRef.current=disarm;
 const canMoveRef=useRef(canMove);canMoveRef.current=canMove;
 const keys=useRef(new Set<string>()),pointer=useRef<number|null>(null);
 const driver=useMemo(()=>new Deadman(v=>callbacks.current.command('velocity',v),()=>callbacks.current.command('stop'),message=>{setError(message);callbacks.current.report(message);}),[]);
 const selected=useMemo(()=>Object.values(sources).reduce((best,input)=>magnitude(input)>magnitude(best)?input:best,ZERO),[sources]);
 const moving=magnitude(selected)>0;
 const velocity=useMemo(()=>({
  vx:Math.max(-.3,Math.min(.3,selected.vx*.15*GEARS[gear])),
  vy:Math.max(-.2,Math.min(.2,selected.vy*.1*GEARS[gear])),
  omega:Math.max(-.4,Math.min(.4,selected.omega*.2*GEARS[gear]))
 }),[selected,gear]);
 const setSource=useCallback((source:Source,input:GameInput)=>setSources(current=>sameInput(current[source],input)?current:{...current,[source]:input}),[]);
 const release=useCallback(()=>{
  keys.current.clear();pointer.current=null;setKnob({x:0,y:0});setSources({joystick:ZERO,keyboard:ZERO,gamepad:ZERO});driver.release();
 },[driver]);
 const fail=useCallback((value:unknown,fallback:string)=>{const message=value instanceof Error?value.message:fallback;setError(message);report(message);},[report]);
 const stop=useCallback(()=>{
  keys.current.clear();pointer.current=null;setKnob({x:0,y:0});setSources({joystick:ZERO,keyboard:ZERO,gamepad:ZERO});
  if(!driver.release())void command('stop').catch(value=>fail(value,'停止回执未确认；请观察机器人'));
 },[command,driver,fail]);
 const end=useCallback(()=>{release();if(authorized)void disarmRef.current().catch(value=>fail(value,'结束驾驶失败'));},[authorized,release,fail]);
 const shift=useCallback((change:-1|1)=>setGear(value=>Math.max(0,Math.min(2,value+change))),[]);
 // Window and gamepad listeners subscribe once; telemetry re-renders must never reset held-input state.
 const actions=useRef({stop,end});actions.current={stop,end};

 useEffect(()=>{if(canMove)driver.update(velocity);else driver.release();},[canMove,driver,velocity]);
 useEffect(()=>{if(!canMove)release();},[canMove,release]);
 useEffect(()=>{
  const blur=()=>release(),visibility=()=>{if(document.hidden)release();};
  window.addEventListener('blur',blur);document.addEventListener('visibilitychange',visibility);
  return()=>{window.removeEventListener('blur',blur);document.removeEventListener('visibilitychange',visibility);release();};
 },[release]);
 useEffect(()=>watchGamepad({onInput:input=>setSource('gamepad',input),onStop:()=>actions.current.stop(),onGear:shift,onDisarm:()=>actions.current.end(),onDisconnect:release}),[release,setSource,shift]);
 useEffect(()=>{
  const updateKeys=()=>setSource('keyboard',{
   vx:(keys.current.has('KeyW')?1:0)-(keys.current.has('KeyS')?1:0),
   vy:(keys.current.has('KeyQ')?1:0)-(keys.current.has('KeyE')?1:0),
   omega:(keys.current.has('KeyA')?1:0)-(keys.current.has('KeyD')?1:0)
  });
  const down=(event:KeyboardEvent)=>{
   if(typing(event.target)||event.metaKey||event.ctrlKey||event.altKey)return;
   if(event.code==='Space'){event.preventDefault();if(!event.repeat)actions.current.stop();return;}
   if(event.code==='Escape'){event.preventDefault();event.stopImmediatePropagation();if(!event.repeat)actions.current.end();return;}
   if(['Digit1','Digit2','Digit3'].includes(event.code)){event.preventDefault();if(!event.repeat)setGear(Number(event.code.at(-1))-1);return;}
   if(!canMoveRef.current||!['KeyW','KeyS','KeyA','KeyD','KeyQ','KeyE'].includes(event.code))return;
   event.preventDefault();keys.current.add(event.code);updateKeys();
  };
  const up=(event:KeyboardEvent)=>{if(keys.current.delete(event.code)){event.preventDefault();updateKeys();}};
  window.addEventListener('keydown',down,{capture:true});window.addEventListener('keyup',up);
  return()=>{window.removeEventListener('keydown',down,{capture:true});window.removeEventListener('keyup',up);keys.current.clear();setSource('keyboard',ZERO);};
 },[setSource]);

 const openConfirm=()=>{setChecks([false,false,false,false]);setError(null);setConfirming(true);};
 async function enable(){
  setBusy(true);setError(null);
  try{if(!held)await takeControl();await arm();setChecks([false,false,false,false]);setConfirming(false);}
  catch(value){setChecks([false,false,false,false]);fail(value,'开始驾驶失败');}
  finally{setBusy(false);}
 }
 async function gesture(id:string){
  if(moving)return;
  setGestureBusy(id);setError(null);
  try{await command(id);}
  catch(value){fail(value,'手势请求失败');}
  finally{setGestureBusy(null);}
 }
 const status=!state.connected||!fresh?'连接中断，不能驾驶':restricted?'当前服务不是驾驶范围':remote?.blocker?(BLOCKERS[remote.blocker.code]??remote.blocker.message):(!authorized?'准备就绪，点开始驾驶':`驾驶中 · 剩余 ${remaining} 秒`);
 const telemetry=!state.connected||!fresh?'遥测过期':'遥测新鲜';
 const joystick=(event:React.PointerEvent<HTMLDivElement>)=>{
  if(pointer.current!==event.pointerId)return;
  const box=event.currentTarget.getBoundingClientRect(),radius=box.width/2;
  let x=(event.clientX-(box.left+radius))/radius,y=(event.clientY-(box.top+radius))/radius;
  const length=Math.hypot(x,y);if(length>1){x/=length;y/=length;}
  const input={vx:Math.abs(y)<.15?0:-y,vy:0,omega:Math.abs(x)<.15?0:-x};
  setKnob({x,y});setSource('joystick',input);
 };
 const releaseJoystick=()=>{pointer.current=null;setKnob({x:0,y:0});setSource('joystick',ZERO);};

 return <section className="game-view" aria-label="游戏式驾驶">
  <div className="game-camera" aria-hidden="true"><CameraView state={state} transport={fresh} clock={clock}/></div>
  <div className="game-shade"/>
  <header className={'game-status '+(authorized?'driving':'')} aria-live="polite"><span className="game-status-dot"/><div><b>{status}</b><small>FSM {state.assessment.state_learning.control_mode.sdk_label??'未知'}{state.telemetry.fsm!==null?` · ${state.telemetry.fsm}`:''} · {telemetry}</small></div>{authorized&&remaining<=10&&<span className="game-expiry">即将到期</span>}</header>
  <div className="game-top-actions">
   <div className="game-gears" aria-label="速度档位">{GEARS.map((scale,index)=><button key={scale} className={gear===index?'active':''} onClick={()=>setGear(index)}><b>{index+1}</b><span>{scale}×</span></button>)}</div>
   {!authorized?<button className="game-start" disabled={!fresh||restricted||!!remote?.blocker||!!state.control.owner&&!held} onClick={openConfirm}><GameControllerIcon size={22} weight="fill"/>开始驾驶</button>:<div className="game-session-actions">{remaining<=10&&<button className="game-renew" onClick={openConfirm}>续期</button>}<button className="game-end" onClick={end}>结束驾驶</button></div>}
  </div>
  <PowerSequence state={state} held={held} owner={owner} takeControl={takeControl} command={command} report={report} onStarted={openConfirm}/>
  <div className="game-controls">
   <div className="game-stick-wrap"><div className="game-stick-label">移动 / 转向</div><div className={'game-stick '+(!canMove?'disabled':'')} role="application" aria-label="虚拟摇杆" onPointerDown={event=>{if(!canMove||event.button!==0)return;pointer.current=event.pointerId;event.currentTarget.setPointerCapture(event.pointerId);joystick(event);}} onPointerMove={joystick} onPointerUp={releaseJoystick} onPointerCancel={()=>{releaseJoystick();release();}} onLostPointerCapture={()=>{if(pointer.current!==null){releaseJoystick();release();}}}><i/><span style={{transform:`translate(calc(-50% + ${knob.x*62}px), calc(-50% + ${knob.y*62}px))`}}><GameControllerIcon size={28}/></span></div><small>W/S 前后 · A/D 转向 · Q/E 平移</small></div>
   <button className="game-stop" onClick={stop}><SquareIcon size={34} weight="fill"/><b>停止</b><span>SPACE</span></button>
  </div>
  <div className="game-gestures" aria-label="手势">{GESTURES.map(([id,label])=><button key={id} disabled={!authorized||moving||gestureBusy!==null} onClick={()=>void gesture(id)}><span>{gestureBusy===id?'发送中':label}</span><small>{id}</small></button>)}</div>
  {error&&<div className="game-error" role="alert">{error}</div>}
  <div className="game-input-hint">键盘 · 虚拟摇杆 · 浏览器手柄</div>
  {confirming&&<div className="game-confirm-backdrop" role="presentation"><section className="game-confirm" role="dialog" aria-modal="true" aria-labelledby="game-confirm-title"><header><div><span>现场确认</span><h2 id="game-confirm-title">{authorized?'重新确认并续期':'开始驾驶前请逐项确认'}</h2></div><button aria-label="关闭" onClick={()=>{setConfirming(false);setChecks([false,false,false,false]);}}><XIcon size={20}/></button></header><p>每次开始或续期都必须重新确认；勾选不会被保存。</p><div className="game-confirm-list">{CONFIRMATIONS.map((text,index)=><label key={text}><input type="checkbox" checked={checks[index]} disabled={busy} onChange={event=>setChecks(current=>current.map((value,item)=>item===index?event.target.checked:value))}/><span><ShieldCheckIcon size={18}/>{text}</span></label>)}</div>{error&&<div className="game-confirm-error">{error}</div>}<button className="game-confirm-submit" disabled={busy||!checks.every(Boolean)} onClick={()=>void enable()}>{busy?'正在取得授权…':'确认并开始'}</button></section></div>}
 </section>;
}
