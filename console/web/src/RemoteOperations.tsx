import {useState} from 'react';
import {CaretRightIcon,LockKeyIcon} from '@phosphor-icons/react';
import type {Capability,Snapshot} from './types';
import ServiceReadback from './ServiceReadback';
import {latestQueryReadback,queryReadbackLabel} from './queryReadback';
const STATUS:Record<string,string>={read_result:'已读取 · 历史样本',read_unavailable:'查询不可用 · 值未知',sdk_ack:'接口已应答 · 完成未知',awaiting_feedback:'已应答 · 等待 FSM',state_observed:'目标 FSM 已观测 · 站稳未知',feedback_timeout:'FSM 核验超时',feedback_interrupted:'FSM 核验中断',rejected:'已拒绝',outcome_unknown:'调用异常 · 执行结果未知'};
const TABS=['模式与起身','手臂任务','声光与音频','只读查询','高级控制'];
function category(cap:Capability){return cap.risk==='read'?'只读查询':cap.risk==='accessory'?'声光与音频':cap.group==='高级'?'高级控制':['手臂','官方手势'].includes(cap.group)?'手臂任务':'模式与起身';}
export default function RemoteOperations({state,fresh,held,choose,showOperations=true}:{state:Snapshot;fresh:boolean;held:boolean;choose:(c:Capability)=>void;showOperations?:boolean}){
 const [tab,setTab]=useState(TABS[0]),[search,setSearch]=useState('');
 const all=state.capabilities.filter(c=>!['velocity','stop'].includes(c.id));
 const caps=all.filter(c=>(search||category(c)===tab)&&`${c.label} ${c.method}`.toLowerCase().includes(search.toLowerCase()));
 const history=state.command_history||[];
 return <>
  {showOperations&&<section className="surface remote-operations"><div className="panel-title"><div><h2>官方操作面板</h2><small>全部入口可查看参数；发送前由后端复核实机条件</small></div><input aria-label="搜索遥控功能" placeholder="搜索模式、接口、语音…" value={search} onChange={e=>setSearch(e.target.value)}/></div>
   <div className="remote-tabs" role="group" aria-label="操作分类">{TABS.map(t=><button key={t} className={tab===t&&!search?'selected':''} aria-pressed={tab===t&&!search} onClick={()=>{setTab(t);setSearch('');}}>{t}<small>{all.filter(c=>category(c)===t).length}</small></button>)}</div>
   <div className="remote-operation-grid">{caps.map(cap=>{const readback=cap.risk==='read'?latestQueryReadback(history,cap.id,state.mode):null;const allowed=fresh&&cap.execution_gate?.allowed&&(cap.risk==='read'||held&&!state.read_only);const message=!fresh?'等待新鲜连接':state.read_only&&cap.risk!=='read'?'只读连接':cap.risk!=='read'&&!held?'需取得控制权':cap.execution_gate?.message||'尚未核实';return <article className="remote-operation" key={cap.id} data-capability={cap.id}><div><span className={'tag '+(allowed?'success':'')}>{allowed?cap.risk==='read'?'可发起查询':'可发送':'当前锁定'}</span>{!allowed&&<LockKeyIcon size={15}/>}</div><h3>{cap.label}</h3><code>{cap.method}</code><p>{cap.note||'复用宇树官方服务；接口存在不证明当前姿态适用。'}</p><small>{allowed?cap.risk==='read'?'按需查询，不自动轮询':'发送后查看下方应答记录':message}</small>{cap.risk==='read'&&<div className="remote-query-readback" aria-live="polite">{readback?<><b>最近查询 · {new Date(readback.at*1000).toLocaleTimeString('zh-CN',{hour12:false})}</b><small>{queryReadbackLabel(readback)}</small><p>{readback.message}</p></>:<small>本会话尚无查询回执；实时可用性未知。</small>}</div>}<button className="secondary" disabled={cap.risk==='read'&&!allowed} onClick={()=>choose(cap)}>{cap.risk==='read'?'读取返回值':'查看参数与发送'}<CaretRightIcon size={14}/></button></article>;})}</div>{!caps.length&&<p className="empty-text">没有匹配的官方操作。</p>}
  </section>}
  <section className="surface remote-results"><div className="panel-title"><h2>指令与反馈</h2><span className="minor">最近 60 项 · 不自动重发</span></div><p className="panel-note">接口应答、目标状态机回读、实际完成分开判断。速度续发不逐帧占用记录；异常会保留。此处查询值是带时间的历史样本，不能代替实时身体反馈。</p>
   {history.length?<div className="remote-result-list" aria-live="polite">{history.map(row=><article className={'remote-result '+row.status} key={row.request_id}><div><b>{row.label}</b><time>{new Date(row.at*1000).toLocaleTimeString('zh-CN',{hour12:false})}</time><span className="tag">{STATUS[row.status]||row.status}</span><small>{row.mode==='demo'?'合成演示':'实机服务'}{row.sdk_code!=null?` · 返回码 ${row.sdk_code}`:''}</small></div><p>{row.message}</p>{row.capability==='get_services'&&row.status==='read_result'&&<ServiceReadback data={row.data}/>}<details><summary>请求与有效返回值</summary><small>请求 {row.request_id}{row.error_code?` · ${row.error_code}`:''}</small><pre>{JSON.stringify(row.data??null,null,2)}</pre></details></article>)}</div>:<p className="empty-text">尚未发送操作。打开页面不会取得控制权、授权运动或调用动作。</p>}
  </section>
 </>;
}
