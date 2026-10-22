import type {Capability,Snapshot,MotionControlReview} from './types';
import {latestQueryReadback,queryReadbackLabel} from './queryReadback';
import LocoReadback from './LocoReadback';

export default function ControllerReadiness({state,fresh,review,choose,command}:{state:Snapshot;fresh:boolean;review:MotionControlReview;choose:(c:Capability)=>void;command:(id:string)=>Promise<unknown>}){
 const feedback=state.telemetry.safety_feedback;
 const mode=feedback?.motion_mode;
 const fsmMode=feedback?.fsm_mode;
 const versions=state.telemetry.service_versions;
 const archive=review.service_advertisement;
 const publisher=review.publisher_review;
 const reboot=review.reboot_readonly_audit;
 const time=(stamp:number)=>new Date(stamp*1000).toLocaleTimeString('zh-CN',{hour12:false});
 return <section className="surface controller-readiness">
  <div className="panel-title"><h2>实机控制就绪证据</h2><span className="tag">{state.mode==='demo'?'合成演示':'起身 / 平衡接受尚未验证'}</span></div>
  <p className="panel-note">窗口租约防止多个页面同时操作；G1 官方运动客户端未启用 SDK 租约。取得窗口控制权、silent=false 或服务开启，均不能证明机器人会接受起身或平衡切换。</p>
  <div className="controller-evidence-grid">
   <div><span>实时官方模式</span><b>{fresh&&mode?.current?`${String(mode.raw?.name)} / form ${String(mode.raw?.form)}`:'未知 / 回读过期'}</b><small>{fresh&&fsmMode?.current?`GetFsmMode ${String(fsmMode.raw?.data)} · ${fsmMode.age_ms} ms 前收到`:'GetFsmMode 未取得新鲜值'}</small></div>
   {['get_silent','get_balance','get_versions'].map(id=>{const row=latestQueryReadback(state.command_history||[],id,state.mode);const cap=state.capabilities.find(c=>c.id===id);return <div key={id}><span>{cap?.label||id}</span><b>{row?queryReadbackLabel(row):'尚未查询'}</b><small>{row?`${time(row.at)} · ${row.sdk_code??(row.status==='read_result'?'读取成功':'未知返回码')} · ${JSON.stringify(row.data??null)}`:'按需只读查询；不会切换模式'}</small>{cap&&<button className="subtle" disabled={!fresh} onClick={()=>choose(cap)}>读取当前值</button>}</div>;})}
  </div>
  <LocoReadback state={state} fresh={fresh} command={command}/>
  {review.installed_sdk_audit&&<details className="version-listener"><summary>官方 SDK 初始化核验 · 历史证据</summary>
   <p>{new Date(review.installed_sdk_audit.checked_at_utc).toLocaleString('zh-CN',{hour12:false})}：{review.installed_sdk_audit.summary}</p>
   <p>构造发送 {review.installed_sdk_audit.constructor_rpc_requests} 次 RPC，Init 发送 {review.installed_sdk_audit.init_rpc_requests} 次 RPC。Init 只注册本地接口，不负责初始化运动服务内的 LocoState。</p>
   <p>{review.installed_sdk_audit.readiness}</p><p className="panel-note">{review.installed_sdk_audit.value_limits}</p>
  </details>}
  {reboot&&<details className="version-listener"><summary>充电重启后的接口核验 · {reboot.reads.length} 次只读查询</summary>
   <p>{new Date(reboot.checked_at_utc).toLocaleString('zh-CN',{hour12:false})} 至 {new Date(reboot.completed_at_utc).toLocaleTimeString('zh-CN',{hour12:false})}；历史快照，不替代上方实时回读。</p>
   <p>接收超时 3104：{reboot.reads.filter(r=>r.code===3104).length} 次；LocoState 不可用 7301：{reboot.reads.filter(r=>r.code===7301).length} 次；未实现 3203：{reboot.reads.filter(r=>r.code===3203).length} 次。没有请求控制租约或发送动作。</p>
   <div className="table-scroll"><table><thead><tr><th>官方只读接口</th><th>三轮返回码</th><th>解释</th></tr></thead><tbody>{reboot.reads.filter(r=>r.trial===0).map(r=><tr key={`${r.service}-${r.api_id}`}><td>{r.service} / API {r.api_id}</td><td>{reboot.reads.filter(q=>q.service===r.service&&q.api_id===r.api_id).map(q=>q.code).join(' / ')}</td><td>{r.code!==0?'读取值未知':r.api_id===1?`${r.raw} · 服务协议版本`:`${r.raw}`}</td></tr>)}</tbody></table></div>
   <p className="panel-note">核验期间 FSM {reboot.before.fsm??'未知'} → {reboot.after.fsm??'未知'}；运行固件和 7301 的内部就绪判定仍未知。不能通过重发模式切换来试探是否恢复。</p>
  </details>}
  <details className="version-listener"><summary>运动固件版本 · {versions?.rows.length?'收到服务版本广告，运行构建待核验':'未知'}</summary>
   <p>{fresh&&versions?.listening?`持续只读监听 ${versions.topic} · 本次连接收到 ${versions.frames} 帧`:'版本监听未连接'}{versions?.received_at?` · 最近接收 ${time(versions.received_at)}`:''}</p>
   <small>发布端使用 Volatile，不向后加入的订阅者补发旧帧；等待新发布。接收时间不是固件安装时间，服务版本广告也不自动等于正在运行的运动构建。</small>
   {!!versions?.rows.length&&<div className="table-scroll"><table><thead><tr><th>广告中的服务</th><th>广告版本</th><th>原始字段路径</th></tr></thead><tbody>{versions.rows.map((r,i)=><tr key={i}><td>{r.name}</td><td>{r.version}</td><td><code>{r.path}</code></td></tr>)}</tbody></table></div>}
   {versions&&['invalid_frame','no_matching_fields','reader_unavailable'].includes(versions.status)&&<p>监听状态：{versions.status}；未填入运动固件版本。</p>}
   {archive&&<p className="panel-note">基础档案中的历史公告（{new Date(archive.received_at_utc).toLocaleString('zh-CN',{hour12:false})}）：{archive.rows.filter(r=>r.name==='ai_sport').map(r=>`${r.name} = ${r.version}`).join('、')}。仅为版本线索，未确认运行构建；重连后仍保留，不能当作当前状态或动作授权。</p>}
  </details>
  {publisher&&<details className="version-listener"><summary>运动板发布来源核验 · 历史采样 {publisher.duration_s.toFixed(0)} 秒</summary>
   <p>{publisher.summary}</p><small>{new Date(publisher.checked_at_utc).toLocaleString('zh-CN',{hour12:false})} 至 {new Date(publisher.completed_at_utc).toLocaleTimeString('zh-CN',{hour12:false})}；不是当前实时状态。</small>
   <div className="table-scroll"><table><thead><tr><th>证据</th><th>来源 IP / 自报进程</th><th>实测结果</th></tr></thead><tbody>
    {publisher.advertisements.map((r,i)=><tr key={`ad-${i}`}><td>服务版本公告</td><td>{r.source_ip}<br/>{r.self_reported.__ProcessName} · PID {r.self_reported.__Pid}</td><td>{r.rows.filter(v=>v.name==='ai_sport').map(v=>`${v.name} ${v.version}`).join('、')}；运行构建未确认</td></tr>)}
    {publisher.states.map((r,i)=><tr key={`state-${i}`}><td>HG 状态流</td><td>{r.source_ip}<br/>{r.self_reported.__ProcessName} · PID {r.self_reported.__Pid}</td><td>{r.samples} 帧；{JSON.stringify(r.values)}；任务编号语义未知</td></tr>)}
    {publisher.queries.filter(r=>r.phase==='after_service_advertisement').map((r,i)=><tr key={`rpc-${i}`}><td>只读 API {r.api_id}</td><td>{r.source_ip}<br/>{r.self_reported.__ProcessName} · PID {r.self_reported.__Pid}</td><td>返回 {r.code} · {r.code===0?r.data:'值未知'}</td></tr>)}
   </tbody></table></div><p>{publisher.readiness}</p>
   {publisher.limits.map(s=><p className="panel-note" key={s}>{s}</p>)}
  </details>}
  <details className="version-listener"><summary>起身、姿态保持、平衡切换 · 尚缺的适用条件</summary>
   <div className="table-scroll"><table><thead><tr><th>功能 / 官方接口候选</th><th>进入条件</th><th>仍待确认</th></tr></thead><tbody>{review.modes.filter(r=>['lie','squat','prepare','balance'].includes(r.id)).map(r=><tr key={r.id}><td>{r.name}<br/><code>{r.documented_entry}</code></td><td>{r.entry_conditions}</td><td>{r.remaining_unknown}</td></tr>)}</tbody></table></div>
   <p className="panel-note">历史报告中的靠柜坐地没有已核定的起身入口；当前姿态以实时身体判断为准，起身时长和速度参数未知。未合并社区 PR 的 1→4→501 路线未获得本机适用证据，未加入自动切换。</p>
  </details>
  <p className="panel-note">7301 表示 LocoState 不可用，当前固件内部判定尚未公开；3203 表示该服务未实现查询 API。当前证据不支持将重连或窗口租约作为修复方法。当前姿态适用的起身路线与承重条件未核实，动作继续锁定。</p>
 </section>;
}
