import type {Snapshot} from './types';
export default function CurrentPosture({state,fresh}:{state:Snapshot;fresh:boolean}){
 const learned=state.assessment.state_learning;
 const current=fresh&&state.mode==='robot'&&learned.current;
 const posture=learned.posture,control=learned.control_mode,window=learned.observation_window;
 const degrees=(values:number[]|null)=>values?.map(v=>`${v.toFixed(1)}°`).join(' / ')||'未知';
 return <section className="surface remote-posture" aria-label="自主身体状态判断">
  <div className="panel-title"><h2>当前身体判断</h2><span className="tag">{current?'实时传感器证据':'等待新鲜实机证据'}</span></div>
  <div className="remote-status-grid">
   <div><span>关节姿态</span><b>{current?posture.description:'未知'}</b><small>左右膝：{current?degrees(posture.knee_deg):'未知'}</small></div>
   <div><span>官方控制状态</span><b>{current?`FSM ${control.fsm??'未知'} · ${control.sdk_label||'未核定'}`:'未知'}</b><small>正在执行的官方能力尚未确认</small></div>
   <div><span>站稳与承重</span><b>尚未确认自行站稳</b><small>双脚接触、承重及背部支撑均未知</small></div>
  </div>
  <p className="panel-note">{current?`机身 roll / pitch / yaw：${degrees(posture.imu_rpy_deg)}；观察 ${window.seconds.toFixed(1)} 秒，最大关节速度 ${window.max_abs_dq_rad_s?.toFixed(4)??'未知'} rad/s。`:'反馈过期时不沿用上次姿态。'}静止和 FSM 数字不能单独证明双脚承重或平衡。</p>
  <p className="panel-note">尚未取得覆盖脚底与背部接触面的有效视觉或受力证据。历史上的“靠柜坐地”不会自动视为当前事实；承重未知时，不自动触发起身或阻尼。</p>
 </section>;
}
