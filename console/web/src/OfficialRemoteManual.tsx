import type {MotionControlReview} from './types';

export default function OfficialRemoteManual({review}:{review:MotionControlReview}){
 const manual=review.official_manual,result=review.motor_test_contract?.device_result;
 if(!manual)return null;
 return <section className="surface remote-reference official-remote-manual" aria-label="官方遥控手册与实机验证">
  <div className="panel-title"><h2>官方遥控手册 · 23 自由度</h2><span className="tag">{manual.edition}</span></div>
  <p>{manual.meaning} 适用分支：{manual.branch_condition}；本机运行构建尚未核定，分支未自动选用。</p>
  {result&&<div className="motion-review-summary"><b>已完成一次电机模式实测 · 历史记录</b><p>{result.summary}</p><p>SDK {result.sdk_code}：{result.acknowledgement}。发送时间：{new Date(result.sent_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间。</p><p>完整 {result.trace.duration_s.toFixed(1)} 秒窗口、{result.trace.samples} 次采样：最大关节速度 {result.trace.max_abs_dq_rad_s.toFixed(3)} rad/s，较起点最大角度变化 {(result.trace.max_abs_q_delta_rad*180/Math.PI).toFixed(1)}°（{result.trace.max_motion_joint}）；变化原因未知，不视作站稳证据。</p><small>此记录不代表当前机器人状态。机器人关机或断连后，当前姿态和 FSM 保持未知。</small></div>}
  <details className="motion-review-details" open><summary>模式按键与软件入口</summary><div className="table-scroll"><table><thead><tr><th>官方功能</th><th>遥控按键 / 时长</th><th>已核对 SDK / 实机范围</th></tr></thead><tbody>{manual.controls.map(c=><tr key={c.id}><td>{c.name}</td><td>{c.keys}<br/>{c.press}</td><td>{c.sdk}<br/><small>{c.verified}</small></td></tr>)}</tbody></table></div></details>
  <details className="motion-review-details"><summary>官方启动路线与保护操作</summary><div className="table-scroll"><table><thead><tr><th>起点</th><th>手册路线</th><th>适用条件</th></tr></thead><tbody>{manual.routes.map(r=><tr key={r.start}><td>{r.start}</td><td>{r.steps}</td><td>{r.condition}</td></tr>)}</tbody></table></div><ul>{manual.protections.map(p=><li key={p.name}><b>{p.name}：</b>{p.operation}。{p.meaning}</li>)}</ul></details>
  <p className="panel-note">{manual.scope}</p><ul>{manual.unknowns.map(u=><li key={u}>{u}</li>)}</ul>
  <div className="official-manual-sources"><a href={manual.pdf_url} target="_blank" rel="noreferrer">宇树官方 23 自由度手册 ↗</a> · <a href={manual.catalog_url} target="_blank" rel="noreferrer">官方版本目录 ↗</a></div>
 </section>;
}
