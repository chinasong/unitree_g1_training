import type {MotionControlReview} from './types';
import ServiceReadback from './ServiceReadback';

export default function MotionModeReview({review:r}:{review:MotionControlReview}){
 return <section className="surface remote-reference motion-review" aria-label="官方起身与模式核对">
  <div className="panel-title"><h2>起身、姿态保持与平衡切换</h2><span className="tag">只读核对 · 切换锁定</span></div>
  <div className="motion-review-summary"><b>人工报告起点：靠柜坐地 · 非实时姿态</b><p>{r.conclusion}</p><small>资料核对：{new Date(r.checked_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间；此档案不是实时承重或动作权限。</small></div>
  <details className="motion-review-details" open>
   <summary>humanoid 两条状态链路 · 官方读取入口覆盖</summary>
   <p><b>{r.humanoid_review.summary}</b></p>
   <small>历史并发采样：{new Date(r.humanoid_review.checked_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间；接收时间不等于传感器时间。</small>
   <div className="table-scroll"><table><thead><tr><th>官方 HG 状态通道</th><th>有效帧 / 接收窗口</th><th>原始模式与任务值</th></tr></thead><tbody>{r.humanoid_review.channels.map(c=><tr key={c.topic}><td>{c.topic}<br/>{c.source_ip}</td><td>{c.samples} 帧<br/>{c.first_receipt_s.toFixed(3)}–{c.last_receipt_s.toFixed(3)} 秒</td><td><pre className="official-source">{JSON.stringify(c.values,null,2)}</pre></td></tr>)}</tbody></table></div>
   <p>同一窗口固定查询：{r.humanoid_review.queries.map(q=>`${q.api_id} → ${q.code}（${q.elapsed_ms} ms）`).join('；')}。7301 的值保持未知，不使用错误占位。</p>
   <div className="table-scroll"><table><thead><tr><th>核查项目</th><th>实际证据</th><th>仍不能推导的结论</th></tr></thead><tbody>{r.humanoid_review.findings.map(f=><tr key={f.name}><td>{f.name}</td><td>{f.evidence}</td><td>{f.meaning}</td></tr>)}</tbody></table></div>
   <details><summary>保留两条 DDS 来源 · 自报信息不是固件证明</summary>{r.humanoid_review.participants.map(p=><article key={p.key}><b>{p.path}</b><p>{p.key}</p><pre className="official-source">{JSON.stringify(p.self_reported,null,2)}</pre></article>)}</details>
   <h3>软件遥控器的官方读取入口</h3>
   <div className="table-scroll"><table><thead><tr><th>入口</th><th>覆盖状态</th><th>使用条件与缺口</th></tr></thead><tbody>{r.humanoid_review.read_routes.map(route=><tr key={route.name}><td><b>{route.name}</b><br/>{route.route}</td><td>{route.status}</td><td>{route.condition}</td></tr>)}</tbody></table></div>
   <ul>{r.humanoid_review.limits.map(limit=><li key={limit}>{limit}</li>)}</ul>
  </details>
  <details className="motion-review-details" open>
   <summary>ai_sport 服务与应答来源 · 7301 条件证据</summary>
   <p>{r.service_readiness.summary}</p>
   <small>历史采样：{new Date(r.service_readiness.checked_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间；节点归属不随页面实时更新。</small>
   <p><b>观测到的应答发送节点：{r.service_readiness.observed_sender_ips.join(' / ')}</b></p>
   <p>{r.service_readiness.attribution}</p>
   <div className="table-scroll"><table><thead><tr><th>已核查项目</th><th>本机证据</th><th>判定边界</th></tr></thead><tbody>{r.service_readiness.findings.map(f=><tr key={f.name}><td>{f.name}</td><td>{f.evidence}</td><td>{f.meaning}</td></tr>)}</tbody></table></div>
   <details><summary>10 项固定查询的原始返回与来源</summary><div className="table-scroll"><table><thead><tr><th>服务 / API</th><th>返回码</th><th>有效返回值</th><th>发送节点</th></tr></thead><tbody>{r.service_readiness.queries.map(q=><tr key={`${q.service}:${q.api_id}`}><td>{q.service} / {q.api_id}</td><td>{q.code}</td><td>{q.code===0?q.api_id===1003?`${r.service_readiness.services.length} 项服务清单，见下表`:q.raw:'未知 · 错误占位未用作读数'}</td><td>{q.observed_sender_ips.join(' / ')}</td></tr>)}</tbody></table></div></details>
   <details><summary>历史服务清单 · 状态 0 开启 / 1 关闭</summary><ServiceReadback data={r.service_readiness.services}/></details>
   <p><b>运动固件、7301 内部判定、平衡就绪：仍未知。起身与模式切换继续锁定。</b></p>
  </details>
  <details className="motion-review-details" open>
   <summary>服务协议版本与固件边界 · 7301 复查</summary>
   <p>{r.protocol_review.summary}</p>
   <small>历史只读采样：{new Date(r.protocol_review.checked_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间；可在查询区重新读取。</small>
   <div className="table-scroll"><table><thead><tr><th>服务</th><th>API 协议版本</th><th>运动固件</th></tr></thead><tbody>{r.protocol_review.services.map(s=><tr key={s.service}><td>{s.service}</td><td>{s.api_version}</td><td>未知 · 不由协议推导</td></tr>)}</tbody></table></div>
   <p>{r.protocol_review.responder_evidence}</p>
   <ul>{r.protocol_review.limits.map(s=><li key={s}>{s}</li>)}</ul>
   <p><b>官方仓库中的兼容性线索 · 社区报告，非本机结论</b></p>
   <ul>{r.protocol_review.compatibility_notes.map(s=><li key={s}>{s}</li>)}</ul>
  </details>
  <details className="motion-review-details" open><summary>3104 / 7301 诊断 · 只读实机对照</summary><p>{r.rpc_diagnosis.summary}</p><small>采样时间：{new Date(r.rpc_diagnosis.checked_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间 · 历史证据</small><div className="table-scroll"><table><thead><tr><th>接收路径</th><th>查询数</th><th>报告 3104</th><th>报告 7301</th></tr></thead><tbody>{r.rpc_diagnosis.trials.map(t=><tr key={t.path}><td>{t.path}</td><td>{t.attempts}</td><td>{t.timeouts}</td><td>{t.code_7301}</td></tr>)}</tbody></table></div><p>{r.rpc_diagnosis.proof}</p><ul>{r.rpc_diagnosis.code_meanings.map(c=><li key={c.code}><b>{c.code}：</b>{c.meaning}</li>)}</ul><ul>{r.rpc_diagnosis.limits.map(s=><li key={s}>{s}</li>)}</ul></details>
  <div className="motion-stop-grid">{r.stop_semantics.map(s=><article key={s.name}><h3>{s.name}</h3><p>{s.mechanism}</p><p>{s.scope}</p><small>{s.verified}</small></article>)}</div>
  <details className="motion-review-details"><summary>App 截图证据 · {r.app_observation.device.model} · 软件 {r.app_observation.device.software_version}</summary>
   <p>{r.app_observation.authority}</p>
   <div className="table-scroll"><table><thead><tr><th>截图字段</th><th>读到的值</th><th>适用范围</th></tr></thead><tbody>
    <tr><td>型号 / 硬件</td><td>{r.app_observation.device.model} / {r.app_observation.device.hardware_version}</td><td>用户提供的设备页；23 自由度配置保持原有证据</td></tr>
    <tr><td>软件版本</td><td>{r.app_observation.device.software_version}</td><td>{r.app_observation.device.version_meaning}</td></tr>
    <tr><td>运动控制版本</td><td>未知 · 截图未单列</td><td>起身手册分支仍未匹配</td></tr>
    <tr><td>{r.app_observation.motor_snapshot.selected_metric}</td><td>可见通道 {r.app_observation.motor_snapshot.visible_value}</td><td>{r.app_observation.motor_snapshot.meaning}</td></tr>
   </tbody></table></div>
   <p><b>IMU 历史读数 · 拍摄时间未知</b></p>
   <div className="table-scroll"><table><thead><tr><th>部位</th><th>温度</th><th>roll / pitch</th></tr></thead><tbody>{r.app_observation.imu_snapshot.map(i=><tr key={i.label}><td>{i.label}</td><td>{i.temperature_c}°C</td><td>{i.roll_deg.toFixed(2)}° / {i.pitch_deg.toFixed(2)}°</td></tr>)}</tbody></table></div>
   <p>{r.app_observation.imu_meaning}</p>
   <p>截图的“{r.app_observation.device.update_label}”：{r.app_observation.device.update_meaning}</p>
   <ul>{r.app_observation.menu_evidence.map(m=><li key={m.group}><b>{m.group}：</b>{m.entries.join('、')}。{m.meaning}</li>)}</ul>
   <small>实机标定、足底/柜体载荷与独立站稳没有新增有效证据；动作切换继续锁定。</small>
  </details>
  <details className="motion-review-details"><summary>本机版本证据与下一步 · 104 原因已查明</summary><p>{r.firmware.summary}</p><p><b>查询修正：</b>{r.firmware.diagnosis.summary}</p><small>{r.firmware.diagnosis.proof}</small><div className="table-scroll"><table><thead><tr><th>版本层级</th><th>已取得内容</th><th>含义</th></tr></thead><tbody>{r.firmware.layers.map(l=><tr key={l.name}><td>{l.name}</td><td>{l.value??'未知'}</td><td>{l.meaning}</td></tr>)}</tbody></table></div><p>{r.firmware.next_step}</p></details>
  <details className="motion-review-details"><summary>23 自由度官方固件分支 · 本机尚未匹配</summary><p>官方分支的起身入口不同。先取得运动固件版本，再核定 SDK 映射；当前姿态的适用性仍未知。靠柜坐地仅为历史人工报告，不作为当前姿态证据。</p><div className="table-scroll"><table><thead><tr><th>手册版本条件</th><th>规定躺卧起点的路线</th><th>本机匹配</th></tr></thead><tbody>{r.firmware.manual_branches.map(b=><tr key={b.id}><td><a href={b.source_url} target="_blank" rel="noreferrer">{b.label} ↗</a></td><td>{b.lying_path}</td><td>未知 · 未核定 SDK 映射</td></tr>)}</tbody></table></div><ul>{r.firmware.manual_conditions.map(c=><li key={c}>{c}</li>)}</ul></details>
  <details className="motion-review-details"><summary>{r.modes.length} 项官方入口 · 进入条件、回读和未核实项</summary><div className="table-scroll"><table><thead><tr><th>功能 / SDK 入口</th><th>进入条件</th><th>反馈 / 当前缺口</th></tr></thead><tbody>{r.modes.map(m=><tr key={m.id}><td><b>{m.name}</b><br/>{m.documented_entry}<br/><span className="condition-badge unknown">本机适用性未知 · 锁定</span></td><td>{m.entry_conditions}</td><td>{m.feedback}<br/><b>待核实：</b>{m.remaining_unknown}</td></tr>)}</tbody></table></div></details>
  <details className="motion-review-details"><summary>如何继续形成可执行方案</summary><ol>{r.readiness.map(s=><li key={s}>{s}</li>)}</ol><ul>{r.controller_notes.map(s=><li key={s}>{s}</li>)}</ul></details>
  <div className="motion-review-sources">{r.documents.map(d=><a key={d.url} href={d.url} target="_blank" rel="noreferrer">{d.title} ↗</a>)}</div>
 </section>;
}
