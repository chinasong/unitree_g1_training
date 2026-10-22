import type { Body, Compatibility, Condition, ConditionStatus, Joint, Snapshot } from './types';
import {ServiceAudit} from './OfficialKnowledge';
import StateLearningView from './StateLearningView';
import Disclosure from './Disclosure';

const label:Record<ConditionStatus,string> = {matched:'条件符合',violated:'条件不符',unknown:'待核实'};
const n = (v:number|null|undefined,d=3) => v==null?'未知':v.toFixed(d);
export function CompatibilityBadge({value,fresh}:{value:Compatibility|undefined;fresh:boolean}){
 const status = fresh&&value?value.status:'unknown';
 return <span className={`condition-badge ${status}`} title={fresh&&value?value.conditions.map(c=>`${c.name}：${c.detail}`).join('\n'):'反馈缺失或过期'}>{label[status]}</span>;
}
export function BodyAssessment({state,fresh,policy}:{state:Snapshot;fresh:boolean;policy:Body['motor_knowledge']['state_safety']}){
 const a=state.assessment;
 const mismatch=fresh?a.joint_limits.filter(j=>j.status==='violated'):[];
 const matched=fresh?a.joint_limits.filter(j=>j.status==='matched').length:0;
 return <section className="surface body-assessment"><div className="panel-title"><h2>当前身体状态</h2><span className="tag">基础档案 + 实时反馈</span></div>
  <p className="state-summary">{fresh?a.summary:'身体反馈已过期；当前状态待核实'}</p>
  <Disclosure title="只读补证、站稳方案与起身理解" summary="只读反馈、分阶段站稳方案、防摔边界及官方能力适用性证据">
   <StateLearningView value={a.state_learning} policy={policy} fresh={fresh}/>
  </Disclosure>
  <Disclosure title="行走状态条件与关节范围证据" summary={`${matched} 项符合 · ${mismatch.length} 项不符 · ${23-matched-mismatch.length} 项待核实`}>
   <div className="condition-grid">{a.checks.map(c=><div key={c.name}><span className={`condition-badge ${fresh?c.status:'unknown'}`}>{fresh?label[c.status]:label.unknown}</span><h3>{c.name}</h3><p>{fresh?c.detail:'等待新鲜反馈'}</p><small>{c.source}</small></div>)}</div>
   <div className="assessment-meta"><span>坐标：{fresh?a.coordinate_mode:'未知'}</span><span>机型代码：{fresh?state.telemetry.mode_machine??'未知':'未知'}</span><span>角度范围：{matched} 符合 / {mismatch.length} 不符 / {23-matched-mismatch.length} 待核实</span><span>有效窗口 {fresh?a.movement.window_s:0}s · {fresh?a.movement.window_samples:0} 样本 · 最大速度 {fresh?n(a.movement.window_max_abs_dq_rad_s):'未知'} rad/s</span></div>
   <p className="panel-note">{fresh?a.mode_machine_note:'机型文档适用性待核实'}</p>
   {mismatch.map(j=><p className="model-deviation" key={j.index}>编号 {j.index}：{j.detail}；偏差 {n(j.deviation_rad,5)} rad（{n(j.deviation_rad==null?null:j.deviation_rad*180/Math.PI,3)}°）。</p>)}
   <p className="panel-note">只比较已取得的条件，符合不表示可执行。模型范围、低速及小倾角均不能证明实机安全或稳定站立。</p>
  </Disclosure>
 </section>;
}
export default function BodyKnowledge({body,selected,live,fresh,applicability}:{body:Body;selected:number;live:Joint|undefined;fresh:boolean;applicability:Condition}){
 const k=body.motor_knowledge, m=k.motors.find(j=>j.index===selected);
 if(!m)return null;
 const feedback:Record<string,unknown>={mode:live?.mode,q:live?.q,dq:live?.dq,ddq:live?.ddq,tau_est:live?.tau,temperature:live?.temperature,vol:live?.voltage,sensor:live?.sensor,motorstate:live?.status,reserve:live?.reserve};
 const raw=(key:string)=>!fresh||feedback[key]==null?'未知':JSON.stringify(feedback[key]);
 const v=m.documented_variant;
 const variantStatus=fresh?applicability.status:'unknown';
 const changed=v.model.effort_nm!==m.model.effort_nm||v.model.velocity_rad_s!==m.model.velocity_rad_s;
 const audit=k.board_audit, installed=audit.installed_model.joints.find(j=>j.index===selected);
 const interfaces=k.interface_audit, rpc=interfaces.rpc_metadata;
 const github=k.github_review;
 const staticRows = [
  ['模型角度范围',`${m.model.position_rad.join(' ~ ')} rad`],['模型速度限值',`${m.model.velocity_rad_s} rad/s`],
  ['URDF effort',`${m.model.effort_nm} N·m`],['MuJoCo 力矩范围',`${m.model.actuator_force_nm.join(' ~ ')} N·m`],
  ['关节轴',m.model.axis.join(', ')],['父 / 子连杆',`${m.model.parent} / ${m.model.child}`],
  ['原点 xyz (m)',m.model.origin.xyz],['原点 rpy (rad)',m.model.origin.rpy],['连杆质量',`${m.model.link_mass_kg} kg`],
  ['惯性张量 (kg·m²)',Object.entries(m.model.inertia_kg_m2).map(([key,v])=>`${key}=${v}`).join(' · ')],
  ['URDF dynamics',m.model.dynamics?JSON.stringify(m.model.dynamics):'未提供'],
 ];
 return <section className="surface motor-knowledge"><div className="panel-title"><h2>机器人基础档案 <span className="tag">持久保存 v{k.schema_version}</span></h2><span className="minor">{m.label} · MOTOR {selected}</span></div>
  <p className="panel-note">{k.persistence}。档案 {k.foundation_id.slice(0,12)}。官方机型文档与实机标定分开保存；实机增益、标定和完整热限制尚未取得。</p>
  <ServiceAudit audit={k.service_audit}/>
  <details className="knowledge-fields"><summary>官方 GitHub issue / 讨论核查 · 历史资料</summary>
   <p className="panel-note">{new Date(github.checked_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间（UTC+8）。{github.search_query_count} 组公开查询取得 {github.search_result_count} 个去重命中，完整读取 {github.full_comment_thread_count} 个相关 issue 的 {github.comment_count} 条评论；命中数不代表全部已逐条审核。</p>
   <p className="panel-note">{github.conclusion}。新增核定实机配置：0 项。</p>
   <div className="table-scroll"><table><thead><tr><th>来源</th><th>核查结果</th><th>G1 适用范围</th></tr></thead><tbody>{github.findings.map(f=><tr key={f.url}><td><a href={f.url} target="_blank" rel="noreferrer">{f.repo} #{f.number}</a>{f.reply_url&&<small><a href={f.reply_url} target="_blank" rel="noreferrer">相关回复</a></small>}</td><td>{f.result}<small>{f.collaborator_reply?'含 GitHub COLLABORATOR 回复':'社区提问 / 回复'}</small></td><td>{f.detail}</td></tr>)}</tbody></table></div>
   <details><summary>来源身份、Discussions 与检索范围</summary><p className="panel-note">{github.authority}</p><p className="panel-note">本次核对的 {github.repositories.length} 个核心仓库中，{github.repositories.filter(r=>!r.has_discussions).length} 个未启用 Discussions；SDK2 / ROS2 也未启用 Issues。不能据此推定其他官方渠道没有资料。</p><ul className="source-list">{github.repositories.map(r=><li key={r.full_name}><a href={r.html_url} target="_blank" rel="noreferrer">{r.full_name}</a><small>Issues {r.has_issues?'开启':'关闭'} · Discussions {r.has_discussions?'开启':'关闭'}</small></li>)}</ul><ul>{github.limitations.map(s=><li key={s}>{s}</li>)}</ul></details>
  </details>
  <details className="knowledge-fields" open><summary>非 SSH 接口核查 · 历史查询，非实时</summary>
   <p className="panel-note">{new Date(rpc.completed_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间（UTC+8）完成。经 Orin 查询，未登录运动主机。{rpc.versions.filter(v=>v.code===0).length} 个服务返回接口版本，{rpc.robot_state.services.entries?.length??0} 条软件服务记录；新增核定的逐电机配置值：{interfaces.motor_configuration_values_obtained} 项。</p>
   <p className="panel-note">{interfaces.conclusion}。配置读取：待核实；标定读取：待核实。服务版本不作为电机固件，服务列表不证明物理板或灵巧手存在。</p>
   <div className="table-scroll"><table><thead><tr><th>接口</th><th>核查结果</th><th>含义与边界</th></tr></thead><tbody>{interfaces.findings.map(f=><tr key={f.interface}><td>{f.interface}</td><td>{f.result}</td><td>{f.detail}</td></tr>)}</tbody></table></div>
   <details><summary>实收版本与服务原码</summary><div className="table-scroll"><table><thead><tr><th>服务</th><th>返回码</th><th>API 版本</th></tr></thead><tbody>{rpc.versions.map(v=><tr key={v.service}><td><code>{v.service}</code></td><td>{v.code}</td><td>{v.api_version??'未知'}</td></tr>)}</tbody></table></div><p className="panel-note">GetPkgVersion 返回 {rpc.robot_state.packages.code}（本次 3203 为官方定义的接口未实现）；软件包与模块版本保持未知。未确认各服务实际运行在哪块物理主机。</p><div className="table-scroll"><table><thead><tr><th>软件服务名</th><th>status 原码</th><th>protect 原码</th></tr></thead><tbody>{rpc.robot_state.services.entries?.map(s=><tr key={s.name}><td><code>{s.name}</code></td><td>{s.status}</td><td>{s.protect}</td></tr>)}</tbody></table></div></details>
   <details><summary>继续核定所需的官方资料</summary><ul>{interfaces.required_official_materials.map(s=><li key={s}>{s}</li>)}</ul></details>
   <details><summary>本轮官方文档与源码 · 已固定提交及 SHA256</summary><ul className="source-list">{interfaces.documents.map(d=><li key={d.url}><a href={d.url} target="_blank" rel="noreferrer">{d.title}</a><small>文档标注更新：{d.updated} · {d.review}</small></li>)}{interfaces.source_files.map(s=><li key={s.url}><a href={s.url} target="_blank" rel="noreferrer">{s.repository_path}</a><small>commit {s.commit} · SHA256 {s.sha256}</small></li>)}</ul></details>
  </details>
  <details className="knowledge-fields"><summary>其他控制板核查 · 此前被动读取，非实时</summary>
   <p className="panel-note">检查完成：{new Date(audit.completed_at_utc).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})} 北京时间（UTC+8）。主板与 BMS 话题有实收数据；运动主机 {audit.pc1_ssh.address}:22 {audit.pc1_ssh.status==='refused'?'拒绝连接':'状态未确认'}。新核定的逐电机配置值：{audit.motor_configuration_values_obtained} 项。</p>
   <div className="table-scroll"><table><thead><tr><th>被动读取话题</th><th>实收样本</th><th>证据</th></tr></thead><tbody>{audit.topic_readings.map(r=><tr key={r.topic}><td><code>{r.topic}</code></td><td>{r.samples}</td><td>{r.raw_fields?<details><summary>历史原始字段</summary><pre style={{whiteSpace:'pre-wrap',overflowWrap:'anywhere'}}>{JSON.stringify(r.raw_fields,null,2)}</pre><small>{r.meaning}</small></details>:'本次窗口未收到，不推定不存在'}</td></tr>)}</tbody></table></div>
   <p className="panel-note">{audit.meaning}。DDS 端点是软件接口，不能据此数控制板、确认灵巧手或核定电机序号。LowCmd 仅被动订阅；读到的零增益不代表出厂或生效增益。主板/BMS 字段保留原码，未猜测单位、通道或状态位。</p>
   <details><summary>此前发现的配置 / 标定服务</summary><p className="panel-note">{audit.unqueried_services.join(' · ')}。{audit.service_limitation}。后续版本查询见上方接口核查；配置及标定业务方法仍未调用。</p></details>
   <details><summary>Orin 本机安装模型 · 与实机配置分开保存</summary><p className="panel-note">{audit.installed_model.name}。{audit.installed_model.meaning}。{audit.installed_model.differences.length} 个关节的模型力矩范围与对应官方 URDF 不同。</p>{installed&&<p className="panel-note">当前选中关节：模型角度 {installed.position_rad.join(' ~ ')} rad；模型力矩范围 {installed.actuator_force_range_nm.join(' ~ ')} N·m。未用于填写本机连续 / 峰值力矩或动作门槛。</p>}</details>
  </details>
  <div className="knowledge-columns"><div><h3>机型对应的官方文档 <span className={`condition-badge ${variantStatus}`}>{label[variantStatus]}</span></h3><p className="panel-note">{k.documented_hardware.name} · mode_machine={v.mode_machine}。{fresh?applicability.detail:'等待新鲜反馈确认适用性。'}</p>
   <dl className="knowledge-pairs"><div><dt>文档减速比</dt><dd>{v.gear_ratio??'本关节未核定'}{v.gear_ratio!=null?' : 1':''}</dd></div><div><dt>对应 URDF 角度</dt><dd>{v.model.position_rad.join(' ~ ')} rad</dd></div><div><dt>对应 URDF effort</dt><dd>{v.model.effort_nm} N·m</dd></div><div><dt>对应 URDF 速度</dt><dd>{v.model.velocity_rad_s} rad/s</dd></div><div><dt>对应 URDF 总质量</dt><dd>{n(k.documented_hardware.model_total_mass_kg)} kg</dd></div><div><dt>逐电机序号 / 标定核验</dt><dd>尚未取得</dd></div></dl>
   <p className="panel-note">以上是官方对应机型的文档值，effort 与速度仍是模型限值。{changed?'本关节与旧 RL Gym 参考值不同，已保留差异。':''}</p><details><summary>旧 RL Gym 模型与结构参考</summary><p className="panel-note">旧参考 URDF 总质量 {n(k.model_total_mass_kg)} kg。3D 结构沿用此参考，不能代表实机称量或完整硬件版本。</p><dl className="knowledge-pairs">{staticRows.map(([key,value])=><div key={key}><dt>{key}</dt><dd>{value}</dd></div>)}</dl></details></div>
   <div><h3>实机配置与温度定义</h3><dl className="knowledge-pairs"><div><dt>本机当前 Kp / Kd</dt><dd>未知 / 未知</dd></div><div><dt>本机连续 / 峰值力矩</dt><dd>未知 / 未知</dd></div><div><dt>电流限制 / 零位</dt><dd>未知</dd></div><div><dt>减速器实测 / 标定</dt><dd>未知</dd></div><div><dt>电机型号 / 序号 / 固件</dt><dd>未知</dd></div>{k.temperature_channels.map(c=><div key={c.index}><dt>temperature[{c.index}]</dt><dd>{c.label}温度 · 官方 G1 SDK 定义</dd></div>)}<div><dt>本机固件热保护阈值</dt><dd>未知</dd></div></dl>
   <details><summary>SDK 示例与辅助函数 · 不作为本机配置</summary><p className="panel-note">示例 Kp / Kd：{m.example_only.kp} / {m.example_only.kd}。{m.example_only.meaning}。</p><p className="panel-note">{k.temperature_channels.map(c=>`${c.label}辅助函数默认 ${c.sdk_helper_default_c} °C`).join('；')}。{k.thermal_authority}。</p></details><details><summary>待补齐的实机资料（{k.unknowns.length} 类）</summary><ul>{k.unknowns.map(u=><li key={u}>{u}</li>)}</ul></details></div></div>
  <details className="knowledge-fields"><summary>已核查并排除的参数混用</summary><ul>{k.rejected_inferences.map(r=><li key={r.source}>{r.reason}</li>)}</ul></details>
  <details className="knowledge-fields"><summary>全部电机反馈字段 · 10 项 · {fresh?'当前回读':'当前未知'}</summary><div className="table-scroll"><table><thead><tr><th>字段 / 类型</th><th>当前值</th><th>单位与定义</th></tr></thead><tbody>{k.feedback_fields.map(f=><tr key={f.name}><td><code>{f.name}</code><small>{f.type}</small></td><td className="raw-value">{raw(f.name)}</td><td>{f.unit}<small>{f.meaning}</small></td></tr>)}</tbody></table></div></details>
  <details className="knowledge-fields"><summary>命令协议字段 · 7 项 · 仅定义，不发布</summary><div className="table-scroll"><table><thead><tr><th>字段 / 类型</th><th>单位</th><th>含义</th></tr></thead><tbody>{k.command_fields.map(f=><tr key={f.name}><td><code>{f.name}</code><small>{f.type}</small></td><td>{f.unit}</td><td>{f.meaning}</td></tr>)}</tbody></table></div></details>
  <details className="knowledge-fields"><summary>官方来源与版本指纹 · {Object.keys(k.sources).length} 项</summary><ul className="source-list">{Object.entries(k.sources).map(([id,s])=><li key={id}><a href={s.url} target="_blank" rel="noreferrer">{id} · {s.path.split('/').at(-1)}</a><small>commit {s.commit} · SHA256 {s.sha256}</small></li>)}</ul></details>
 </section>;
}
