import {useState} from 'react';
import type {ConfigurationAnalysis,Knowledge} from './types';

const time=(v:string)=>new Date(v).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false});
function ConfigurationResult({analysis:a}:{analysis:ConfigurationAnalysis}){
 return <details className="knowledge-fields" open><summary>两处非明文配置 · 1 / 2 已解析 · 新增电机实参 0 项</summary>
  <p className="panel-note">分析时间：{time(a.analyzed_at_utc)} 北京时间（UTC+8）。离线检查本机文件和对应程序，未启动厂商程序；这是历史证据。</p>
  <div className="table-scroll"><table><thead><tr><th>文件</th><th>已确认格式和内容</th><th>电机配置 / 标定</th></tr></thead><tbody>
   <tr><td>MegaFlow config.bin<br/>14640 B</td><td>{a.megaflow.format}<br/>{a.megaflow.content_classification}</td><td>完整 JSON 结构未见逐电机配置字段；未取得增益、限值或零位</td></tr>
   <tr><td>master_service.json<br/>468 B</td><td>{a.master.format}</td><td>内部内容未知，不能判断是否包含电机参数</td></tr>
  </tbody></table></div>
  <p className="panel-note">MegaFlow 清单：{Object.entries(a.megaflow.category_counts).map(([k,v])=>`${k} ${v}`).join(' · ')}。{a.megaflow.authority}。凭据与原始明文未进入基础模型。</p>
  <div className="table-scroll"><table><thead><tr><th>配置声明的插件</th><th>声明版本</th><th>核定状态</th></tr></thead><tbody>{a.megaflow.declared_plugins.map(p=><tr key={p.filename}><td>{p.name}<br/><small>{p.filename}</small></td><td>{p.version}</td><td>安装 / 加载 / 当前可用待核实；未授权执行</td></tr>)}</tbody></table></div>
  <details><summary>格式依据、组件版本与文件指纹</summary>
   <p className="panel-note">MegaFlow 本机加载函数 {a.megaflow.loader} @ {a.megaflow.loader_address}：{a.megaflow.call_chain.join(' → ')}。完整 JSON 为 {a.megaflow.plaintext_bytes} B；只保留指纹和允许公开的清单。</p>
   <p className="panel-note">FMX：{a.master.conclusion}。待核定：{a.master.unconfirmed.join('；')}。</p>
   {a.megaflow.declared_binaries.map(b=><p className="panel-note" key={b.name}>{b.name} · {b.version}</p>)}
   {a.files.map(f=><p className="panel-note" key={f.name}>{f.name} · {f.bytes} B<br/><small>SHA256 {f.sha256}</small></p>)}
   <ul>{a.limitations.map(l=><li key={l}>{l}</li>)}</ul><small>脱敏证据 SHA256 {a.source.sha256}</small>
  </details>
 </details>;
}
export function ServiceAudit({audit}:{audit:Knowledge['service_audit']}){
 const review=audit.fmx_loader_review,large=audit.large_library_review,packages=audit.package_cache_review;
 return <details className="knowledge-fields" open><summary>Orin 官方服务核查 · 本机启动与日志</summary>
  <details className="knowledge-fields" open><summary>安装包与 OTA 缓存 · 新增电机实参 0 项</summary><p className="panel-note">{time(packages.checked_at_utc)} 北京时间（UTC+8），历史核查。{packages.conclusion}。</p><p className="panel-note">有界扫描 {packages.files_scanned} 文件、{packages.binaries_scanned} 个 ELF。当前 master_service 与 FMX 配置读取前后指纹一致。</p><div className="table-scroll"><table><thead><tr><th>缓存包</th><th>格式 / 核定边界</th></tr></thead><tbody>{packages.packages.map(p=><tr key={p.path}><td>{p.path.split('/').pop()}<br/>{(p.bytes/1048576).toFixed(2)} MiB</td><td>{p.format}<br/>{p.version_meaning}</td></tr>)}</tbody></table></div><details><summary>范围、指纹与限制</summary><ul>{packages.roots.map(r=><li key={r}>{r}</li>)}</ul>{packages.packages.map(p=><p className="panel-note" key={p.path}>{p.path}<br/><small>SHA256 {p.sha256}</small></p>)}<ul>{packages.limitations.map(l=><li key={l}>{l}</li>)}</ul><p className="panel-note">需要：{packages.required_material}。</p><small>证据 SHA256 {packages.source.sha256}</small></details></details>
  <details className="knowledge-fields" open><summary>大库补扫 · 2 / 2 已完成 · 新增电机实参 0 项</summary>
   <p className="panel-note">{time(large.checked_at_utc)} 北京时间（UTC+8），历史证据。{large.conclusion}。</p>
   <div className="table-scroll"><table><thead><tr><th>本机文件 / 全符号行</th><th>核查结果</th></tr></thead><tbody>{large.files.map(f=><tr key={f.path}><td>{f.path.split('/').pop()}<br/>{(f.bytes/1048576).toFixed(2)} MiB · {f.symbols.line_count.toLocaleString('zh-CN')} 行</td><td>Mixer / FMX 标记未命中<br/>{f.classification}</td></tr>)}</tbody></table></div>
   <p className="panel-note">两份文件读取前后指纹一致。已知 master_service 的 Mixer::Gen / LoadFile 符号可见，但目标仍在磁盘无内容区域。未取得电机增益、限值或零位，253 个实际配置字段继续留空。</p>
   <details><summary>大库指纹、关联依据与限制</summary><p className="panel-note">{large.scope}。下方旧核查中的“两份大库未扫描”是此前快照，本次已补扫。</p>
    {large.files.map(f=><p className="panel-note" key={f.path}>{f.path} · {f.bytes} B<br/><small>SHA256 {f.sha256}</small><br/>{f.kind==='archive'?`Blowfish 符号行 ${f.symbols.generic_algorithm_line_counts.Blowfish}，仅为通用算法线索`:`动态符号 ${f.dynamic_symbols?.line_count.toLocaleString('zh-CN')} 行；rs2_ 相关行不等于电机配置`}</p>)}
    <p className="panel-note">master_service 的 DT_NEEDED 未列出这两个库；命名 CryptoPP / RealSense 符号分别为 {large.master_linkage.named_cryptopp_symbols} / {large.master_linkage.named_realsense_symbols}。这不能排除静态裁剪或运行时加载。</p>
    <ul>{large.limitations.map(l=><li key={l}>{l}</li>)}</ul>{large.source_references.map(r=><p className="panel-note" key={r.url}><a href={r.url} target="_blank" rel="noreferrer">上游资料</a>：{r.role}</p>)}<p className="panel-note">需要：{large.required_material}。</p><small>证据 SHA256 {large.source.sha256}</small>
   </details>
  </details>
  <details className="knowledge-fields" open><summary>Mixer / Gen 扩展核查 · 实机参数仍未知</summary>
   <p className="panel-note">{time(review.checked_at_utc)} 北京时间（UTC+8），历史证据。{review.conclusion}。</p>
   <p className="panel-note">新增范围：{review.local_search.unique_elfs} 个唯一 ELF、{review.local_search.static_archives} 个静态库，另有 {review.local_search.other_ar_containers} 个 Debian 包。{review.local_search.scope}。</p>
   <div className="table-scroll"><table><thead><tr><th>本机模块</th><th>清单版本 / 提交</th></tr></thead><tbody>{review.module_identity.modules.map(m=><tr key={m.name}><td>{m.name}</td><td>{m.declared_version}<br/><small>{m.declared_commit}</small></td></tr>)}</tbody></table></div>
   <p className="panel-note">{review.module_identity.authority}。文件大小匹配，清单未提供内容哈希；未取得对应源码和可复现构建。</p>
   <p className="panel-note">缓存补丁 {review.cached_packages[0].cached_path_version} 与已安装清单 {review.cached_packages[0].installed_manifest_version} 不同。两个 UTPK 包均未解开；OTA 解包入口同样指向磁盘无内容区域，不能据此认定 FMX 算法。</p>
   <details><summary>官方代码检索、限制与所需材料</summary>{review.official_code_search.queries.map(q=><p className="panel-note" key={q.query}>{q.query}：{q.total_count} 个命中{q.query.endsWith('UTPK')?'（STM32 HAL 文件，无 G1 格式依据）':''}</p>)}<ul>{review.limitations.map(l=><li key={l}>{l}</li>)}</ul><p className="panel-note">需要：{review.required_material}。</p><small>证据 SHA256 {review.source.sha256}</small></details>
  </details>
  <ConfigurationResult analysis={audit.configuration_analysis}/>
  <details className="knowledge-fields" open><summary>FMX 后续核查 · 载荷仍未核定 · 新增电机实参 0 项</summary>
   <p className="panel-note">{time(audit.fmx_analysis.checked_at_utc)} 北京时间（UTC+8），历史证据。{audit.fmx_analysis.conclusion}。</p>
   <p className="panel-note">文件 {audit.fmx_analysis.structure.file_bytes} B，偏移 8 的数值 {audit.fmx_analysis.structure.offset8_u32_le}。{audit.fmx_analysis.structure.conclusion}。</p>
   <p className="panel-note">检查 {audit.fmx_analysis.local_loader_search.elf_count} 个官方 ELF。{audit.fmx_analysis.local_loader_search.scope}。</p>
   <details><summary>公开研究假设与核定条件</summary><p className="panel-note"><a href={audit.fmx_analysis.research_hypotheses.source_url} target="_blank" rel="noreferrer">原作者研究</a>：{audit.fmx_analysis.research_hypotheses.authority}。有界测试 {audit.fmx_analysis.research_hypotheses.hypotheses_tested} 个组合，完整 JSON {audit.fmx_analysis.research_hypotheses.complete_json_count} 份。{audit.fmx_analysis.research_hypotheses.limitation}。</p><p className="panel-note">需要：{audit.fmx_analysis.required_material}。</p><small>证据 SHA256 {audit.fmx_analysis.source.sha256}</small></details>
  </details>
  <p className="panel-note">Mac 接收：{time(audit.receipt.received_at_utc)} 北京时间（UTC+8）。{audit.processes.length} 个官方进程 · {audit.files.length} 个配置记录 · {audit.inventory.length} 个文件条目。历史快照，非实时。</p>
  <p className="panel-note">{audit.conclusion}。新增核定实机配置：{audit.motor_configuration_values_obtained} 项。未执行动作、修改配置或启动官方脚本。</p>
  <div className="table-scroll"><table><thead><tr><th>服务</th><th>本机证据</th><th>参数核定边界</th></tr></thead><tbody>{audit.findings.map(f=><tr key={f.item}><td>{f.item}</td><td>{f.result}</td><td>{f.detail}</td></tr>)}</tbody></table></div>
  <details><summary>实际进程参数与 systemd 启动定义</summary><p className="panel-note">root 进程的 cwd / exe / 日志句柄不可读；未提权。MainPID=0 的生成单元不能据此判断服务未运行。以下内容只展示。</p>{audit.processes.map(p=><div key={p.pid}><h4>PID {p.pid} · PPID {p.ppid}</h4><pre className="official-source">{p.argv.join(' ')}</pre></div>)}{audit.units.map(u=><pre className="official-source" key={u.unit}>{u.fields}</pre>)}</details>
  <details><summary>首次检查的配置与日志 · SHA256 可追溯</summary><p className="panel-note">{audit.receipt.clock_authority}。首次检查的非明文文件只记录大小和指纹，后续格式分析见上方。历史日志中的配置加载不证明该配置属于电机。</p>{audit.files.map(f=><details key={f.path}><summary>{f.path} · {f.bytes} B · {f.format==='opaque'?'首次检查为非明文':'文本'}</summary><small>SHA256 {f.sha256??'超出读取上限，未核定'}</small>{f.text&&<pre className="official-source">{f.text}</pre>}</details>)}{audit.logs.map((l,i)=><details key={i}><summary>{l.path??l.unit} · {l.matching_lines??l.matches.length} 条命中{l.search_bytes?'（有界全文）':''}</summary><pre className="official-source">{l.matches.join('\n')||'本次有界读取无匹配；不证明不存在'}</pre></details>)}<ul>{audit.limitations.map(s=><li key={s}>{s}</li>)}</ul><small>首次审计源 SHA256 {audit.source.sha256}</small></details>
 </details>;
}

export default function OfficialKnowledge({knowledge}:{knowledge:Knowledge['official_capabilities']}){
 const [search,setSearch]=useState(''),[language,setLanguage]=useState('全部');
 const entries=knowledge.methods.filter(m=>(language==='全部'||m.language===language)&&`${m.group} ${m.service} ${m.name} ${m.parameters.map(p=>p.name).join(' ')}`.toLowerCase().includes(search.toLowerCase()));
 const reads=knowledge.historical_reads;
 return <section className="surface official-knowledge"><div className="panel-title"><h2>官方能力参数档案</h2><span className="tag">{knowledge.methods.length} 个方法 / 重载 · 只读资料</span></div>
  <p className="panel-note">{knowledge.scope}。{knowledge.authority}</p>
  <details open><summary>本机能力参数回读 · 历史查询</summary><p className="panel-note">{time(reads.completed_at_utc)} 北京时间（UTC+8）。{reads.rpc_calls} 次固定 getter；未申请控制权、未执行动作。7301 = 官方 LocoState 不可用；失败项没有有效值，不能推定为零或电机故障。</p><div className="table-scroll"><table><thead><tr><th>官方读取接口</th><th>返回码</th><th>实际返回</th></tr></thead><tbody>{reads.readings.map(r=><tr key={`${r.service}.${r.api_id}`}><td>{r.service}.{r.method}<small>API {r.api_id}</small></td><td>{r.code??'未收到'}</td><td>{r.code!==0?'未取得有效值':r.method==='GetActionList'?<details><summary>固件原始能力目录</summary><pre className="official-source">{JSON.stringify(r.payload,null,2)}</pre></details>:<code>{JSON.stringify(r.payload)}</code>}</td></tr>)}</tbody></table></div>{reads.unqueried.map(u=><p className="panel-note" key={u.method}>{u.method}：{u.reason}</p>)}</details>
  <details className="knowledge-fields"><summary>SDK 版本差异与项目参数限制</summary><ul>{knowledge.differences.map(s=><li key={s}>{s}</li>)}</ul><p className="panel-note">以下为控制台输入约束，不能用作实机能力上限：</p><pre className="official-source">{JSON.stringify(knowledge.project_policy,null,2)}</pre></details>
  <div className="official-filter"><input aria-label="搜索能力参数" placeholder="搜索方法、服务或参数名…" value={search} onChange={e=>setSearch(e.target.value)}/><select aria-label="SDK 语言" value={language} onChange={e=>setLanguage(e.target.value)}>{['全部','Python','C++'].map(l=><option key={l}>{l}</option>)}</select><span className="minor">显示 {entries.length} 项</span></div>
  <div className="official-methods">{entries.map(m=><details key={m.id}><summary><b>{m.name}</b><span>{m.language} · {m.group} · {m.kind==='read'?'读取定义':m.kind==='client_local'?'客户端本地状态':'改变设备状态的接口定义'}</span></summary><p className="panel-note">{m.service} · API v{m.api_version} · ID {m.api_ids.join(' / ')||'无 RPC'}。{m.support}。此处没有执行入口。</p>
   {m.parameters.length?<div className="table-scroll"><table><thead><tr><th>参数</th><th>类型 / 方向</th><th>SDK 默认值</th><th>单位 / 实机范围</th></tr></thead><tbody>{m.parameters.map(p=><tr key={p.name}><td><code>{p.name}</code></td><td>{p.type} · {p.direction==='out'?'输出':'输入'}</td><td>{p.has_default?p.default:'无默认值'}</td><td>{p.unit??'未核定'} / {p.hardware_range?JSON.stringify(p.hardware_range):'未核定'}</td></tr>)}</tbody></table></div>:<p className="panel-note">无调用参数；固定别名参数见下方。</p>}
   {m.wire!==null&&<pre className="official-source">{JSON.stringify(m.wire,null,2)}</pre>}<ul>{m.notes.map(n=><li key={n}>{n}</li>)}</ul><details><summary>固定版本官方实现与来源</summary><pre className="official-source">{m.source_excerpt}</pre><a href={`${knowledge.sources[m.source].url}#L${m.line}`} target="_blank" rel="noreferrer">官方源码 · {knowledge.sources[m.source].commit.slice(0,12)}</a><small>SHA256 {knowledge.sources[m.source].sha256}</small></details>
  </details>)}</div>{entries.length===0&&<p className="panel-note">没有匹配的参数定义。</p>}
  <details className="knowledge-fields"><summary>仅有常量、无公开封装的接口 · {knowledge.unwrapped_constants.length} 项</summary><p className="panel-note">仅记录定义，不猜测调用载荷或补造读取方法。</p>{knowledge.unwrapped_constants.map((c,i)=><p className="panel-note" key={i}>{c.language} / {c.group} · {c.name} = {c.api_id}</p>)}</details>
 </section>;
}
