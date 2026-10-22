import { useState } from 'react';
import { EyeIcon, PauseIcon, PlayIcon, ShieldCheckIcon } from '@phosphor-icons/react';
import { api } from './api';
import type { Finding, Snapshot } from './types';

const STATUS:Record<string,string>={paused:'已暂停',waiting_worker:'等待 Mac 观察进程',waiting_input:'等待新鲜画面与身体反馈',analyzing:'Codex 正在理解',observing:'自主观察中',error:'本轮分析失败'};
const ERROR:Record<string,string>={CODEX_FAILED:'本轮分析未完成，将自动重试；持续失败时请检查 Mac 登录和网络',CODEX_RATE_LIMITED:'Codex 服务限流，已延长重试间隔至 60–300 秒；上一条结果保留原采样时间',CODEX_TIMEOUT:'本轮分析超时，将自动重试',INVALID_REPORT:'本轮输出未通过结构校验',INPUT_EXPIRED:'输入已失效，等待重新采样'};
const EVIDENCE={image:'画面',telemetry:'身体反馈',both:'画面＋身体反馈'};
const CONFIDENCE={high:'较高置信',medium:'中等置信',low:'低置信'};
function Findings({items}:{items:Finding[]}){return <ul className="insight-findings">{items.map((f,i)=><li key={i}><p>{f.statement}</p><small>{EVIDENCE[f.evidence]} · {CONFIDENCE[f.confidence]}</small></li>)}</ul>;}

export default function ObservationView({state,transport,clock}:{state:Snapshot;transport:boolean;clock:number}){
 const [busy,setBusy]=useState(false),[error,setError]=useState('');
 const insight=state.insight, result=insight?.report;
 const age=Math.round((insight?.report_age_s??0)+Math.max(0,clock/1000-state.at));
 const current=!!insight?.current&&transport&&age<insight.report_ttl_s;
 async function toggle(){setBusy(true);setError('');try{await api('/api/insight/control',{enabled:!insight.enabled});}catch(e){setError(e instanceof Error?e.message:'观察设置失败');}finally{setBusy(false);}}
 if(!insight)return null;
 return <section className="surface insight-panel" aria-label="Codex 自主观察">
  <div className="panel-title"><h2><EyeIcon size={20}/>Codex 自主观察</h2><span className="tag">建议模式 · 动作锁定</span></div>
  <div className="insight-controls"><span><i className={'dot '+(transport&&insight.worker_online&&insight.enabled?'live':'')}/>{transport?STATUS[insight.status]||insight.status:'控制台连接中断'}</span><button className="secondary" disabled={!transport||busy||!state.read_only||state.mode!=='robot'} onClick={()=>void toggle()}>{insight.enabled?<PauseIcon/>:<PlayIcon/>}{insight.enabled?'暂停观察':'开始观察'}</button></div>
  <p className="insight-runtime">Mac · {insight.model||'等待 Codex'} · 已完成 {insight.completed} 次<br/>实时预览持续刷新；语义分析逐次采样。图像与身体数据由 Mac 已登录的 Codex 服务分析。</p>
  {(error||insight.error)&&<p role="alert" className="insight-alert">{error||ERROR[insight.error!]||'观察异常，等待下一轮'}</p>}
  {result?<div className={'insight-result '+(!current?'historical':'')}>
   <div className="insight-evidence"><b>{current?'最近一次观察':'历史观察 · 当前不可作为依据'}</b><span>基于 {age} 秒前的采样 · 分析耗时 {(result.analysis_ms/1000).toFixed(1)} 秒</span><span>{new Date(result.evidence.captured_at*1000).toLocaleString('zh-CN',{hour12:false})} · 相机 #{result.evidence.camera.sequence} · 身体 tick {result.evidence.tick}</span><span>{result.evidence.joints} 个关节 · 采样时身体 {result.evidence.telemetry_age_ms} ms / 相机 {result.evidence.camera.age_ms} ms</span></div>
   <p className="insight-summary">{result.report.summary}</p>
   <div className="insight-columns"><section><h3>环境事实</h3><Findings items={result.report.scene_facts}/></section><section><h3>身体解读</h3><Findings items={result.report.body_findings}/></section></div>
   <div className="insight-changes"><h3>采样间变化</h3><p>{result.report.change_summary}</p></div>
   <div className="insight-unknown"><h3>仍不确定</h3><ul>{result.report.uncertainties.map((s,i)=><li key={i}>{s}</li>)}</ul></div>
   <div className="insight-suggestions"><h3>任务建议 <small>仅供审阅，不会执行</small></h3>{result.report.suggestions.map((s,i)=><article key={i}><b>{String(i+1).padStart(2,'0')} · {s.title}</b><p>{s.rationale}</p>{s.requirements.length>0&&<ul>{s.requirements.map((r,j)=><li key={j}>{r}</li>)}</ul>}{s.capability_ids.length>0&&<small>相关官方接口：{s.capability_ids.map(id=>state.capabilities.find(c=>c.id===id)?.label||id).join('、')}</small>}</article>)}</div>
  </div>:<div className="insight-empty"><EyeIcon size={30}/><h3>{insight.enabled?'正在等待首份观察':'让 Codex 理解此刻的环境'}</h3><p>结合一帧官方相机画面、23 个关节和 IMU 反馈，给出有依据的环境判断与下一步建议。</p></div>}
  <p className="panel-note"><ShieldCheckIcon size={14}/> 观察结果不证明通路可行或机器人不会摔倒。深度、定位、接触和支撑条件未验证前，保持只读。</p>
 </section>;
}
