import {serviceRows,serviceStatus} from './serviceState';

export default function ServiceReadback({data}:{data:unknown}){
 const rows=serviceRows(data);
 if(rows===null)return <p>没有有效服务清单；不能判断 ai_sport 开启状态或运动固件。</p>;
 return <div className="service-readback">
  <p><b>服务开启状态 · {rows.length} 项</b></p>
  <p>开启只说明服务状态，不证明平衡、独立承重或动作适用。此接口未提供运动固件版本；protect 仅显示原值，含义未核定。</p>
  <div className="table-scroll"><table><thead><tr><th>官方服务</th><th>状态 / 原值</th><th>protect 原值</th></tr></thead><tbody>{rows.map((r,i)=><tr key={`${r.name}:${i}`}><td><b>{r.name}</b>{r.name==='ai_sport'&&<small>主运动控制服务</small>}</td><td>{serviceStatus(r.status)} · {r.status}</td><td>{r.protect}</td></tr>)}</tbody></table></div>
  <small><a href="https://support.unitree.com/home/en/G1_developer/robot_state_client_interface" target="_blank" rel="noreferrer">官方 G1 状态定义 ↗</a> · status 0 开启 / 1 关闭；不提供服务开关操作。</small>
 </div>;
}
