import { useEffect, useState } from 'react';
import { CameraIcon, DownloadSimpleIcon } from '@phosphor-icons/react';
import type { Snapshot } from './types';

export default function CameraView({state, transport, clock}:{state:Snapshot;transport:boolean;clock:number}) {
 const [frame,setFrame]=useState<{url:string;received:number;age:number;sequence:string}|null>(null);
 const [failed,setFailed]=useState(false);
 const live=transport&&state.world.camera_age_ms!==null&&state.world.camera_age_ms<1000;
 useEffect(()=>{
  if(!live)return;
  let disposed=false, url:string|undefined, timer:ReturnType<typeof setTimeout>;
  const controller=new AbortController();
  const load=async()=>{
   const start=performance.now();
   try {
    if(!document.hidden){
     const response=await fetch('/api/camera.jpg',{credentials:'same-origin',cache:'no-store',signal:controller.signal});
     if(!response.ok)throw new Error('相机帧不可用');
     const blob=await response.blob();
     const age=Number(response.headers.get('X-G1-Camera-Age-Ms'))+performance.now()-start;
     if(disposed)return;
     if(age>=1000)throw new Error('相机帧已过期');
     const next=URL.createObjectURL(blob),previous=url;url=next;
     setFrame({url:next,received:performance.now(),age,sequence:response.headers.get('X-G1-Camera-Sequence')||'—'});
     setFailed(false);
     if(previous)URL.revokeObjectURL(previous);
    }
   }catch{if(!disposed)setFailed(true);}
   if(!disposed)timer=setTimeout(load,Math.max(20,250-(performance.now()-start)));
  };
  void load();
  return()=>{disposed=true;controller.abort();clearTimeout(timer);if(url)URL.revokeObjectURL(url);};
 },[live]);
 // Parent clock also expires a decoded image when both HTTP and websocket stop.
 void clock;
 const visible=live&&!failed&&frame!==null&&performance.now()-frame.received+frame.age<1000;
 const camera=state.camera;
 return <section className="surface camera-panel live-camera">
  <div className="panel-title"><h2><CameraIcon size={20}/>机器人视野</h2><span className="minor"><i className={'dot '+(visible?'live':'')}/>{visible?'实时画面':'等待画面'}</span></div>
  <div className="camera-viewport">{visible?<img alt="G1 相机实时画面" src={frame.url} onError={()=>setFailed(true)}/>:<div className="camera-empty"><CameraIcon size={34}/><b>{state.mode==='demo'?'演示未生成相机画面':failed||camera?.status==='stale'?'画面已中断，正在重试':'等待官方相机服务'}</b><p>{state.mode==='disconnected'?'连接真实 G1 后读取视频':camera?.error?`官方取图暂不可用 · ${camera.error}`:'接收成功后自动显示，过期画面自动隐藏'}</p></div>}</div>
  <div className="camera-facts"><span>{camera?.width?`${camera.width} × ${camera.height}`:'RGB'}<small>{camera?.source_width?`原始 ${camera.source_width} × ${camera.source_height}`:'等待图像信息'}</small></span><span>帧 {visible?frame.sequence:'—'}<small>{visible?`服务端帧龄 ${camera?.age_ms??'—'} ms`:'尚无有效画面'}</small></span><a className={'secondary '+(!visible?'disabled':'')} href={visible?frame.url:undefined} download="g1-camera.jpg" aria-disabled={!visible}><DownloadSimpleIcon size={16}/>保存当前帧</a></div>
  {camera?.diagnostics&&<p className="panel-note">{transport?'近':'连接中断前'} {camera.diagnostics.window_s} 秒取图：成功 {camera.diagnostics.successes} / {camera.diagnostics.attempts} 次，超时 {camera.diagnostics.timeouts} 次，失败合计 {camera.diagnostics.errors} 次。{camera.diagnostics.attempts===0?'正在等待首次回读。':'此统计只表示取图链路，不证明机器人可安全行动。'}</p>}
  <p className="panel-note">只读预览，不持续录像。启用自主观察时，采样画面与身体状态会提交至已登录的 Codex 服务分析。帧龄按取图请求计算，官方未提供曝光时间；RGB 尚未接入定位与导航。</p>
  {camera?.error==='SDK_3102'&&<p className="panel-note">SDK 请求发送失败。请核对相机 Type-C 连接及 Orin 的相机枚举；也可能是视频服务 DDS 接收端暂不可达。仅凭此码不能确定硬件故障，连接恢复后会自动重试。</p>}
  {camera?.error==='SDK_3104'&&<p className="panel-note">700 ms 内未收到与本次请求匹配的官方图像响应。系统继续只读重试；此错误不能单独定位为相机或 USB 故障。</p>}
 </section>;
}
