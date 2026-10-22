import { useMemo } from 'react';
import type { Snapshot } from './types';
export default function MapView({state,target,onTarget}:{state:Snapshot;target:number[]|null;onTarget:(p:number[])=>void}){
 const grid=state.world.grid;
 const walls=useMemo(()=>{if(!grid)return '';let d='';for(let y=0;y<grid.height;y++)for(let x=0;x<grid.width;x++)if(grid.cells[y*grid.width+x]>=50||grid.cells[y*grid.width+x]<0)d+=`M${x},${grid.height-y-1}h1v1h-1z`;return d;},[grid?.revision,grid?.frame,grid?.width,grid?.height]);
 if(!grid)return <div className="map-empty"><span className="crosshair"/><strong>等待环境地图</strong><p>接入同一坐标系的定位、占用栅格和障碍物扫描后，即可选择目标。</p></div>;
 const point=(p:number[])=>[(p[0]-grid.origin[0])/grid.resolution,grid.height-(p[1]-grid.origin[1])/grid.resolution];
 const pose=state.world.pose;const pos=pose?point([pose.x,pose.y]):null;
 const path=state.navigation.path.map(p=>point(p).join(',')).join(' ');
 return <div className="map-view"><svg viewBox={`-1 -1 ${grid.width+2} ${grid.height+2}`} role="img" aria-label="环境占用地图，点击选择导航目标" onClick={e=>{const r=e.currentTarget.getBoundingClientRect();const scale=Math.min(r.width/(grid.width+2),r.height/(grid.height+2));const ox=(r.width-(grid.width+2)*scale)/2,oy=(r.height-(grid.height+2)*scale)/2;const x=(e.clientX-r.left-ox)/scale-1,y=(e.clientY-r.top-oy)/scale-1;if(x>=0&&x<grid.width&&y>=0&&y<grid.height)onTarget([Number((grid.origin[0]+x*grid.resolution).toFixed(2)),Number((grid.origin[1]+(grid.height-y)*grid.resolution).toFixed(2))]);}}>
 <defs><pattern id="map-grid" width="5" height="5" patternUnits="userSpaceOnUse"><path d="M 5 0 L 0 0 0 5" fill="none" stroke="#293832" strokeWidth=".1"/></pattern></defs><rect width={grid.width} height={grid.height} fill="#17221e"/><rect width={grid.width} height={grid.height} fill="url(#map-grid)"/><path d={walls} fill="#506159"/>
 {pose&&state.world.scan.map(([x,y],i)=>{const wp=point([pose.x+x*Math.cos(pose.yaw)-y*Math.sin(pose.yaw),pose.y+x*Math.sin(pose.yaw)+y*Math.cos(pose.yaw)]);return <circle key={i} cx={wp[0]} cy={wp[1]} r=".16" fill="#94c7a8" opacity=".65"/>;})}
 {path&&<polyline points={path} fill="none" stroke="#8accad" strokeWidth=".4" strokeDasharray="1 .7"/>}
 {target&&(()=>{const p=point(target);return <g transform={`translate(${p[0]} ${p[1]})`}><circle r="1" fill="none" stroke="#bce6ce" strokeWidth=".3"/><path d="M-1.8 0h3.6M0-1.8v3.6" stroke="#bce6ce" strokeWidth=".2"/></g>;})()}
 {pos&&pose&&<g transform={`translate(${pos[0]} ${pos[1]}) rotate(${-pose.yaw*180/Math.PI})`}><circle r="3.2" fill="#76c6a4" opacity=".09"/><circle r="1.2" fill="#8cc9a8"/><path d="M2.5 0L.4-.65V.65Z" fill="#d4efe0"/></g>}
 </svg><div className="map-overlay"><span>{state.mode==='demo'?'合成演示场景':'实时环境'} / {grid.frame}</span><span>{(grid.width*grid.resolution).toFixed(1)} × {(grid.height*grid.resolution).toFixed(1)} m</span></div></div>;
}
