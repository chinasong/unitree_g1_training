import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { Body, BodyNode, Joint } from './types';

export default function BodyView({body,joints,selected,onSelect,fresh}:{body:Body;joints:Joint[];selected:number;onSelect:(index:number)=>void;fresh:boolean}) {
 const host=useRef<HTMLDivElement>(null);
 const latest=useRef({joints,selected,onSelect,fresh}); latest.current={joints,selected,onSelect,fresh};
 const [notice,setNotice]=useState('正在载入官方 G1 模型…');
 const [fallback,setFallback]=useState(false);
 useEffect(()=>{
  if(!host.current)return;
  const element=host.current;
  let renderer:THREE.WebGLRenderer;
  try{renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});}catch{setFallback(true);setNotice('WebGL 不可用 · 显示交互式身体示意');return;}
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setClearColor(0x141b1b,0);
  renderer.outputColorSpace=THREE.SRGBColorSpace;element.appendChild(renderer.domElement);
  const scene=new THREE.Scene();
  scene.add(new THREE.HemisphereLight(0xe4f5f1,0x2b3635,2.5));
  const light=new THREE.DirectionalLight(0xffffff,3.2);light.position.set(3,-4,6);scene.add(light);
  const rim=new THREE.DirectionalLight(0x77c9b7,1.2);rim.position.set(-2,3,3);scene.add(rim);
  const camera=new THREE.PerspectiveCamera(33,1,.01,50);camera.up.set(0,0,1);camera.position.set(2.5,-3.4,1.65);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0,.67);controls.enableDamping=true;controls.enablePan=false;controls.minDistance=1.8;controls.maxDistance=5;controls.maxPolarAngle=Math.PI*.52;
  const floor=new THREE.GridHelper(5,40,0x34433f,0x26322f);floor.rotation.x=Math.PI/2;scene.add(floor);
  const disk=new THREE.Mesh(new THREE.CircleGeometry(.6,72),new THREE.MeshBasicMaterial({color:0x21332e,transparent:true,opacity:.45,side:THREE.DoubleSide}));disk.position.z=-.005;scene.add(disk);
  const axes=new THREE.Group();
  const ring=new THREE.Mesh(new THREE.RingGeometry(.46,.464,80),new THREE.MeshBasicMaterial({color:0x4b7a6c,side:THREE.DoubleSide}));ring.position.z=.006;axes.add(ring);scene.add(axes);
  const nodes=new Map<number,{group:THREE.Group;base:THREE.Quaternion;axis:THREE.Vector3}>();
  const visuals:THREE.Mesh<THREE.BufferGeometry,THREE.MeshStandardMaterial>[]=[];
  const loader=new STLLoader();let disposed=false,failed=0,loaded=0,total=0;
  const geometries=new Set<THREE.BufferGeometry>();const materials=new Set<THREE.Material>();
  function build(node:BodyNode,parent:THREE.Object3D,inherited:number|null){
   const group=new THREE.Group();group.name=node.name;group.position.fromArray(node.position);
   const [w,x,y,z]=node.quaternion;group.quaternion.set(x,y,z,w);parent.add(group);
   const index=node.joint_index??inherited;group.userData.index=index;
   if(node.joint_index!==null)nodes.set(node.joint_index,{group,base:group.quaternion.clone(),axis:new THREE.Vector3().fromArray(node.axis)});
   for(const mesh of node.meshes){total++;loader.load('/api/model/meshes/'+mesh.file,geometry=>{
    if(disposed){geometry.dispose();return;}
    geometry.computeVertexNormals();geometries.add(geometry);
    const isDark=mesh.rgba[0]<.4;
    const material=new THREE.MeshStandardMaterial({color:isDark?0x343d3c:0xb6c1bd,metalness:.35,roughness:.56});
    material.userData.original=material.color.clone();materials.add(material);
    const visual=new THREE.Mesh(geometry,material);visual.userData.index=index;group.add(visual);visuals.push(visual);loaded++;
    setNotice(loaded+failed===total?(failed?'部分模型网格缺失，请同步官方依赖':''):'正在载入官方模型…');
   },undefined,()=>{if(!disposed){failed++;setNotice('部分模型网格缺失，请同步官方依赖');if(failed===total)setFallback(true);}});}
   node.children.forEach(child=>build(child,group,index));
  }
  build(body.tree,scene,null);
  const raycaster=new THREE.Raycaster();const mouse=new THREE.Vector2();let down=[0,0];
  const pointerDown=(event:PointerEvent)=>{down=[event.clientX,event.clientY];};
  const pick=(event:PointerEvent)=>{
   if(Math.hypot(event.clientX-down[0],event.clientY-down[1])>5)return;
   const r=element.getBoundingClientRect();mouse.set((event.clientX-r.left)/r.width*2-1,-(event.clientY-r.top)/r.height*2+1);raycaster.setFromCamera(mouse,camera);
   const hit=raycaster.intersectObjects(visuals)[0];if(hit&&typeof hit.object.userData.index==='number')latest.current.onSelect(hit.object.userData.index);
  };
  element.addEventListener('pointerdown',pointerDown);element.addEventListener('pointerup',pick);
  const resize=new ResizeObserver(()=>{const {width,height}=element.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/Math.max(1,height);camera.updateProjectionMatrix();});resize.observe(element);
  let frame=0;const q=new THREE.Quaternion();
  const render=()=>{
   if(disposed)return;
   frame=requestAnimationFrame(render);if(document.hidden)return;
   for(const joint of latest.current.joints){const node=nodes.get(joint.index);if(node)node.group.quaternion.copy(node.base).multiply(q.setFromAxisAngle(node.axis,joint.q??0));}
   for(const mesh of visuals){const mat=mesh.material;mat.color.copy(mat.userData.original);if(mesh.userData.index===latest.current.selected){mat.color.set(0x63b49c);mat.emissive.set(0x193f32);}else mat.emissive.set(0x000000);}
   controls.update();renderer.render(scene,camera);
  };render();
  return()=>{disposed=true;cancelAnimationFrame(frame);resize.disconnect();controls.dispose();element.removeEventListener('pointerdown',pointerDown);element.removeEventListener('pointerup',pick);geometries.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());floor.geometry.dispose();(floor.material as THREE.Material).dispose();disk.geometry.dispose();disk.material.dispose();ring.geometry.dispose();ring.material.dispose();renderer.dispose();renderer.domElement.remove();};
 },[body]);
 return <div className="body-view"><div className="webgl" ref={host} aria-label="可旋转的 G1 23 自由度身体模型"/>{fallback&&<svg className="body-fallback" viewBox="0 0 300 430"><g stroke="#859c93" strokeWidth="23" strokeLinecap="round"><path d="M150 55V210M105 120L75 185L65 245M195 120L225 185L235 245M125 213L115 305L110 385M175 213L185 305L190 385"/><path d="M108 125H192M120 210H180" strokeWidth="48"/></g><rect x="124" y="23" width="52" height="47" rx="17" fill="#64776e"/>{body.joints.map((joint,i)=>{const coords=[[125,213],[120,230],[118,248],[115,305],[110,366],[110,385],[175,213],[180,230],[182,248],[185,305],[190,366],[190,385],[150,199],[105,119],[98,137],[90,156],[75,185],[65,245],[195,119],[202,137],[210,156],[225,185],[235,245]][i];return <circle key={joint.index} cx={coords[0]} cy={coords[1]} r={selected===joint.index?9:5} fill={selected===joint.index?'#87d4b4':'#314c40'} onClick={()=>onSelect(joint.index)}><title>{joint.label}</title></circle>})}</svg>}
 <div className="body-top-label"><span className="micro">UNITREE / G1 EDU</span><span className="model-source">官方 23 DOF 模型</span></div>
 <div className="body-view-footer"><span>{fresh?'关节形态参考 · 未融合 IMU / 接触 / 支撑':'参考姿态 · 等待身体反馈'}</span><span>拖动旋转 · 点击关节</span></div>{notice&&<div className="model-notice">{notice}</div>}</div>;
}
