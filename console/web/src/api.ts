const BASE = import.meta.env.VITE_API_BASE || '';
export class ApiError extends Error { constructor(public code:string, message:string) {super(message)} }
export async function api<T>(path:string, data?:unknown):Promise<T> {
 const response=await fetch(BASE+path,{method:data===undefined?'GET':'POST',credentials:'same-origin',headers:{'Content-Type':'application/json','X-G1-Intent':'operator'},body:data===undefined?undefined:JSON.stringify(data)});
 // A proxy with no backend (connector window closed) answers 502 with an empty body, not JSON.
 const text=await response.text();
 let result:any;
 try{result=text?JSON.parse(text):{};}catch{result={};}
 if(!response.ok)throw new ApiError(result.error?.code||(response.status===502||response.status===504?'BRIDGE_DOWN':'HTTP_ERROR'),
  result.error?.message||(response.status===502||response.status===504?'连不上机器人服务：连接器窗口可能已关闭，请双击启动器重新连接':`接口返回 ${response.status}`));
 return result as T;
}
export function websocketURL(){ const url=new URL(BASE||location.origin,location.origin);url.protocol=url.protocol==='https:'?'wss:':'ws:';url.pathname='/api/live';return url.href; }
