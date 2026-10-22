type StreamSocket = Pick<WebSocket,'onmessage'|'onclose'|'onerror'|'close'>;

/** Renew the read session before every WebSocket; Orin restarts rotate its cookie. */
export function followState<T>(options:{
 refreshSession:()=>Promise<unknown>;
 createSocket:()=>StreamSocket;
 onValue:(value:T)=>void;
 onOffline:()=>void;
 onInvalid:()=>void;
 schedule?:(callback:()=>void,delay:number)=>ReturnType<typeof setTimeout>;
 cancel?:(timer:ReturnType<typeof setTimeout>)=>void;
}){
 let stopped=false,attempt=0,pending=false,socket:StreamSocket|undefined,timer:ReturnType<typeof setTimeout>|undefined;
 const schedule=options.schedule??setTimeout,cancel=options.cancel??clearTimeout;
 const retry=()=>{if(stopped||pending)return;pending=true;options.onOffline();timer=schedule(()=>{pending=false;void connect();},Math.min(5000,600*2**attempt++));};
 const connect=async()=>{
  try{
   await options.refreshSession();if(stopped)return;
   const current=options.createSocket();socket=current;
   current.onmessage=event=>{if(stopped||socket!==current||pending)return;try{const value=JSON.parse(event.data);attempt=0;options.onValue(value);}catch{options.onInvalid();}};
   current.onclose=retry;current.onerror=()=>current.close();
  }catch{retry();}
 };
 void connect();
 return ()=>{stopped=true;if(timer!==undefined)cancel(timer);socket?.close();};
}
