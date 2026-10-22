/** One held input, one in-flight velocity request. A release invalidates every continuation. */
export class Deadman {
 private generation=0;
 private active=false;
 private timer:ReturnType<typeof setTimeout>|undefined;
 private timeout:ReturnType<typeof setTimeout>|undefined;
 private velocity:Record<string,number>={};
 private send:(velocity:Record<string,number>)=>Promise<unknown>;
 private stop:()=>Promise<unknown>;
 private report:(message:string)=>void;
 constructor(send:(velocity:Record<string,number>)=>Promise<unknown>,stop:()=>Promise<unknown>,report:(message:string)=>void){this.send=send;this.stop=stop;this.report=report;}
 start(velocity:Record<string,number>){
  if(this.active)return;
  this.velocity=velocity;
  this.active=true;
  const id=++this.generation;
  const pulse=async()=>{
   if(!this.active||id!==this.generation)return;
   this.timeout=setTimeout(()=>{if(id===this.generation){this.report('速度请求响应超时，已停止续发；请检查连接');this.release();}},300);
   try{
    await this.send(this.velocity);
    if(id!==this.generation)return;
    clearTimeout(this.timeout);
    this.timer=setTimeout(pulse,100);
   }catch(error){
    if(id===this.generation){this.report(error instanceof Error?error.message:'速度操作失败');this.release();}
   }
  };
  void pulse();
 }
 update(velocity:Record<string,number>){
  if(Object.values(velocity).every(value=>value===0)){this.release();return;}
  this.velocity=velocity;
  if(!this.active)this.start(velocity);
 }
 release(){
  ++this.generation;
  clearTimeout(this.timer);clearTimeout(this.timeout);
  const held=this.active;this.active=false;
  if(held)void this.stop().catch(error=>this.report(error instanceof Error?error.message:'停止回执未确认；请观察机器人'));
  return held;
 }
}
