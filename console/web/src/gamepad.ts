export type GameInput={vx:number;vy:number;omega:number};

type Handlers={
 onInput:(input:GameInput)=>void;
 onStop:()=>void;
 onGear:(change:-1|1)=>void;
 onDisarm:()=>void;
 onDisconnect:()=>void;
};

const ZERO:GameInput={vx:0,vy:0,omega:0};
const deadzone=(value:number)=>Math.abs(value)<.15?0:value;

/** Poll the first connected browser gamepad and report edge-triggered buttons. */
export function watchGamepad(handlers:Handlers){
 let frame=0,connected=false,previousButtons:boolean[]=[];
 const poll=()=>{
  const pad=Array.from(navigator.getGamepads?.()||[]).find((item):item is Gamepad=>!!item&&item.connected);
  if(!pad){
   if(connected){connected=false;previousButtons=[];handlers.onDisconnect();}
   handlers.onInput(ZERO);
   frame=requestAnimationFrame(poll);
   return;
  }
  connected=true;
  handlers.onInput({vx:deadzone(-(pad.axes[1]||0)),vy:deadzone(-(pad.axes[0]||0)),omega:deadzone(-(pad.axes[2]||0))});
  const pressed=pad.buttons.map(button=>button.pressed);
  const edge=(index:number)=>pressed[index]&&!previousButtons[index];
  if(edge(1))handlers.onStop();
  if(edge(4))handlers.onGear(-1);
  if(edge(5))handlers.onGear(1);
  if(edge(9))handlers.onDisarm();
  previousButtons=pressed;
  frame=requestAnimationFrame(poll);
 };
 frame=requestAnimationFrame(poll);
 return()=>cancelAnimationFrame(frame);
}
