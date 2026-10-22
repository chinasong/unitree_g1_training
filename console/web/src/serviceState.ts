export type ServiceState = {name:string;status:number;protect:number};

export function serviceRows(data:unknown):ServiceState[]|null{
 if(!Array.isArray(data)||data.length>128||!data.every(r=>r!==null&&typeof r==='object'&&typeof r.name==='string'&&Number.isInteger(r.status)&&Number.isInteger(r.protect)))return null;
 return data.map(r=>({name:r.name,status:r.status,protect:r.protect}));
}

// G1 RobotStateClient documentation: this enum is the reverse of a switch input.
export function serviceStatus(status:number):string{
 return status===0?'开启':status===1?'关闭':`未知枚举 ${status}`;
}
