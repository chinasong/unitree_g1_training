import type {Snapshot} from './types';

type QueryRow = NonNullable<Snapshot['command_history']>[number];

// A command receipt is historical evidence, never proof of current readiness.
export function latestQueryReadback(history:QueryRow[], capability:string, mode:Snapshot['mode']):QueryRow|null {
 let latest:QueryRow|null=null;
 for(const row of history){
  if(row.capability!==capability||row.mode!==mode||!Number.isFinite(row.at)||
   !['read_result','read_unavailable','outcome_unknown','rejected'].includes(row.status)) continue;
  if(!latest||row.at>latest.at) latest=row;
 }
 return latest;
}

export function queryReadbackLabel(row:QueryRow):string {
 if(row.status==='read_result') return '已读取 · 历史样本';
 const code=row.sdk_code==null?'':String(row.sdk_code);
 const meaning:Record<string,string>={'7301':'LocoState 不可用 · 值未知','3203':'服务端未实现 API · 值未知','3104':'查询超时 · 值未知'};
 return code?`${code} · ${meaning[code]||'查询未成功 · 值未知'}`:'查询未成功 · 值未知';
}
