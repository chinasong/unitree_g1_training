/** Receipt age keeps advancing when the WebSocket stops delivering snapshots. */
export function telemetryAgeMs(reportedAge:number|null|undefined, receivedAt:number, now:number):number|null {
 if(typeof reportedAge!=='number'||!Number.isFinite(reportedAge)||reportedAge<0||!Number.isFinite(receivedAt)||receivedAt<=0||!Number.isFinite(now))return null;
 return reportedAge+Math.max(0,now-receivedAt);
}
