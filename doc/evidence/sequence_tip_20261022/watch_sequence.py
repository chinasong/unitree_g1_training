"""Read-only watcher: polls /api/state, logs every sequence / FSM change and a body line every 2 s."""
import http.cookiejar, json, sys, time, urllib.request

BASE = 'http://127.0.0.1:18766'
OUT = sys.argv[1]
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 900
op = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
op.open(BASE + '/api/session', timeout=5).read()


def state():
    return json.loads(op.open(BASE + '/api/state', timeout=5).read())


start, last_key, last_body, seen_live, rows = time.monotonic(), None, 0, False, []
with open(OUT, 'w') as log:
    def emit(row):
        row['t'] = round(time.monotonic() - start, 2)
        line = json.dumps(row, ensure_ascii=False)
        log.write(line + '\n'); log.flush(); print(line, flush=True)
    while time.monotonic() - start < LIMIT:
        try:
            s = state()
        except Exception as exc:
            emit(dict(kind='read_error', error=type(exc).__name__)); time.sleep(1); continue
        t, m = s['telemetry'], s['manual_remote']
        seq = m.get('sequence') or {}
        sport = ((t.get('safety_feedback') or {}).get('sport') or {}).get('raw') or {}
        key = (seq.get('status'), seq.get('current'), tuple(x['status'] for x in seq.get('steps', [])), seq.get('message'),
               t.get('fsm'), sport.get('fsm_id'), s['control']['owner'] is not None, m.get('armed'))
        joints = t.get('joints') or []
        body = dict(fsm=t.get('fsm'), sport=sport.get('fsm_id'), rpy=[round(v, 3) for v in (t.get('imu') or {}).get('rpy', [])],
                    max_dq=round(max((abs(j['dq']) for j in joints), default=0), 3),
                    max_temp=max((max(j['temperature']) for j in joints), default=None), age_ms=s.get('telemetry_age_ms'))
        if key != last_key:
            emit(dict(kind='change', seq_status=seq.get('status'), steps=[f"{x['label']}:{x['status']}" for x in seq.get('steps', [])],
                      message=seq.get('message'), owner=s['control']['owner'] is not None, armed=m.get('armed'), **body))
            last_key = key
        elif time.monotonic() - last_body > 2:
            emit(dict(kind='body', **body)); last_body = time.monotonic()
        seen_live = seen_live or seq.get('status') in ('running', 'waiting_operator')
        if seen_live and seq.get('status') in ('done', 'aborted') and seq.get('kind') == 'stop':
            emit(dict(kind='finished_stop_sequence')); break
        time.sleep(.2)
