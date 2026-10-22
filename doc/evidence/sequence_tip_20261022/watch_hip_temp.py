"""Read-only: poll /api/state every 5 s; exit (so the caller is woken) when the left hip winding crosses LIMIT,
the FSM changes, or the bridge stays unreachable for 60 s. A trend line goes to the log every minute."""
import http.cookiejar, json, sys, time, urllib.request

BASE, OUT = 'http://127.0.0.1:18766', sys.argv[1]
LIMIT, DURATION = 85, 4 * 3600
op = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))


def state():
    return json.loads(op.open(BASE + '/api/state', timeout=5).read())


start = time.monotonic()
failing_since, first_fsm, last_line = None, None, 0
with open(OUT, 'a') as log:
    def emit(text):
        line = time.strftime('%H:%M:%S ') + text
        log.write(line + '\n'); log.flush(); print(line, flush=True)
    while time.monotonic() - start < DURATION:
        try:
            if failing_since is not None or first_fsm is None:
                op.open(BASE + '/api/session', timeout=5).read()
            t = state()['telemetry']
            failing_since = None
        except Exception as exc:
            failing_since = failing_since or time.monotonic()
            if time.monotonic() - failing_since > 60:
                emit(f'ALERT 读不到身体反馈已超过 60 s（{type(exc).__name__}）'); sys.exit(2)
            time.sleep(5); continue
        j = t['joints']
        hip, right = j[0]['temperature'], j[6]['temperature']
        first_fsm = t['fsm'] if first_fsm is None else first_fsm
        text = f"fsm {t['fsm']} 左髋 {hip} tau {j[0]['tau']:.1f} 右髋 {right} max {max(max(x['temperature']) for x in j)}"
        if max(hip) > LIMIT:
            emit('ALERT 左髋温度超过 85 ℃ ' + text); sys.exit(1)
        if t['fsm'] != first_fsm:
            emit('ALERT FSM 变化 ' + text); sys.exit(3)
        if time.monotonic() - last_line >= 60:
            emit(text); last_line = time.monotonic()
        time.sleep(5)
    emit('监控到时结束 ' + text)
