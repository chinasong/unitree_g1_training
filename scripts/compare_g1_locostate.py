#!/usr/bin/env python3
"""Read-only 7301 comparison: sample fixed sport getters while a human changes modes with the physical remote.

The script never acquires control and never sends a write. It talks only to the local SSH bridge:
GET /api/session, GET /api/state and POST /api/command for the fixed getters in PROBES.
Hypothesis under test: 7003-7006 return 7301 only in passive FSMs (0 zero torque, 1 damp).
"""
import argparse
import http.cookiejar
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_console.capabilities import READ_OPERATIONS

FSM_PROBES = ("get_fsm", "get_fsm_mode")
LOCOSTATE_PROBES = ("get_balance", "get_swing_height", "get_stand_height", "get_phase")
PROBES = FSM_PROBES + LOCOSTATE_PROBES
PASSIVE_FSM = frozenset((0, 1))
LOCOSTATE_UNAVAILABLE = 7301
assert set(PROBES) <= READ_OPERATIONS, "probe list must stay within the console's read-only catalog"


def validate_base(url):
    parsed = urllib.parse.urlsplit(url)
    if (parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost", "::1") or parsed.username
            or parsed.password or parsed.path not in ("", "/") or parsed.query or parsed.fragment):
        raise ValueError("仅接受本机 SSH 转发入口，例如 http://127.0.0.1:18765")
    return url.rstrip("/")


class ReadOnlyConsole:
    """HTTP client that can only reach the session, state and fixed read getters."""

    def __init__(self, base, opener=None):
        self.base = validate_base(base)
        self.opener = opener or urllib.request.build_opener(urllib.request.ProxyHandler({}),
            urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
        # Owner is required by the command contract; read getters never check or acquire it.
        self.owner = "locostate-probe-" + uuid.uuid4().hex[:12]
        self.sequence = 0

    def _get(self, path):
        if path not in ("/api/session", "/api/state"):
            raise ValueError(f"只读对照不访问 {path}")
        with self.opener.open(self.base + path, timeout=5) as response:
            return json.load(response)

    def login(self):
        # Session cookie stays in memory; never printed or persisted.
        return self._get("/api/session")

    def state(self):
        return self._get("/api/state")

    def read(self, op):
        if op not in PROBES or op not in READ_OPERATIONS:
            raise ValueError(f"{op} 不在固定只读查询清单内")
        self.sequence += 1
        body = json.dumps(dict(owner=self.owner, capability=op, params={}, request_id=uuid.uuid4().hex,
                               issued_at=time.time(), sequence=self.sequence)).encode()
        request = urllib.request.Request(self.base + "/api/command", data=body, method="POST",
                                         headers={"Content-Type": "application/json", "X-G1-Intent": "operator"})
        started = time.monotonic()
        try:
            with self.opener.open(request, timeout=5) as response:
                payload, status = json.load(response), response.status
        except urllib.error.HTTPError as exc:
            payload, status = json.load(exc), exc.code
        return dict(op=op, http=status, elapsed_ms=round((time.monotonic() - started) * 1000, 2), **outcome(payload))


def outcome(payload):
    """Normalize a console reply; non-zero SDK replies keep value unknown (None)."""
    if "error" in payload:
        error = payload["error"]
        return dict(code=error.get("sdk_code"), error=error.get("code"), value=None)
    return dict(code=0, error=None, value=payload.get("data"))


def check_service(state):
    if state.get("mode") != "robot" or not state.get("connected"):
        raise ValueError("需要已连接真实机器人的控制台服务；没有执行任何查询")
    return dict(run_id=state.get("run_id"), read_only=state.get("read_only"),
                owner=state.get("control", {}).get("owner"), armed=state.get("manual_remote", {}).get("armed"))


def passive(state):
    """Independent FSM evidence from the service's own polling and HG broadcast."""
    feedback = state.get("telemetry", {}).get("safety_feedback", {})
    sport = feedback.get("sport") or {}
    return dict(polled_fsm=state.get("telemetry", {}).get("fsm"),
                hg_fsm_id=(sport.get("raw") or {}).get("fsm_id") if sport.get("current") else None,
                hg_fsm_mode=(sport.get("raw") or {}).get("fsm_mode") if sport.get("current") else None,
                owner=state.get("control", {}).get("owner"))


def sample(console, index):
    before = passive(console.state())
    reads = [console.read(op) for op in PROBES]
    after = passive(console.state())
    by_op = {r["op"]: r for r in reads}
    fsm = by_op["get_fsm"]["value"] if by_op["get_fsm"]["code"] == 0 else None
    # A sample counts only when the FSM did not move during the probe window.
    stable = fsm is not None and before["polled_fsm"] == fsm == after["polled_fsm"]
    return dict(index=index, at=time.time(), fsm=fsm, fsm_stable=stable, before=before, after=after, reads=reads)


def summarize(samples):
    groups = {}
    for s in samples:
        if not s["fsm_stable"]:
            continue
        group = groups.setdefault(s["fsm"], dict(samples=0, codes={op: {} for op in LOCOSTATE_PROBES}, values={}))
        group["samples"] += 1
        for r in s["reads"]:
            if r["op"] in LOCOSTATE_PROBES:
                key = str(r["code"])
                group["codes"][r["op"]][key] = group["codes"][r["op"]].get(key, 0) + 1
                if r["code"] == 0:
                    group["values"].setdefault(r["op"], r["value"])
    active = {fsm: g for fsm, g in groups.items() if fsm not in PASSIVE_FSM}
    if not active:
        verdict = "inconclusive"
        reason = "没有在非被动 FSM（0/1 以外）下取得稳定样本，无法判断"
    elif any(g["codes"][op].get("0") for g in active.values() for op in LOCOSTATE_PROBES):
        verdict = "supports"
        reason = "非被动 FSM 下至少一项 7003–7006 返回 0，支持“7301 仅出现在被动模式”"
    elif all(set(g["codes"][op]) == {str(LOCOSTATE_UNAVAILABLE)} for g in active.values() for op in LOCOSTATE_PROBES):
        verdict = "refutes"
        reason = "非被动 FSM 下 7003–7006 仍全部为 7301，否定该假设"
    else:
        verdict = "inconclusive"
        reason = "非被动 FSM 下出现 7301 以外的错误码，需逐条查看原始记录"
    return dict(verdict=verdict, reason=reason, stable_samples=sum(g["samples"] for g in groups.values()),
                unstable_samples=sum(1 for s in samples if not s["fsm_stable"]),
                by_fsm={str(k): v for k, v in sorted(groups.items())})


def run(console, duration, interval, output, note, clock=time.monotonic, sleep=time.sleep, log=print):
    console.login()
    header = dict(kind="header", started_at=time.time(), note=note, probes=PROBES, interval_s=interval,
                  duration_s=duration, service=check_service(console.state()),
                  writes_sent=0, control_acquired=False)
    samples = []
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w") as stream:
        stream.write(json.dumps(header, ensure_ascii=False) + "\n")
        deadline, index, last_fsm = clock() + duration, 0, object()
        try:
            while clock() < deadline:
                started = clock()
                record = sample(console, index)
                samples.append(record)
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
                stream.flush()
                codes = " ".join(f"{r['op']}={r['code']}" for r in record["reads"] if r["op"] in LOCOSTATE_PROBES)
                marker = "  <- FSM 变化" if record["fsm"] != last_fsm else ""
                log(f"[{index:04d}] fsm={record['fsm']} stable={record['fsm_stable']} {codes}{marker}")
                last_fsm, index = record["fsm"], index + 1
                sleep(max(0., interval - (clock() - started)))
        except KeyboardInterrupt:
            log("已手动停止采样")
        summary = dict(kind="summary", finished_at=time.time(), samples=len(samples), **summarize(samples))
        stream.write(json.dumps(summary, ensure_ascii=False) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description="7301 只读对照：人用实体遥控器切换模式时，固定读取 7001–7006；不取得控制权、不发送任何写入")
    parser.add_argument("--url", default="http://127.0.0.1:18765")
    parser.add_argument("--duration", type=float, default=300, help="采样总时长（秒），默认 300；Ctrl-C 可提前结束")
    parser.add_argument("--interval", type=float, default=2, help="采样间隔（秒），默认 2，最小 0.5")
    parser.add_argument("--note", default="", help="记录现场条件，例如保护方式、操作人、遥控器步骤")
    parser.add_argument("--output", type=Path,
                        default=ROOT / ".g1-deploy/research" / time.strftime("locostate-compare-%Y%m%d-%H%M%S.jsonl"))
    args = parser.parse_args()
    if args.interval < .5 or not 0 < args.duration <= 3600:
        raise SystemExit("间隔至少 0.5 秒，时长 0–3600 秒")
    try:
        summary = run(ReadOnlyConsole(args.url), args.duration, args.interval, args.output, args.note)
    except Exception as exc:
        raise SystemExit(f"对照中止：{type(exc).__name__}: {exc}") from None
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"原始记录：{args.output}")


if __name__ == "__main__":
    main()
