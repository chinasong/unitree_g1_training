"""7301 read-only comparison script: fixed getters only, real console service, recording fake robot."""
import io
import json
import time
import urllib.error
import urllib.request

import pytest
from fastapi.testclient import TestClient

from g1_console.adapters import DemoAdapter
from g1_console.app import create_app
from g1_console.capabilities import BY_ID, READ_OPERATIONS
from g1_console.config import Settings
from scripts.compare_g1_locostate import (LOCOSTATE_PROBES, PROBES, ReadOnlyConsole, outcome, run, summarize,
                                          validate_base)


class Response:
    def __init__(self, status, body):
        self.status, self._body = status, body

    def read(self, *args):
        return self._body

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


class AppOpener:
    """Routes the script's urllib calls into the in-process app and records every request."""

    def __init__(self, client):
        self.client, self.requests = client, []

    def open(self, target, timeout=None):
        if isinstance(target, urllib.request.Request):
            path = target.full_url.removeprefix("http://127.0.0.1:18765")
            headers = {k: v for k, v in target.header_items()}
            self.requests.append(("POST", path, json.loads(target.data)))
            response = self.client.post(path, content=target.data, headers=headers)
        else:
            path = target.removeprefix("http://127.0.0.1:18765")
            self.requests.append(("GET", path, None))
            response = self.client.get(path)
        if response.status_code >= 400:
            raise urllib.error.HTTPError(path, response.status_code, "error", {}, io.BytesIO(response.content))
        return Response(response.status_code, response.content)


class ModeRobot(DemoAdapter):
    """LocoState getters answer 7301 in the listed FSMs, mirroring the hypothesis or its negation."""
    mode = "robot"
    unavailable_in = frozenset((0, 1))

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.fsm, self.calls = 0, []

    def sample(self):
        data = super().sample()
        data["mode_machine"] = 4
        data["safety_feedback"] = {"sport": {"current": True, "raw": {"fsm_id": self.fsm, "fsm_mode": 0}}}
        return data

    def execute(self, op, params):
        self.calls.append(op)
        assert op in READ_OPERATIONS, f"comparison must never dispatch {op}"
        if op in LOCOSTATE_PROBES and self.fsm in self.unavailable_in:
            return 7301, 0  # Placeholder zero must not surface as a value.
        return super().execute(op, params)

    def close(self):
        # Match UnitreeAdapter.close (no stop on shutdown), not DemoAdapter.close.
        self.closed = True


def wait_fsm(client, fsm):
    for _ in range(100):
        if client.get("/api/state").json()["telemetry"].get("fsm") == fsm:
            return
        time.sleep(.02)
    raise AssertionError(f"service never observed fsm={fsm}")


def comparison(monkeypatch, tmp_path, sequence, unavailable_in=frozenset((0, 1))):
    robots = []

    def factory(*args, **kwargs):
        robot = ModeRobot()
        robot.unavailable_in = unavailable_in
        robots.append(robot)
        return robot

    monkeypatch.setattr("g1_console.service.UnitreeAdapter", factory)
    app = create_app(Settings(manual_remote=True, damping_test=True, data_dir=tmp_path / "state"))
    with TestClient(app, base_url="http://127.0.0.1:18765") as client:
        client.get("/api/session")
        assert client.post("/api/connect", json={"mode": "robot", "interface": "eth0"},
                           headers={"X-G1-Intent": "operator"}).status_code == 200
        robot = robots[0]
        opener = AppOpener(client)
        console = ReadOnlyConsole("http://127.0.0.1:18765", opener=opener)
        steps = iter(sequence)
        now = [0.]

        def advance(seconds):
            # The human changes mode between samples; wait until the service's own polling sees it.
            now[0] += 1.
            fsm = next(steps, None)
            if fsm is not None:
                robot.fsm = fsm
                wait_fsm(client, fsm)

        robot.fsm = sequence[0]
        wait_fsm(client, sequence[0])
        next(steps)
        summary = run(console, duration=len(sequence), interval=1, output=tmp_path / "out.jsonl", note="test",
                      clock=lambda: now[0], sleep=advance, log=lambda *_: None)
        final = client.get("/api/state").json()
    return summary, robot, opener, final, tmp_path / "out.jsonl"


def test_base_url_and_operations_are_fixed():
    for url in ("http://192.0.2.28:8765", "https://127.0.0.1:18765", "http://127.0.0.1:18765/api", "http://u:p@127.0.0.1:1"):
        with pytest.raises(ValueError):
            validate_base(url)
    assert set(PROBES) <= READ_OPERATIONS and all(BY_ID[op]["risk"] == "read" for op in PROBES)

    class Refuse:
        def open(self, *args, **kwargs):
            raise AssertionError("no request may be sent")

    console = ReadOnlyConsole("http://127.0.0.1:18765", opener=Refuse())
    for op in ("damp", "stop", "velocity", "start", "lie_to_stand", "get_actions", "get_volume"):
        with pytest.raises(ValueError):
            console.read(op)
    for path in ("/api/control/acquire", "/api/connect", "/api/command"):
        with pytest.raises(ValueError):
            console._get(path)


def test_sdk_error_never_yields_a_value():
    assert outcome({"error": {"code": "SDK_REJECTED", "sdk_code": 7301}, "data": 0}) == dict(code=7301, error="SDK_REJECTED", value=None)
    assert outcome({"status": "accepted", "data": 0}) == dict(code=0, error=None, value=0)


def test_supports_hypothesis_through_real_service_without_writes(monkeypatch, tmp_path):
    summary, robot, opener, final, output = comparison(monkeypatch, tmp_path, [0, 1, 4, 500])
    assert summary["verdict"] == "supports"
    assert summary["by_fsm"]["0"]["codes"]["get_balance"] == {"7301": 1}
    assert summary["by_fsm"]["500"]["codes"]["get_balance"] == {"0": 1}
    assert "get_balance" not in summary["by_fsm"]["1"]["values"]  # 7301 placeholder zero discarded
    assert set(robot.calls) <= set(PROBES)
    assert {path for _, path, _ in opener.requests} <= {"/api/session", "/api/state", "/api/command"}
    assert all(body["capability"] in PROBES for method, _, body in opener.requests if method == "POST")
    assert final["control"]["owner"] is None and final["manual_remote"]["armed"] is False
    lines = [json.loads(line) for line in output.read_text().splitlines()]
    assert lines[0]["kind"] == "header" and lines[0]["writes_sent"] == 0 and lines[-1]["kind"] == "summary"


def test_refutes_when_locostate_stays_unavailable_in_active_modes(monkeypatch, tmp_path):
    summary, robot, *_ = comparison(monkeypatch, tmp_path, [0, 4, 500], unavailable_in=frozenset((0, 1, 4, 500)))
    assert summary["verdict"] == "refutes"
    assert set(robot.calls) <= set(PROBES)


def test_passive_only_is_inconclusive(monkeypatch, tmp_path):
    summary, *_ = comparison(monkeypatch, tmp_path, [0, 1, 0])
    assert summary["verdict"] == "inconclusive"


def test_samples_with_moving_fsm_are_excluded():
    moving = dict(fsm=500, fsm_stable=False, reads=[dict(op=op, code=0, value=1) for op in LOCOSTATE_PROBES])
    summary = summarize([moving])
    assert summary["verdict"] == "inconclusive" and summary["unstable_samples"] == 1 and summary["stable_samples"] == 0


@pytest.mark.parametrize("option,value", [
    ("--interval", "0.4"),
    ("--duration", "0"),
    ("--duration", "-1"),
    ("--duration", "3601"),
])
def test_main_rejects_invalid_sampling_bounds_before_creating_console(monkeypatch, option, value):
    import scripts.compare_g1_locostate as compare

    constructed = []
    monkeypatch.setattr(compare, "ReadOnlyConsole", lambda *args, **kwargs: constructed.append((args, kwargs)))
    monkeypatch.setattr(compare, "run", lambda *args, **kwargs: pytest.fail("run must not be called"))
    monkeypatch.setattr(compare.sys, "argv", ["compare_g1_locostate.py", option, value])

    with pytest.raises(SystemExit):
        compare.main()

    assert constructed == []


def test_main_accepts_sampling_boundaries(monkeypatch, tmp_path):
    import scripts.compare_g1_locostate as compare

    console = object()
    constructed = []
    calls = []
    output = tmp_path / "out.jsonl"

    def make_console(base):
        constructed.append(base)
        return console

    def fake_run(*args):
        calls.append(args)
        return {"verdict": "inconclusive"}

    monkeypatch.setattr(compare, "ReadOnlyConsole", make_console)
    monkeypatch.setattr(compare, "run", fake_run)
    monkeypatch.setattr(compare.sys, "argv", [
        "compare_g1_locostate.py", "--interval", "0.5", "--duration", "3600", "--output", str(output),
    ])

    compare.main()

    assert constructed == ["http://127.0.0.1:18765"]
    assert calls == [(console, 3600.0, 0.5, output, "")]
