"""No hardware writes: adapter injection, real SDK message/CRC, HTTP and closed-loop tests."""
import asyncio
import json
import base64
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time
from uuid import uuid4
import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError
from g1_console.adapters import DemoAdapter, UnitreeAdapter, decode_lowstate
from g1_console.app import create_app
from g1_console.body import BODY, JOINTS
from g1_console.capabilities import CATALOG, READ_OPERATIONS, action_ids, firmware_action_catalog, validate_params
from g1_console.config import ConsoleError, Settings
from g1_console.contracts import CameraFrame, Command, Goal, Observation
from g1_console.navigation import World, plan, local_clear
from g1_console.ros_bridge import scan_points, occupancy_grid
from g1_console.service import ConsoleService
from g1_console.simulation import observe

OWNER = "test-owner-00000000001"
OTHER = "test-owner-00000000002"


def command(op, seq=1, **extra):
    return Command(owner=OWNER, capability=op, sequence=seq, issued_at=time.time(), request_id=uuid4().hex, **extra)


def goal(x=-.8, y=-1.4, seq=1):
    return Goal(owner=OWNER, x=x, y=y, sequence=seq, issued_at=time.time(), request_id=uuid4().hex)


def run(coro):
    return asyncio.run(coro)


@pytest.fixture
def service(tmp_path):
    return ConsoleService(Settings(data_dir=tmp_path))


async def connected(service):
    await service.connect("demo", "eth0")
    await service.acquire(OWNER, True)
    return service.adapter


def test_exact_23_sparse_body_and_sdk_decode():
    assert len(JOINTS) == 23
    assert [j["index"] for j in JOINTS] == list(range(13)) + list(range(15, 20)) + list(range(22, 27))
    motors = [NS(q=i / 10, dq=i, ddq=0, tau_est=i * 2, vol=49, temperature=[i, i+1], motorstate=0, mode=1, sensor=[11,12], reserve=[0,1,2,3]) for i in range(35)]
    imu = NS(rpy=[0,0,0], gyroscope=[0,0,0], accelerometer=[0,0,9.81], quaternion=[1,0,0,0], temperature=30)
    result = decode_lowstate(NS(motor_state=motors, imu_state=imu, tick=20, mode_pr=0, mode_machine=0, version=[1,0]))
    assert result["joints"][18]["index"] == 22
    assert result["joints"][18]["q"] == 2.2
    assert result['joints'][18]['sensor'] == [11,12]
    assert result['joints'][18]['reserve'] == [0,1,2,3] and result['mode_pr'] == 0
    assert all(j["range"][0] < j["range"][1] for j in JOINTS)
    assert BODY["hands"] is False
    motors[22].q = math.nan
    with pytest.raises(ValueError):
        decode_lowstate(NS(motor_state=motors, imu_state=imu))


def test_real_hg_crc_and_frozen_tick():
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.utils.crc import CRC
    adapter = object.__new__(UnitreeAdapter)
    adapter.read_only = adapter.verify_only = adapter.accessory_test = adapter.damping_test = False
    adapter.closed, adapter.tick, adapter.received = False, None, 0
    adapter.lock, adapter.crc, adapter.bad_packets = threading.Lock(), CRC(), 0
    message = unitree_hg_msg_dds__LowState_()
    message.tick = 50
    message.motor_state[22].q = .4
    message.crc = adapter.crc.Crc(message)
    adapter._lowstate(message)
    first = adapter.received
    assert first > 0 and adapter.low["joints"][18]["q"] == .4
    adapter._lowstate(message)
    assert adapter.received == first  # Repeated packet cannot refresh a dead robot.
    message.tick += 1  # Deliberately leave the old CRC.
    adapter._lowstate(message)
    assert adapter.bad_packets == 1 and adapter.received == first
    message.crc = adapter.crc.Crc(message)
    adapter._lowstate(message)
    assert adapter.tick == 51


def test_capabilities_are_explicit_and_firmware_driven():
    assert len(CATALOG) == 61 and len({c["id"] for c in CATALOG}) == 61
    assert action_ids({"actions":[{"id":17}, {"action_id":99}]}) == {17,99}
    assert action_ids({"clap":17,"release":99}) == {17,99}
    assert action_ids({"unrecognized":"all actions available"}) == set()
    for op, params in (("raw_rpc", {}), ("velocity", {"vx":float("nan")}), ("velocity", {"vx":1}), ("volume", {"value":101}), ("fsm", {"value":999})):
        with pytest.raises(ConsoleError):
            validate_params(op, params)


def test_nested_real_firmware_catalog_keeps_constraints_and_named_entries():
    # Shape observed on the G1 EDU; named motions have no ExecuteAction ID.
    data = [[{"id": 1, "name": "turn_back_wave", "fsm": [500, 501]},
             {"id": 31, "name": "extend_right_arm_forward", "mode_machine": [4]}],
            [{"name": "Waist_Drum_Dance", "time": 9.5},
             {"name": "Scratch_head", "time": 8.1}]]
    assert action_ids(data) == {1, 31}
    entries = firmware_action_catalog(data)
    assert entries[0] == dict(id=1, name="turn_back_wave", fsm=[500, 501])
    assert entries[1]["mode_machine"] == [4]
    assert entries[2] == dict(id=None, name="Waist_Drum_Dance", duration_s=9.5)
    assert action_ids([[True, {"id": "99"}], [{"name": "unnumbered", "time": 3}]]) == set()
    assert firmware_action_catalog({"unrecognized": {"id": 99}}) == []


def test_default_is_disconnected_and_null(service):
    state = service.snapshot()
    assert state["mode"] == "disconnected" and not state["connected"]
    assert all(j["q"] is None for j in state["telemetry"]["joints"])
    assert state["hardware"]["battery"] is None


def test_single_controller_and_switch_protection(service):
    async def scenario():
        await connected(service)
        with pytest.raises(ConsoleError, match="另一个窗口"):
            await service.acquire(OTHER, True)
        with pytest.raises(ConsoleError, match="释放控制权"):
            await service.connect("robot", "eth0")
        await service.release(OWNER)
        await service.acquire(OTHER, True)
        assert service.owner == OTHER
    run(scenario())


def test_velocity_lease_timeout_stops_and_no_auto_resume(service):
    async def scenario():
        adapter = await connected(service)
        await service.command(command("velocity", params={"vx":.2}))
        assert adapter.velocity[0] == .2
        service.lease_until = time.monotonic() - 1
        await service.tick()
        assert adapter.velocity == [0,0,0] and service.owner is None
        await service.tick()
        assert adapter.velocity == [0,0,0]
    run(scenario())


def test_velocity_has_independent_deadline(service):
    async def scenario():
        adapter = await connected(service)
        await service.command(command("velocity", params={"vx":.2}))
        service.motion_until = time.monotonic() - .01
        await service.tick()
        assert service.owner == OWNER and adapter.velocity == [0,0,0]
    run(scenario())


def test_stop_rejects_delayed_velocity_and_idempotency(service):
    async def scenario():
        adapter = await connected(service)
        calls = []
        original = adapter.execute
        def execute(op, params):
            calls.append(op)
            return original(op, params)
        adapter.execute = execute
        wave = command("wave", seq=1)
        a = await service.command(wave)
        assert await service.command(wave) == a
        assert calls.count("wave") == 1
        with pytest.raises(ConsoleError) as conflict:
            await service.command(wave.model_copy(update={"capability":"start"}))
        assert conflict.value.code == "IDEMPOTENCY_CONFLICT"
        await service.command(command("stop", seq=5))
        with pytest.raises(ConsoleError) as old:
            await service.command(command("velocity", seq=4, params={"vx":.2}))
        assert old.value.code == "OLD_COMMAND" and "velocity" not in calls
        expired = command("velocity", seq=6, params={"vx":.2}).model_copy(update={"issued_at":time.time()-1})
        with pytest.raises(ConsoleError) as stale:
            await service.command(expired)
        assert stale.value.code == "EXPIRED_COMMAND"
    run(scenario())


def test_sdk_rejection_never_reports_success_and_guarded_requires_ack(service):
    async def scenario():
        adapter = await connected(service)
        with pytest.raises(ConsoleError) as unack:
            await service.command(command("zero_torque"))
        assert unack.value.code == "RISK_ACK_REQUIRED"
        adapter.execute = lambda op, p: (3102, None)
        with pytest.raises(ConsoleError) as failed:
            await service.command(command("wave", seq=2))
        assert failed.value.code == "SDK_REJECTED"
        assert not any(e["kind"] == "command_accepted" for e in service.events)
    run(scenario())


def test_fsm_ack_requires_new_readback(service):
    async def scenario():
        await connected(service)
        result = await service.command(command("start"))
        assert result["status"] == "accepted" and service.pending
        service.telemetry["fsm_received"] = service.pending["issued"] - 1
        # Freeze just the sample to prove stale readback cannot verify a new command.
        async def frozen(): pass
        service._sample = frozen
        await service.tick()
        assert service.pending
        service.telemetry["fsm_received"] = time.monotonic()
        await service.tick()
        assert service.pending is None
        assert any(e["kind"] == "feedback_verified" for e in service.events)
    run(scenario())


def test_world_rejects_stale_replay_wrong_frame_and_partial_scan(service):
    world = World()
    observation = observe([-1.4,-1.4,0], 1, True)
    world.update(observation)
    for update in ({"sequence":1}, {"sequence":2,"captured_at":time.time()-2}, {"sequence":2,"source":"other"}, {"sequence":2,"frame":"unrelated"}):
        with pytest.raises(ConsoleError):
            world.update(observation.model_copy(update=update))
    async def scenario():
        await connected(service)
        service.world.observation.coverage_radians = math.pi
        with pytest.raises(ConsoleError) as failed:
            await service.command(command("velocity", params={"vx":.2}))
        assert failed.value.code == "SCAN_COVERAGE"
    run(scenario())


def test_path_respects_body_footprint_and_unknown():
    grid = observe([-1.4,-1.4,0], 1, True).grid
    path = plan(grid, (-1.4,-1.4), (-.8,-1.4), .32)
    assert len(path) > 1
    with pytest.raises(ConsoleError):
        plan(grid, (-1.4,-1.4), (0,0), .32)
    cells = list(grid.cells)
    cells[16 * 60 + 22] = -1
    with pytest.raises(ConsoleError):
        plan(grid.model_copy(update={"cells":cells}), (-1.4,-1.4), (-.75,-1.35), .32)
    assert not local_clear([(0.6,0)], .3, 0, 0, .32)
    assert not local_clear([(0,.2)], 0, 0, .3, .32)
    assert local_clear([(2,0)], .2, 0, 0, .32)


def test_demo_navigation_arrives_using_measured_pose(service):
    async def scenario():
        adapter = await connected(service)
        await service.goal(goal())
        assert service.navigation["status"] == "running"
        for _ in range(240):
            adapter.last -= .08  # Advance the kinematic sensor; no wall-clock wait.
            service.next_nav = 0
            await service.tick()
            await service.heartbeat(OWNER)
            if service.navigation["status"] != "running":
                break
        assert service.navigation["status"] == "succeeded", service.navigation
        assert math.dist(adapter.pose[:2], [-.8,-1.4]) < .13
        assert adapter.velocity == [0,0,0]
        assert service.memories[-1]["status"] == "succeeded"
        assert service.memories[-1]["mode"] == "demo"
    run(scenario())


def test_stuck_robot_replans_then_fails_instead_of_claiming_arrival(service):
    async def scenario():
        adapter = await connected(service)
        await service.goal(goal())
        service.navigation["best_distance"] = .6
        for _ in range(4):
            service.navigation["last_progress"] = time.monotonic() - 6
            service.next_nav = 0
            # Kinematic pose cannot advance without sample time; reset every tick.
            adapter.pose = [-1.4,-1.4,0]
            await service.tick()
        assert service.navigation["status"] == "failed"
        assert service.navigation["corrections"] == 3
        assert adapter.velocity == [0,0,0]
        assert not any(e["kind"] == "goal_verified" for e in service.events)
    run(scenario())


def test_real_ingress_stale_obstacle_and_tilt_watchdog(service):
    async def scenario():
        adapter = await connected(service)
        await service.command(command("velocity", params={"vx":.2}))
        # Simulate already active motion, then test real-input watchdog rules on a fake transport.
        # New real motion is now rejected at the final dispatch boundary.
        adapter.mode = "robot"  # Injected transport; this never instantiates DDS.
        stale = service.world.observation.model_copy(update={"captured_at":time.time()-2})
        service.world.observation = stale
        await service.tick()
        assert adapter.velocity == [0,0,0]
        assert any(e.get("reason") == "NO_OBSTACLE_FEEDBACK" for e in service.events)
        fresh = observe(adapter.pose, 2, True)
        fresh.scan[0] = (.25,0)
        await service.observation(fresh)
        with pytest.raises(ConsoleError) as blocked:
            await service.command(command("velocity", seq=2, params={"vx":.2}))
        assert blocked.value.code == "OBSTACLE_NEAR"
        await service.observation(observe(adapter.pose, 3, True))
        original = adapter.sample
        def tilted():
            value = original()
            value["imu"]["rpy"][0] = .5
            return value
        adapter.sample = tilted
        await service.tick()
        with pytest.raises(ConsoleError) as tilted_error:
            await service.command(command("velocity", seq=3, params={"vx":.2}))
        assert tilted_error.value.code == "BODY_TILT"
    run(scenario())


def test_map_revision_causes_correction(service):
    async def scenario():
        await connected(service)
        await service.goal(goal())
        service.world.grid.revision = 2
        service.next_nav = 0
        await service.tick()
        assert service.navigation["corrections"] == 1
        assert any(e["kind"] == "replanned" for e in service.events)
    run(scenario())


def test_ros_bridge_enforces_full_scan_and_unrotated_grid():
    identity = NS(rotation=NS(x=0,y=0,z=0,w=1), translation=NS(x=.1,y=0,z=0))
    scan = NS(ranges=[2.] * 360, angle_min=-math.pi, angle_increment=math.pi/180, range_min=.1, range_max=10.)
    points, coverage = scan_points(scan, identity)
    assert len(points) == 360 and coverage > 6
    assert points[180] == pytest.approx((2.1,0))
    scan.angle_increment /= 2
    with pytest.raises(ValueError, match="360"):
        scan_points(scan, identity)
    message = NS(header=NS(frame_id="map"), info=NS(width=4,height=4,resolution=.1,
        origin=NS(orientation=NS(x=0,y=0,z=0,w=1),position=NS(x=0,y=0))), data=[0]*16)
    assert occupancy_grid(message)[0]["width"] == 4
    message.info.origin.orientation.z = .2
    with pytest.raises(ValueError, match="rotated"):
        occupancy_grid(message)


@pytest.fixture
def client(tmp_path):
    app = create_app(Settings(data_dir=tmp_path, token="test-token-00000000000000000"))
    with TestClient(app) as client:
        yield client


def test_http_session_origin_intent_validation_and_websocket(client):
    assert client.get("/health").json()["status"] == "ok"
    assert client.get("/api/state").status_code == 401
    assert client.get("/api/session", headers={"Origin":"https://evil.example"}).status_code == 403
    assert client.get("/api/session").status_code == 200
    assert client.post("/api/connect", json={"mode":"demo"}).status_code == 403
    client.headers["X-G1-Intent"] = "operator"
    assert client.post("/api/connect", json={"mode":"demo"}).status_code == 200
    assert client.get("/api/body").json()["dof"] == 23
    assert client.post("/api/command", json={"capability":"raw_rpc"}).status_code == 422
    assert client.post("/api/connect", json={"mode":"robot", "interface":";touch /tmp/bad"}).status_code == 422
    with client.websocket_connect("/api/live") as socket:
        state = socket.receive_json()
        assert state["mode"] == "demo" and len(state["telemetry"]["joints"]) == 23
    assert client.post("/api/control/acquire", json={"owner":OWNER,"floor_verified":True}).status_code == 200
    result = client.post("/api/command", json=command("wave").model_dump())
    assert result.status_code == 200 and result.json()["status"] == "accepted"
    assert client.post("/api/command", content=b"x" * 900001).status_code == 413
    assert client.get("/api/state", headers={"Host":"evil.example"}).status_code == 400


def test_remote_requires_explicit_token(tmp_path):
    app = create_app(Settings(data_dir=tmp_path, token="test-token-00000000000000000"))
    with TestClient(app, client=("192.0.2.2",2345)) as remote:
        assert remote.get("/api/session").status_code == 401
        result = remote.post("/api/session", json={"token":"test-token-00000000000000000"}, headers={"X-G1-Intent":"operator"})
        assert result.status_code == 200
        assert "HttpOnly" in result.headers["set-cookie"] and "SameSite=strict" in result.headers["set-cookie"]
        assert remote.get("/api/state").status_code == 200


def test_all_capabilities_reach_official_boundary_and_preserve_rejection():
    calls = []
    class Client:
        def __getattr__(self, name):
            def rpc(*args):
                calls.append((name, args))
                return (3102, "rejected")
            return rpc
    adapter = object.__new__(UnitreeAdapter)
    adapter.read_only = adapter.verify_only = adapter.accessory_test = adapter.damping_test = False
    adapter.rpc_lock = threading.Lock()
    adapter.loco = adapter.arm = adapter.audio = Client()
    def read_mode():
        calls.append(('CachedGetFsmId', ()))
        return {key: dict(current=False, code=3102) for key in ('fsm_id', 'fsm_mode')}
    adapter.mode_reader = NS(snapshot=read_mode, read=lambda name: (calls.append((name, ())) or (3102, None)))
    examples = {"none":{}, "velocity":{"vx":.2,"vy":0,"omega":0}, "text":{"text":"test"},
                "rgb":{"r":1,"g":2,"b":3}, "pcm":{"pcm_base64":base64.b64encode(b"\0\0").decode()},
                "height":{"value":.5}}
    for cap in CATALOG:
        params = examples.get(cap["schema"], {"value":17 if cap["schema"] == "action" else 0})
        result = adapter.execute(cap["id"], validate_params(cap["id"], params))
        assert result == (3102, None if cap['id'] in READ_OPERATIONS else "rejected"), cap["id"]
    assert len(calls) == len(CATALOG)
    assert ("SetVelocity",(.2,0.,0.,.25)) in calls
    assert ("SetVelocity",(0.,0.,0.,.25)) in calls
    assert ("SetFsmId",(500,)) in calls
    assert ("SetFsmId",(4,)) in calls and ("SetFsmId",(2,)) in calls
    assert ("SetTaskId",(0,)) in calls
    assert ("SetStandHeight",(4294967295,)) in calls
    assert any(name == "_Call" for name,_ in calls)  # Audio stop must retain actual ACK.


def test_camera_ingress_is_real_only_ordered_and_source_bound(service):
    async def scenario():
        adapter = await connected(service)
        frame = CameraFrame(source="orin-camera",sequence=1,captured_at=time.time(),
                            jpeg_base64=base64.b64encode(b"\xff\xd8test\xff\xd9").decode())
        with pytest.raises(ConsoleError) as demo:
            await service.camera(frame)
        assert demo.value.code == "ROBOT_SOURCE_REQUIRED"
        adapter.mode = "robot"  # Injected adapter, no physical robot.
        await service.camera(frame)
        assert service.world.camera_sequence == 1 and service.world.camera_age() < .1
        with pytest.raises(ConsoleError) as replay:
            await service.camera(frame)
        assert replay.value.code == "STALE_CAMERA"
        with pytest.raises(ConsoleError) as source:
            await service.camera(frame.model_copy(update={"source":"other", "sequence":2}))
        assert source.value.code == "SOURCE_CONFLICT"
        with pytest.raises(ConsoleError) as stale:
            await service.camera(frame.model_copy(update={"sequence":3,"captured_at":time.time()-2}))
        assert stale.value.code == "STALE_CAMERA"
        service.world.camera_captured_at = time.time() - 1.1
        assert service.world.snapshot()["camera_age_ms"] >= 1100
    run(scenario())


def test_lan_hosts_require_explicit_allowlist(monkeypatch, tmp_path):
    monkeypatch.setenv("G1_API_TOKEN", "test-token-00000000000000000")
    with pytest.raises(ValueError, match="G1_ALLOWED_HOSTS"):
        Settings(host="0.0.0.0").validate()
    with pytest.raises(ValueError, match="通配符"):
        Settings(host="0.0.0.0",allowed_hosts=("*",)).validate()
    settings = Settings(host="0.0.0.0",allowed_hosts=("orin.local",),data_dir=tmp_path)
    with TestClient(create_app(settings),base_url="http://orin.local") as client:
        assert client.get("/health").status_code == 200
        assert client.get("/health",headers={"Host":"elsewhere.local"}).status_code == 400


def test_read_only_rejects_every_write_at_service_and_sdk_boundary(tmp_path):
    """A recording final consumer proves rejected writes never reach an SDK client."""
    calls = []
    class Client:
        def __getattr__(self, name):
            def rpc(*args):
                calls.append(name)
                return (0, json.dumps({"data": [0., 0.] if args[0] == 7006 else 0 if args[0] == 7003 else .1}) if name == "_Call" else {"volume": 50} if name == "GetVolume" else 500 if name == "GetFsmId" else {"clap": 17})
            return rpc
    adapter = object.__new__(UnitreeAdapter)
    adapter.read_only = adapter.verify_only = adapter.accessory_test = adapter.damping_test = False
    adapter.read_only = True
    adapter.rpc_lock = threading.Lock()
    adapter.loco = adapter.arm = adapter.audio = Client()
    def read_mode():
        calls.append('CachedGetFsmId')
        return {key: dict(current=True, code=0, raw={'data':500}) for key in ('fsm_id', 'fsm_mode')}
    adapter.mode_reader = NS(snapshot=read_mode, read=lambda name: (calls.append(name) or (0, {'data':[0., 0.] if name == 'get_phase' else 0})))
    adapter.action_ids, adapter.action_error = set(), None
    service = ConsoleService(Settings(read_only=True, data_dir=tmp_path))
    service.adapter = adapter
    service.telemetry["received"] = time.monotonic()
    async def scenario():
        for cap in CATALOG:
            if cap["risk"] == "read":
                await service.command(command(cap["id"]))
                continue
            for invoke in (lambda: service.command(command(cap["id"])), lambda: service._rpc(cap["id"], {})):
                with pytest.raises(ConsoleError) as error:
                    await invoke()
                assert error.value.code == "READ_ONLY" and error.value.status == 403
            with pytest.raises(ConsoleError) as error:
                adapter.execute(cap["id"], {})
            assert error.value.code == "READ_ONLY"
        for invoke in (lambda: service.acquire(OWNER, True), lambda: service.heartbeat(OWNER),
                       lambda: service.release(OWNER), lambda: service.goal(goal()),
                       lambda: service.connect("demo", "eth0")):
            with pytest.raises(ConsoleError) as error:
                await invoke()
            assert error.value.code == "READ_ONLY"
        assert service.owner is None and service.snapshot()["read_only"] is True
    run(scenario())
    assert sorted(calls) == sorted(["CachedGetFsmId"] * 2 + ["get_actions", "get_volume", "get_silent", "get_balance", "get_swing_height", "get_stand_height", "get_phase", "get_services", "get_versions", "get_loco_api_version", "get_motion_api_version", "get_state_api_version"])


def test_read_only_http_and_real_adapter_lifecycle_never_send_motion(monkeypatch, tmp_path):
    calls = []
    closed_feedback = []
    class Observer(DemoAdapter):
        mode = "robot"
        def __init__(self, interface, read_only, verify_only=False):
            assert interface == "eth0" and read_only is True
            assert verify_only is False
            super().__init__()
        def execute(self, op, params):
            calls.append(op)
            return super().execute(op, params)
        def close(self):
            # Exercise the actual real adapter close implementation.
            self.stop_event = threading.Event()
            self.subscriber = NS(Close=lambda: None)
            self.safety_feedback = NS(close=lambda: closed_feedback.append(True))
            self.service_versions = NS(close=lambda: closed_feedback.append('versions'))
            self.mode_reader = NS(close=lambda: closed_feedback.append('modes'))
            self.poller = NS(join=lambda **kwargs: None)
            UnitreeAdapter.close(self)
    monkeypatch.setattr("g1_console.service.UnitreeAdapter", Observer)
    app = create_app(Settings(read_only=True, data_dir=tmp_path))
    with TestClient(app) as client:
        client.get("/api/session")
        client.headers["X-G1-Intent"] = "operator"
        assert client.post("/api/connect", json={"mode":"robot","interface":"eth0"}).status_code == 200
        assert client.get("/api/state").json()["read_only"] is True
        for path, payload in (("/api/command", command("stop").model_dump()),
                              ("/api/control/acquire", {"owner":OWNER,"floor_verified":True}),
                              ("/api/goal", goal().model_dump())):
            response = client.post(path, json=payload)
            assert response.status_code == 403 and response.json()["error"]["code"] == "READ_ONLY"
        assert client.post("/api/command", json=command("get_fsm").model_dump()).json()["data"] == 500
        with client.websocket_connect("/api/live") as socket:
            assert socket.receive_json()["read_only"] is True
        assert client.post("/api/disconnect", json={}).status_code == 200
    assert calls == ["get_fsm"]
    assert closed_feedback == [True, 'versions', 'modes']
