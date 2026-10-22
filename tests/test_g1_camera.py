"""Camera reads only: official API, lifecycle, stale frames and authenticated HTTP."""
import asyncio
import base64
import json
import threading
import time

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient

from g1_console.adapters import DemoAdapter
from g1_console.app import create_app
from g1_console.camera import OfficialCamera
from g1_console.config import ConsoleError, Settings
from g1_console.contracts import CameraFrame
from g1_console.service import ConsoleService
from scripts.read_g1_camera import snapshot


def jpeg():
    ok, data = cv2.imencode('.jpg', np.zeros((1080, 1920, 3), dtype=np.uint8))
    assert ok
    return bytes(data)


def test_official_camera_recovers_and_only_reads_sdk(monkeypatch):
    calls, frames = [], [None, jpeg()]
    failed = threading.Event()
    class Client:
        def __init__(self, timeout):
            calls.append(('timeout', timeout))
        def close(self):
            calls.append(('close',))
        def GetImageSample(self):
            calls.append(('read',))
            if len(frames) > 1:
                frames.pop(0)
                failed.set()
                return 3102, []
            return 0, frames[0]
    monkeypatch.setattr('g1_console.camera.ReadOnlyVideoClient', Client)
    camera = OfficialCamera(width=960, fps=10)
    try:
        assert failed.wait(2)
        deadline = time.monotonic() + 3
        while camera.latest()[0] is None and time.monotonic() < deadline:
            time.sleep(.02)
        frame, status, error = camera.latest()
        assert frame is not None and status == 'receiving' and error is None
        raw, _, _, metadata = frame
        assert cv2.imdecode(np.frombuffer(raw, np.uint8), 1).shape == (540, 960, 3)
        assert metadata['source_width'] == 1920 and metadata['sensor_timestamp'] is None
    finally:
        camera.close()
    assert not camera.thread.is_alive()
    assert {c[0] for c in calls} == {'timeout', 'close', 'read'}
    assert camera.diagnostics()['total_successes'] >= 1


def test_camera_lifecycle_stale_source_and_reconnect(monkeypatch, tmp_path):
    readers = []
    class Observer(DemoAdapter):
        mode = 'robot'
        def __init__(self, interface, read_only, verify_only=False):
            assert read_only and not verify_only
            super().__init__()
        def execute(self, op, params):
            raise AssertionError('Camera lifecycle must not send a robot command')
        def close(self):
            self.closed = True
    class Reader:
        def __init__(self, *args):
            self.source = 'source-' + str(len(readers))
            self.closed, self.age = False, 0
            readers.append(self)
        def latest(self):
            meta = dict(source=self.source, sequence=1, timestamp_basis='request_started_at', sensor_timestamp=None)
            return (b'\xff\xd8test\xff\xd9', time.monotonic()-self.age, time.time()-self.age, meta), 'receiving', None
        def close(self):
            self.closed = True
    monkeypatch.setattr('g1_console.service.UnitreeAdapter', Observer)
    monkeypatch.setattr('g1_console.service.OfficialCamera', Reader)
    service = ConsoleService(Settings(data_dir=tmp_path, read_only=True, camera_backend='unitree'))
    async def scenario():
        await service.connect('robot', 'eth0')
        assert service.camera_status()['status'] == 'live'
        with pytest.raises(ConsoleError) as error:
            await service.camera(CameraFrame(source='external', sequence=2, captured_at=time.time(), jpeg_base64=base64.b64encode(jpeg()).decode()))
        assert error.value.code == 'CAMERA_SOURCE_RESERVED'
        service.world.camera_at -= 2
        await service.tick()  # Repeated sequence must not refresh the old frame.
        assert service.camera_status()['status'] == 'stale'
        await service.connect('robot', 'eth0')
        assert readers[0].closed and service.world.camera_source == 'source-1'
        await service.disconnect()
        assert readers[1].closed and service.world.camera is None
        assert service.owner is None and service.navigation['status'] == 'idle'
    asyncio.run(scenario())


def test_camera_http_auth_metadata_and_stale_fail_closed(tmp_path):
    app = create_app(Settings(data_dir=tmp_path, read_only=True))
    world = app.state.service.world
    with TestClient(app) as client:
        assert client.get('/api/camera.jpg').status_code == 401
        assert client.get('/api/camera/status').status_code == 401
        client.get('/api/session')
        assert client.get('/api/camera.jpg').status_code == 404
        world.camera = jpeg()
        world.camera_at, world.camera_captured_at = time.monotonic(), time.time()
        world.camera_sequence, world.camera_source = 7, '相机\r\nunsafe'
        world.camera_metadata = dict(width=1920, height=1080)
        response = client.get('/api/camera.jpg')
        assert response.status_code == 200 and response.content == world.camera
        metadata = json.loads(response.headers['X-G1-Camera-Metadata'])
        assert metadata['sequence'] == 7 and metadata['width'] == 1920
        assert response.headers['cache-control'] == 'no-store'
        assert '\r' not in response.headers['X-G1-Camera-Source']
        world.camera_captured_at -= 2
        assert client.get('/api/camera/status').json()['status'] == 'stale'
        assert client.get('/api/camera.jpg').status_code == 404


@pytest.mark.parametrize('url', ['https://example.com', 'http://192.0.2.28:8765', 'http://user:pass@localhost:8765', 'http://localhost:8765/private'])
def test_codex_snapshot_only_reads_local_tunnel(url, tmp_path):
    with pytest.raises(ValueError, match='本机'):
        snapshot(url, tmp_path/'camera.jpg')
    assert not (tmp_path/'camera.jpg').exists()
