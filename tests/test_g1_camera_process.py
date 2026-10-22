"""Real spawned processes, no SDK or robot: IPC timestamps, cleanup and failure."""
import time

from g1_console.camera_process import IsolatedCamera


def fake_capture(send, stop, width, fps, interface):
    assert (width, fps, interface) == (960, 5, 'eth0')
    stamp = time.monotonic() - .3
    frame = (b'\xff\xd8frame', stamp, 1234., {'source': 'test', 'sequence': 9})
    send.send((frame, 'receiving', None, dict(total_attempts=1, successes=1)))
    stop.wait(5)
    send.close()


def failed_capture(send, stop, width, fps, interface):
    send.close()


def test_spawned_camera_preserves_original_timestamp_and_shuts_down():
    camera = IsolatedCamera(worker=fake_capture)
    try:
        deadline = time.monotonic()+5
        while camera.latest()[0] is None and time.monotonic() < deadline:
            time.sleep(.02)
        frame, status, error = camera.latest()
        assert status == 'receiving' and error is None
        assert time.monotonic()-frame[1] >= .3 and frame[2] == 1234.
        assert camera.diagnostics()['transport'] == 'official_api_1001_isolated_polled'
        with camera.lock:
            camera.received -= 3
        assert camera.latest() == (None, 'unavailable', 'CAMERA_WORKER_STALLED')
        assert camera.diagnostics() is None
    finally:
        camera.close(); camera.close()
    assert not camera.process.is_alive() and not camera.thread.is_alive()
    assert camera.latest()[0] is None


def test_spawned_camera_exit_does_not_leave_a_live_frame():
    camera = IsolatedCamera(worker=failed_capture)
    try:
        camera.thread.join(timeout=5)
        assert camera.latest() == (None, 'unavailable', 'CAMERA_WORKER_EXITED')
        assert camera.diagnostics() is None
    finally:
        camera.close()
