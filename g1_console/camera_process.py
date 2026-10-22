"""Keep image DDS reception separate from high-rate body DDS callbacks.

Spawn a fresh interpreter: forking a process that already owns DDS threads is
unsafe. Only the existing read-only camera sampler runs in this child.
"""
import multiprocessing
import threading
import time


def _capture(send, stop, width, fps, interface):
    camera = None
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from .camera import OfficialCamera
        ChannelFactoryInitialize(0, interface)
        camera = OfficialCamera(width, fps)
        previous = None
        while not stop.is_set():
            frame, status, error = camera.latest()
            stats = camera.diagnostics()
            key = (stats['total_attempts'], status, error)
            if key != previous:
                send.send((frame, status, error, stats))
                previous = key
            stop.wait(.02)
    except (BrokenPipeError, EOFError, OSError):
        pass  # Parent disconnected; never reconnect or acquire control.
    except Exception as exc:
        try:
            send.send((None, 'unavailable', type(exc).__name__, None))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if camera:
            camera.close()
        send.close()


class IsolatedCamera:
    def __init__(self, width=960, fps=5, interface='eth0', *, worker=_capture):
        context = multiprocessing.get_context('spawn')
        self.receive, send = context.Pipe(duplex=False)
        self.stop = context.Event()
        self.lock = threading.Lock()
        self.closed = False
        self.frame, self.status, self.error, self.stats = None, 'starting', None, None
        self.received = 0
        self.process = context.Process(target=worker, args=(send, self.stop, width, fps, interface),
                                       name='g1-camera-readonly', daemon=True)
        try:
            self.process.start()
        except Exception:
            self.receive.close()
            send.close()
            raise
        send.close()
        self.thread = threading.Thread(target=self._receive, daemon=True, name='g1-camera-ipc')
        self.thread.start()

    def _receive(self):
        try:
            while True:
                if self.receive.poll(.1):
                    frame, status, error, stats = self.receive.recv()
                    with self.lock:
                        self.frame, self.status, self.error, self.stats = frame, status, error, stats
                        self.received = time.monotonic()
                elif not self.process.is_alive():
                    break
        except (EOFError, OSError):
            pass
        finally:
            with self.lock:
                self.frame, self.stats = None, None
                self.status, self.error = ('stopped', None) if self.closed else ('unavailable', 'CAMERA_WORKER_EXITED')

    def latest(self):
        with self.lock:
            # IPC never gives an old frame a new request/exposure timestamp.
            if self.received and time.monotonic()-self.received > 2:
                return None, 'unavailable', 'CAMERA_WORKER_STALLED'
            return self.frame, self.status, self.error

    def diagnostics(self):
        with self.lock:
            if self.stats is None or time.monotonic()-self.received > 2:
                return None
            return dict(self.stats, transport='official_api_1001_isolated_polled')

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        # Keep draining IPC while the sampler finishes its bounded request.
        self.process.join(timeout=3)
        if self.process.is_alive():
            self.process.terminate()  # Only this child, never an official service.
            self.process.join(timeout=1)
        self.thread.join(timeout=1)
        self.receive.close()
