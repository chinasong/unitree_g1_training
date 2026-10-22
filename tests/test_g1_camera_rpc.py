"""Exercise reply correlation, a bounded timeout and the exact read-only wire call."""
from types import SimpleNamespace as NS
import time

from g1_console.camera_rpc import ReadOnlyVideoClient


def client(monkeypatch, replies, write_ok=True):
    from unitree_sdk2py.core.channel import ChannelFactory
    channels = {}
    class Channel:
        def SetReader(self):
            self.reader = True
        def SetWriter(self):
            self.writer = True
        def Write(self, request, timeout):
            self.sent = request
            return write_ok
        def Read(self, timeout):
            assert 0 < timeout <= .03
            return replies(channels['rt/api/videohub/request'].sent)
        def CloseReader(self):
            self.closed = True
        def CloseWriter(self):
            self.closed = True
    def create(self, name, typ):
        assert name in ('rt/api/videohub/request', 'rt/api/videohub/response')
        channels[name] = Channel()
        return channels[name]
    monkeypatch.setattr(ChannelFactory, 'CreateChannel', create)
    return ReadOnlyVideoClient(timeout=.03), channels


def reply(request, offset=0, api=1001, code=0):
    return NS(header=NS(identity=NS(id=request.header.identity.id+offset, api_id=api), status=NS(code=code)), binary=[255, 216])


def test_polled_camera_ignores_other_clients_and_late_replies(monkeypatch):
    count = 0
    def responses(request):
        nonlocal count
        count += 1
        return reply(request, offset=-1 if count == 1 else 0)
    c, channels = client(monkeypatch, responses)
    assert c.GetImageSample() == (0, [255, 216]) and count == 2
    r = channels['rt/api/videohub/request'].sent
    assert r.header.identity.api_id == 1001 and r.header.lease.id == 0
    assert r.header.policy.priority == 0 and not r.header.policy.noreply
    assert r.parameter == '' and r.binary == []
    c.close(); c.close()
    assert all(ch.closed for ch in channels.values())
    assert c.GetImageSample() == (3102, None)


def test_polled_camera_deadline_even_with_unrelated_traffic(monkeypatch):
    c, _ = client(monkeypatch, lambda r: reply(r, offset=-1))
    start = time.monotonic()
    assert c.GetImageSample() == (3104, None)
    assert time.monotonic()-start < .2
    c.close()


def test_polled_camera_no_response_send_failure_and_api_validation(monkeypatch):
    c, _ = client(monkeypatch, lambda r: None)
    assert c.GetImageSample() == (3104, None)
    c.close()
    c, _ = client(monkeypatch, lambda r: reply(r), write_ok=False)
    assert c.GetImageSample() == (3102, None)
    c.close()
    c, _ = client(monkeypatch, lambda r: reply(r, api=1002))
    assert c.GetImageSample() == (3105, None)
    c.close()
