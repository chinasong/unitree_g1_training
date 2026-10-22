"""Official videohub API 1001 with a polled, single-request response reader.

Uses the pinned Unitree IDL and ChannelFactory. No SDK listener queue, retained
futures, lease client, or configurable API ID. This is only an image reader.
"""
import time


class ReadOnlyVideoClient:
    def __init__(self, timeout=.7):
        from unitree_sdk2py.core.channel import ChannelFactory
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_, Response_
        factory = ChannelFactory()
        self.request = factory.CreateChannel("rt/api/videohub/request", Request_)
        self.response = factory.CreateChannel("rt/api/videohub/response", Response_)
        self.timeout = timeout
        self.closed = False
        try:
            self.response.SetReader()  # No on_data_available callback or SDK BQueue.
            self.request.SetWriter()
        except Exception:
            self.close()
            raise

    def GetImageSample(self):
        if self.closed:
            return 3102, None
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
            Request_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_)
        identity = RequestIdentity_(time.monotonic_ns(), 1001)
        request = Request_(RequestHeader_(identity, RequestLease_(0), RequestPolicy_(0, False)), "", [])
        deadline = time.monotonic() + self.timeout
        if not self.request.Write(request, self.timeout):
            return 3102, None
        while (remaining := deadline - time.monotonic()) > 0:
            response = self.response.Read(remaining)
            if response is None:
                break
            header = getattr(response, "header", None)
            if header is None or header.identity.id != identity.id:
                continue  # Other clients and late replies cannot satisfy this request.
            if header.identity.api_id != 1001:
                return 3105, None
            return header.status.code, response.binary
        return 3104, None

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.response.CloseReader()
        self.request.CloseWriter()
