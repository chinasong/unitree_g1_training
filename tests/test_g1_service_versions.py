"""Advertisement bindings are evidence only, with a receive-only DDS boundary."""
import json
from types import SimpleNamespace as NS

import pytest

from g1_console.service_versions import extract_versions, ServiceVersionStore, PassiveServiceVersions, TOPIC


def test_explicit_bindings_keep_path_and_never_adjacency_or_conflicting_names():
    rows = extract_versions(json.dumps({'services': [
        {'name': 'ai_sport', 'version': '8.7.5.212'},
        {'name': 'ai_sport'}, {'version': '9.0.0.0'},
        {'name': 'ai_sport', 'service': 'other', 'version': '8.6.0.0'},
        {'name': 'ai_sport', 'version': '8.6.0.0', 'serviceVersion': '8.7.0.0'},
    ], 'modules': {'motion_switcher': {'version': '1.0.0.1'},
        'invalid': {'name': None, 'version': '1.0.0.2'}},
        'message': 'ai_sport 99.0.0.0'}))
    assert rows == [dict(name='ai_sport', version='8.7.5.212', path='$.services[0]', binding='object_fields'),
                    dict(name='motion_switcher', version='1.0.0.1', path='$.modules.motion_switcher', binding='map_key')]


@pytest.mark.parametrize('text', [
    '{', '{"name":"a","name":"b","version":"1.2"}',
    '{"v":NaN}', '"' + 'x'*262144 + '"', '['*14+'0'+']'*14,
    json.dumps([0]*2048), json.dumps([{'name':'a','version':'1.2'}]*129),
])
def test_invalid_or_unbounded_frames_never_become_evidence(text):
    store = ServiceVersionStore()
    store.record('{"name":"ai_sport","version":"8.7.5.212"}')
    assert len(store.snapshot()['rows']) == 1  # Reachable positive control.
    store.record(text)
    snap = store.snapshot()
    assert snap['status'] == 'invalid_frame' and snap['rows'] == [] and snap['sha256'] is None
    assert snap['frames'] == 2 and snap['received_at'] is not None


def test_store_keeps_only_explicit_evidence_never_payload_or_firmware_authority():
    store = ServiceVersionStore()
    store.record('{"name":"ai_sport","version":"8.7.5.212","password":"PRIVATE-FIXTURE"}')
    snap = store.snapshot()
    assert snap['rows'][0]['version'] == '8.7.5.212' and len(snap['sha256']) == 64
    assert 'PRIVATE-FIXTURE' not in json.dumps(snap)
    assert snap['device_motion_firmware'] is None
    for key in ('current', 'execution_allowed', 'physical_publisher_verified', 'running_build_verified'):
        assert snap[key] is False
    snap['rows'].clear()
    assert len(store.snapshot()['rows']) == 1
    store.record('{"text":"ai_sport 8.7.5.212"}')
    assert store.snapshot()['rows'] == [] and store.snapshot()['status'] == 'no_matching_fields'


def test_subscriber_only_creates_reader_and_close_withdraws_queued_callbacks(monkeypatch):
    from unitree_sdk2py.core.channel import ChannelFactory
    calls = []
    class ReceiveOnlyChannel:
        def SetReader(self, qos, handler, queue_len):
            calls.append(('reader', queue_len))
            self.handler = handler
        def CloseReader(self): calls.append(('close',))
        # Deliberately no Write/SetWriter methods: any sending path fails.
    channel = ReceiveOnlyChannel()
    def create(self, topic, typ):
        assert topic == TOPIC
        calls.append(('topic', topic))
        return channel
    monkeypatch.setattr(ChannelFactory, 'CreateChannel', create)
    listener = PassiveServiceVersions()
    assert listener.snapshot()['listening']
    channel.handler(NS(data='{"name":"ai_sport","version":"8.7.5.212"}'))
    assert listener.snapshot()['rows'][0]['name'] == 'ai_sport'
    listener.close()
    channel.handler(NS(data='{"name":"ai_sport","version":"9.0.0.0"}'))
    snap = listener.snapshot()
    assert not snap['listening'] and snap['rows'] == [] and snap['frames'] == 1
    assert calls == [('topic', TOPIC), ('reader', 4), ('close',)]
