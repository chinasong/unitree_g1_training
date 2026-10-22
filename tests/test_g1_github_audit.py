"""Keep public discussion evidence distinct from validated device configuration."""
import json
import shutil

import pytest

from g1_console.body import KNOWLEDGE
from scripts import build_g1_motor_knowledge as builder


def test_public_review_is_persisted_without_promoting_other_robot_parameters():
    assert builder.build() == KNOWLEDGE
    review = KNOWLEDGE['github_review']
    assert review['current'] is False
    assert review['motor_configuration_read_supported'] is review['calibration_read_supported'] is None
    assert review['confirmed_read_endpoints'] == []
    actual = [v for motor in KNOWLEDGE['motors'] for v in motor['actual_config'].values()]
    assert len(actual) == 253 and all(v is None for v in actual)
    assert any(f['collaborator_reply'] for f in review['findings'])
    assert all(not f['confirmed_g1_read_protocol'] for f in review['findings'])


@pytest.mark.parametrize('change', ['snapshot', 'coverage', 'read_support', 'endpoint', 'protocol'])
def test_source_drift_or_unreviewed_protocol_promotion_is_rejected(tmp_path, monkeypatch, change):
    asset = 'g1_console/assets/g1_github_parameter_audit.json'
    evidence = json.loads((builder.ROOT/asset).read_text())
    source = evidence['snapshot_source']['path']
    for path in [asset, source]:
        destination = tmp_path/path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(builder.ROOT/path, destination)
    if change == 'snapshot':
        (tmp_path/source).write_text('{}')
    elif change == 'coverage':
        evidence['search_result_count'] += 1
    elif change == 'read_support':
        evidence['motor_configuration_read_supported'] = True
    elif change == 'endpoint':
        evidence['confirmed_read_endpoints'] = ['unverified_config_getter']
    else:
        evidence['findings'][0]['confirmed_g1_read_protocol'] = True
    (tmp_path/asset).write_text(json.dumps(evidence))
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    with pytest.raises(AssertionError):
        builder.github_audit()
