"""Bounded package parsing and receipt binding without invoking vendor software."""
from copy import deepcopy
import io
import json
import tarfile
import zipfile

import pytest
from scripts import analyze_g1_package_cache as analyzer
from scripts import audit_g1_package_cache as audit
from g1_console.body import KNOWLEDGE


def test_review_is_bound_to_current_master_and_never_supplies_motor_values():
    review=analyzer.build()
    assert review['files_scanned']==51 and review['binaries_scanned']==5
    assert len(review['packages'])==4 and review['motor_configuration_values_obtained']==0
    assert review['motor_configuration_present'] is None and not review['payload_decoded']
    assert review==json.loads((analyzer.BASE/'package_cache_review_20261021.json').read_text())
    assert len([p for p in review['packages'] if p['format']=='UTPK opaque'])==3
    assert not any(p['implementation_match_confirmed'] for p in review['packages'])
    assert KNOWLEDGE['service_audit']['package_cache_review']['conclusion']==review['conclusion']


def test_mismatched_master_fingerprint_cannot_enter_foundation(tmp_path,monkeypatch):
    raw=json.loads((analyzer.BASE/'package_cache_inventory_20261021.json').read_text())
    raw['identity_before'][0]['sha256']='0'*64
    raw['identity_after']=deepcopy(raw['identity_before'])
    (tmp_path/'package_cache_inventory_20261021.json').write_text(json.dumps(raw))
    (tmp_path/'fmx_loader_inventory_20261021.json').write_bytes((analyzer.BASE/'fmx_loader_inventory_20261021.json').read_bytes())
    monkeypatch.setattr(analyzer,'BASE',tmp_path)
    with pytest.raises(AssertionError):analyzer.build()


def test_zip_reports_token_counts_only_and_stops_at_uncompressed_bound(monkeypatch):
    buffer=io.BytesIO()
    with zipfile.ZipFile(buffer,'w',compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr('format.txt','FMX motor_calibration secret=not_exported')
        z.writestr('large.txt','x'*100)
    monkeypatch.setattr(audit,'MAX_BYTES',80)
    rows,truncated=audit.archive_members(buffer.getvalue())
    assert truncated and len(rows)==1
    assert rows[0]['tokens']=={'fmx_format':1,'motor_calibration':1}
    assert 'not_exported' not in json.dumps(rows)


def test_tar_does_not_follow_links_or_extract_paths(tmp_path):
    target=tmp_path/'private.txt';target.write_text('FMX')
    buffer=io.BytesIO()
    with tarfile.open(fileobj=buffer,mode='w') as tar:
        link=tarfile.TarInfo('../../link');link.type=tarfile.SYMTYPE;link.linkname=str(target)
        tar.addfile(link)
        normal=tarfile.TarInfo('format.txt');normal.size=3;tar.addfile(normal,io.BytesIO(b'FMX'))
    rows,truncated=audit.archive_members(buffer.getvalue())
    assert not truncated and rows[0]['tokens']=={} and rows[1]['tokens']=={'fmx_format':1}
    assert not (tmp_path/'format.txt').exists()
