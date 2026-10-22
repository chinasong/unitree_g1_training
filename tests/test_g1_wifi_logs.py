"""Exercise nonzero diagnostics before trusting a zero in retained robot logs."""
import gzip

from scripts.audit_g1_wifi_logs import MAX_LINE, scan_file


def test_raw_mac_code_is_distinct_from_generic_bb_and_rf_errors(tmp_path):
    path = tmp_path/'kern.log'
    path.write_bytes(b'Booting Linux on physical CPU\n'
                     b'[BB] IO offload fail: 1\n[WARNING][RF] halrf_wrf return fail\n'
                     b'rtw_hal_mac_add_cmd_ofld(): fault, status = 57.\n'
                     b'mac_add_cmd_ofld: IO offload is busy\n')
    result = scan_file(path)
    assert result['complete'] and result['unchanged_during_read']
    assert result['counts']['mac_original_status'] == result['counts']['mac_busy'] == 1
    assert result['counts']['rf_warning'] == result['counts']['bb_offload'] == 1
    assert len(result['details']) == 2 and '57' in result['details'][0]['text']
    assert result['boots'][0]['counts'] == {k: 1 for k in ('bb_offload','rf_warning','rf_write','mac_original_status','mac_busy')}


def test_gzip_nul_chunk_boundaries_and_final_line_are_scanned(tmp_path):
    path = tmp_path/'dmesg.1.gz'
    content = b'x'*65530+b'\n[BB]\0 IO offload fail: 1\n[ERR]platform tx'
    with gzip.open(path, 'wb') as stream:
        stream.write(content)
    result = scan_file(path)
    assert result['complete'] and result['read_bytes'] == len(content) and result['lines'] == 3
    assert result['counts']['bb_offload'] == result['counts']['mac_platform_tx'] == 1
    assert result['counts']['mac_original_status'] == 0


def test_byte_bound_is_explicit_and_unterminated_prefix_is_not_counted(tmp_path):
    path = tmp_path/'syslog'
    path.write_bytes(b'ordinary\n[BB] IO offload fail: 1\n')
    result = scan_file(path, limit=12)
    assert not result['complete'] and result['read_bytes'] == 12 and result['lines'] == 1
    assert result['counts']['bb_offload'] == 0
    assert scan_file(path, limit=path.stat().st_size)['complete']


def test_oversized_line_is_flagged_without_losing_following_diagnostics(tmp_path):
    path = tmp_path/'syslog'
    path.write_bytes(b'x'*(MAX_LINE+1)+b'\nrtw_hal_mac_add_cmd_ofld(): fault, status = 57.\n')
    result = scan_file(path)
    assert not result['complete'] and result['oversized_lines'] == 1
    assert result['counts']['mac_original_status'] == 1


def test_boot_counts_are_separate_from_duplicate_file_totals(tmp_path):
    path = tmp_path/'kern.log'
    path.write_text('[BB] IO offload fail: 1\nBooting Linux on physical CPU 1\n'
                    '[BB] IO offload fail: 1\nBooting Linux on physical CPU 2\nhealthy\n')
    result = scan_file(path)
    assert result['counts']['bb_offload'] == 2
    assert result['boots'][0]['counts']['bb_offload'] == 1
    assert result['boots'][1]['counts'] == {}
