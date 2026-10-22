"""Read retained Wi-Fi kernel logs without changing logging or driver settings.

Each file is counted separately: syslog, kern.log and dmesg may duplicate events.
Only driver diagnostics are retained, never general syslog lines or credentials.
Run on Orin with its standard Python; no vendor binary or RPC is executed.
"""
from collections import Counter
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import re

PATTERNS = {
    'bb_offload': r'\[BB\].*IO offload fail',
    'rf_warning': r'\[WARNING\]\[RF\]',
    'rf_write': r'halrf_wrf return fail',
    'tx_power_control': r'Error Enable Power Control',
    'h2c_class': r'Error H2C CLASS=',
    'mac_original_status': r'rtw_hal_mac_add_cmd_ofld\(\): fault, status\s*=',
    'mac_busy': r'mac_(?:add_)?cmd_ofld(?:_v1)?: IO offload is busy',
    'mac_offload_detail': r'(?:chk_cmd_ofld_(?:reg|pkt)|proc_cmd_ofld|ofld_incompatible_full_cmd):|\[ERR\]\[CMD_OFLD',
    'mac_platform_tx': r'\[ERR\]platform tx',
    'wifi_firmware_error': r'(?:8852|RTW|rtw|MAC|BB|RF).*?(?:firmware.*(?:fail|error)|FW.*(?:assert|exception))',
    'usb_fault': r'usb\s+1-2(?:\.4)?(?::|\s).*?(?:disconnect|reset|error|over-current)',
}
REGEX = {k: re.compile(v) for k, v in PATTERNS.items()}
DETAIL = {'mac_original_status', 'mac_busy', 'mac_offload_detail', 'mac_platform_tx',
          'wifi_firmware_error', 'usb_fault'}
MAX_FILE = 256 * 1024 * 1024
MAX_TOTAL = 512 * 1024 * 1024
MAX_LINE = 512 * 1024


def scan_file(path, limit=MAX_FILE):
    """Stream a bounded uncompressed prefix and report any incomplete coverage."""
    path = Path(path)
    before = path.stat()
    result = dict(path=str(path), stored_bytes=before.st_size, read_bytes=0,
                  lines=0, oversized_lines=0, complete=False, counts={}, boots=[], details=[])
    digest = hashlib.sha256()
    total, counts, boot, pending = 0, Counter(), None, b''
    overflow = False

    def line(raw):
        nonlocal boot
        result['lines'] += 1
        text = raw.replace(b'\0', b'').decode('utf-8', errors='replace')
        if 'Booting Linux on physical CPU' in text:
            boot = dict(start_line=result['lines'], marker=text[:300], counts={}, first={}, last={})
            result['boots'].append(boot)
        matched = [key for key, regex in REGEX.items() if regex.search(text)]
        for key in matched:
            counts[key] += 1
            if boot is not None:
                boot['counts'][key] = boot['counts'].get(key, 0) + 1
                boot['first'].setdefault(key, text[:500])
                boot['last'][key] = text[:500]
        if DETAIL.intersection(matched) and len(result['details']) < 200:
            result['details'].append(dict(line=result['lines'], categories=matched, text=text[:500]))

    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rb') as stream:
        while total < limit:
            chunk = stream.read(min(65536, limit-total))
            if not chunk:
                result['complete'] = True
                break
            total += len(chunk)
            digest.update(chunk)
            parts = (pending+chunk).split(b'\n')
            for raw in parts[:-1]:
                if overflow or len(raw) > MAX_LINE:
                    result['oversized_lines'] += 1
                    overflow = False
                else:
                    line(raw)
            pending = parts[-1]
            if len(pending) > MAX_LINE:
                overflow = True
                pending = b''
        if total == limit:
            result['complete'] = not bool(stream.read(1))
        if pending and result['complete'] and not overflow:
            line(pending)
        elif overflow:
            result['oversized_lines'] += 1
    after = path.stat()
    result.update(read_bytes=total, counts={k: counts[k] for k in PATTERNS},
                  read_sha256=digest.hexdigest(),
                  unchanged_during_read=(before.st_size, before.st_mtime_ns) ==
                                        (after.st_size, after.st_mtime_ns))
    result['complete'] = result['complete'] and not result['oversized_lines']
    return result


def inspect():
    paths = sorted({p for pattern in ('kern.log*', 'syslog*', 'dmesg*')
                    for p in Path('/var/log').glob(pattern) if p.is_file() and not p.is_symlink()})
    out = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
               boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
               patterns=PATTERNS, files=[], errors=[], command_writes=0, rpc_calls=0,
               raw_mac_error_code=None, exact_trigger=None,
               limitation='Per-file counts overlap across logs; zero matches do not rule out unlogged faults.')
    remaining = MAX_TOTAL
    for path in paths:
        if remaining <= 0:
            out['errors'].append(dict(path=str(path), error='total byte bound'))
            continue
        try:
            item = scan_file(path, min(MAX_FILE, remaining))
            out['files'].append(item)
            remaining -= item['read_bytes']
        except (OSError, EOFError) as exc:
            out['errors'].append(dict(path=str(path), error=type(exc).__name__))
    out['completed_at_utc'] = datetime.now(timezone.utc).isoformat()
    return out


if __name__ == '__main__':
    print(json.dumps(inspect(), ensure_ascii=False))
