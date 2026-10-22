#!/usr/bin/env python3
"""Save one authenticated read-only camera frame for local Codex visual inspection."""
import argparse
import hashlib
import http.cookiejar
import json
import math
from pathlib import Path
import time
import urllib.parse
import urllib.request


def snapshot(base, output, *, manual_remote=False):
    parsed = urllib.parse.urlsplit(base)
    if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost", "::1") or parsed.username or parsed.password or parsed.path not in ("", "/") or parsed.query or parsed.fragment:
        raise ValueError("仅接受本机 SSH 转发入口，例如 http://127.0.0.1:18765")
    base = base.rstrip("/")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    # Session is local only; never read, print or persist its cookie/token.
    with opener.open(base + "/api/session", timeout=5) as response:
        response.read(4096)
    with opener.open(base + "/api/state", timeout=5) as response:
        state = json.load(response)
    allowed = state.get("read_only") is True or (manual_remote and state.get("read_only") is False
              and state.get("manual_remote", {}).get("enabled") is True)
    if not allowed or state.get("mode") != "robot":
        raise ValueError("需要真实机器人只读服务，或用 --manual-remote 明确允许软件遥控服务；没有执行任何控制操作")
    started = time.monotonic()
    with opener.open(base + "/api/camera.jpg", timeout=3) as response:
        metadata = json.loads(response.headers["X-G1-Camera-Metadata"])
        raw = response.read(500001)
        age = metadata["age_ms"] + (time.monotonic() - started) * 1000
        if not math.isfinite(age) or not 0 <= age < 1000 or metadata.get("status") != "live":
            raise ValueError("拒绝保存过期相机帧")
        if response.headers.get_content_type() != "image/jpeg" or not raw.startswith(b"\xff\xd8") or not raw.endswith(b"\xff\xd9") or len(raw) > 500000:
            raise ValueError("相机响应不是有效的有界 JPEG")
    output = Path(output).expanduser().resolve()
    metadata.update(mac_received_at=time.time(), mac_age_upper_bound_ms=round(age),
                    sha256=hashlib.sha256(raw).hexdigest(), bytes=len(raw), file=str(output))
    output.write_bytes(raw)
    output.with_suffix(".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    return metadata


def main():
    parser = argparse.ArgumentParser(description="读取 G1 官方相机的一帧，供本机 Codex 查看；不执行动作")
    parser.add_argument("--url", default="http://127.0.0.1:18765")
    parser.add_argument("--output", type=Path, default=Path("/tmp/g1-codex-camera.jpg"))
    parser.add_argument("--manual-remote", action="store_true", help="明确允许从软件遥控服务读取图像；仍然仅使用 GET")
    args = parser.parse_args()
    try:
        result = snapshot(args.url, args.output, manual_remote=args.manual_remote)
    except Exception as exc:
        raise SystemExit(f"读取失败：{type(exc).__name__}: {exc}") from None
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
