#!/usr/bin/env python3
"""Package built console and pinned sources; never copy a Mac virtual environment."""
from pathlib import Path
import hashlib
import json
import tarfile


def main():
    root = Path(__file__).resolve().parents[1]
    directories = ("g1_console", "console/web/dist", "externals/unitree_sdk2_python",
                   "externals/unitree_rl_gym/resources/robots/g1_description/meshes")
    files = []
    for directory in directories:
        base = root / directory
        if not base.is_dir():
            raise SystemExit(f"缺少 {directory}；先恢复官方依赖并构建前端")
        files.extend(p for p in base.rglob("*") if p.is_file() and not any(
            part in (".git", "__pycache__", "build", "dist") or part.endswith(".egg-info")
            for part in p.relative_to(base).parts[:-1]))
    files.extend(root / name for name in ("requirements-console.lock.txt", "unitree-deps.lock.json"))
    manifest = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    encoded = json.dumps(manifest, sort_keys=True, indent=2).encode()
    release_id = hashlib.sha256(encoded).hexdigest()[:12]
    output = root / ".g1-deploy"
    output.mkdir(exist_ok=True)
    (output / "release-manifest.json").write_bytes(encoded)
    (output / "release-id").write_text(release_id + "\n")
    with tarfile.open(output / "console-release.tar.gz", "w:gz") as archive:
        for p in files:
            archive.add(p, arcname=str(p.relative_to(root)), recursive=False)
        archive.add(output / "release-manifest.json", arcname="release-manifest.json")
    print(f"release={release_id} files={len(files)} archive={output / 'console-release.tar.gz'}")


if __name__ == "__main__":
    main()
