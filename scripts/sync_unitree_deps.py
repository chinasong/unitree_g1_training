#!/usr/bin/env python3
"""Restore official Unitree sources at the commits in unitree-deps.lock.json."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify without changing files or using the network")
    args = parser.parse_args()
    env = os.environ.copy()
    clt = Path("/Library/Developer/CommandLineTools")
    git = shutil.which("git")
    if sys.platform == "darwin" and (clt / "usr/bin/git").is_file():
        git = str(clt / "usr/bin/git")
        env["DEVELOPER_DIR"] = str(clt)
    if not git:
        raise RuntimeError("Git is required")

    def run(*command, cwd=ROOT):
        return subprocess.run(
            [git, *command], cwd=cwd, env=env, check=True,
            text=True, stdout=subprocess.PIPE,
        ).stdout.strip()

    lock = json.loads((ROOT / "unitree-deps.lock.json").read_text())
    dependencies = lock["dependencies"]
    # Check all existing directories before updating any of them.
    for name, spec in dependencies.items():
        path = ROOT / "externals" / name
        if not path.exists():
            if args.check:
                raise RuntimeError(f"Missing {path}; run this script without --check")
            continue
        if not (path / ".git").exists():
            raise RuntimeError(f"Refusing to overwrite non-Git directory: {path}")
        origin = run("remote", "get-url", "origin", cwd=path)
        if origin.rstrip("/") not in (spec["url"], spec["url"][:-4]):
            raise RuntimeError(f"Unexpected origin for {name}: {origin}")
        if run("status", "--porcelain", "--untracked-files=all", cwd=path):
            raise RuntimeError(f"Refusing to update modified checkout: {path}")

    for name, spec in dependencies.items():
        path = ROOT / "externals" / name
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            run("clone", "--no-checkout", spec["url"], str(path))
        current = run("rev-parse", "HEAD", cwd=path)
        if current != spec["commit"] or not args.check:
            if args.check:
                raise RuntimeError(f"{name}: expected {spec['commit']}, found {current}")
            # Fetch only when the pinned commit is unavailable locally.
            result = subprocess.run(
                [git, "cat-file", "-e", spec["commit"] + "^{commit}"],
                cwd=path, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            if result.returncode:
                run("fetch", "origin", spec["commit"], cwd=path)
            run("checkout", "--detach", spec["commit"], cwd=path)
        actual = run("rev-parse", "HEAD", cwd=path)
        if actual != spec["commit"]:
            raise RuntimeError(f"{name}: expected {spec['commit']}, found {actual}")
        print(f"OK {name} {actual}")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
