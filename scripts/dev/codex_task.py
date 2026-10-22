#!/usr/bin/env python3
"""Dispatch one well-specified task to Codex in an isolated git worktree, then audit what it changed.

Claude writes the brief and reviews the result; Codex does the exploration and edits on its own quota.
Codex never commits and never reaches the robot: the sandbox is workspace-write (no network) and the
worktree has no deploy credentials. This script never merges; review and commit stay with the reviewer.
"""
import argparse
import fnmatch
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
WORKTREES = ROOT / ".worktrees"
RUNS = ROOT / ".codex-runs"
# Ignored dependency trees the offline sandbox cannot install; symlinked into each worktree (.gitignore matches the link).
LINKED = ("console/web/node_modules",)
GIT_CANDIDATES = (shutil.which("git"),
                  str(Path.home() / ".cache/codex-runtimes/codex-primary-runtime/dependencies/bin/fallback/git"))

# Safety gate and robot-facing code: Codex may propose changes in its report, never edit them.
PROTECTED = (
    "g1_console/service.py", "g1_console/adapters.py", "g1_console/capabilities.py", "g1_console/config.py",
    "g1_console/contracts.py", "g1_console/app.py", "g1_console/mode_readback.py", "g1_console/mode_sequence.py",
    "scripts/connect_orin_console.py", "scripts/orin_console_recovery.py", "scripts/package_orin_console.py",
    "*.command", "CLAUDE.md", "scripts/dev/*", ".gitignore", "pytest.ini", "requirements*", "unitree-deps.lock.json",
)

# The desktop app bundles a newer CLI than npm; newer models refuse older CLIs.
CODEX_CANDIDATES = ("/Applications/Codex.app/Contents/Resources/codex", shutil.which("codex"))

REPORT_SCHEMA = {
    "type": "object", "additionalProperties": False,
    "required": ["summary", "files_changed", "tests", "unverified", "proposals_for_protected_files"],
    "properties": {
        "summary": {"type": "string"},
        "files_changed": {"type": "array", "items": {"type": "string"}},
        "tests": {"type": "array", "items": {"type": "object", "additionalProperties": False,
                  "required": ["command", "result"],
                  "properties": {"command": {"type": "string"}, "result": {"type": "string"}}}},
        "unverified": {"type": "array", "items": {"type": "string"}},
        "proposals_for_protected_files": {"type": "array", "items": {"type": "string"}},
    },
}

PREAMBLE = """你在 unitree_g1_training 项目的独立 git worktree 中执行一个已写清规格的任务，由 Claude Code 审查后决定是否合并。

硬性规则：
1. 只修改任务单“允许修改”列出的路径；需要改其他文件时，写进报告的 proposals_for_protected_files，不要动手。
2. 以下文件永远不改，只能在报告中提出建议：{protected}
3. 不提交 git、不切分支、不删除文件以外的历史；不联网、不连接机器人或 SSH，不运行 scripts/connect_orin_console.py、部署或任何 audit_g1_* 实机脚本。
4. Python 统一使用 {python}（主工作区的虚拟环境），测试用 `{python} -m pytest -q -p no:cacheprovider <测试文件>`。
5. 未知保持未知：不要用示例值、默认值冒充实机参数；没跑过的验证写进 unverified。
6. 结束时只输出符合给定 JSON schema 的报告，files_changed 必须是实际改动的相对路径。

任务单如下：
"""


def find_git():
    for candidate in GIT_CANDIDATES:
        if candidate and subprocess.run([candidate, "--version"], capture_output=True).returncode == 0:
            return candidate
    raise SystemExit("找不到可用的 git（系统 git 可能因 Xcode license 不可用）")


def find_codex():
    for candidate in CODEX_CANDIDATES:
        if candidate and Path(candidate).is_file():
            return candidate
    raise SystemExit("找不到 codex CLI")


def git(git_bin, *args, cwd=ROOT):
    return subprocess.run([git_bin, *args], cwd=cwd, capture_output=True, text=True, check=True).stdout


def allowed_paths(brief):
    """Read the brief's `允许修改:` block: one glob per `- ` line."""
    match = re.search(r"^允许修改[:：]\s*\n((?:\s*-\s+.+\n?)+)", brief, re.M)
    if not match:
        raise SystemExit("任务单必须包含“允许修改:”段落，每行一个路径或 glob，例如 `- tests/test_x.py`")
    return [line.strip()[2:].strip().strip("`") for line in match.group(1).splitlines() if line.strip()]


def matches(path, patterns):
    return any(fnmatch.fnmatch(path, p) or path.startswith(p.rstrip("/") + "/") for p in patterns)


def changed_files(git_bin, worktree):
    out = git(git_bin, "status", "--porcelain", "-uall", cwd=worktree)
    return sorted(line[3:].split(" -> ")[-1] for line in out.splitlines() if line)


def audit(changed, allow):
    return dict(
        protected_touched=[f for f in changed if matches(f, PROTECTED)],
        outside_allowed=[f for f in changed if not matches(f, allow)],
    )


def main():
    parser = argparse.ArgumentParser(description="把一个规格清楚的任务派给 Codex（独立 worktree、无网络、不提交）并审计改动")
    parser.add_argument("brief", type=Path, help="任务单 Markdown，必须含“允许修改:”段落")
    parser.add_argument("--slug", required=True, help="任务短名，用于分支 codex/<slug> 和 worktree 目录")
    parser.add_argument("--base", default="HEAD", help="worktree 基于的提交，默认当前 HEAD")
    parser.add_argument("--effort", choices=("low", "medium", "high", "xhigh"), default="high")
    # config.toml 默认的 gpt-6.1-sol 在 ChatGPT 账号下拒绝 exec；2026-10-22 实测 gpt-5.6-sol 可用（需桌面版自带 CLI）。
    parser.add_argument("--model", default="gpt-5.6-sol", help="Codex 模型，默认 gpt-5.6-sol")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-z0-9][a-z0-9-]{1,40}", args.slug):
        raise SystemExit("slug 只允许小写字母、数字和连字符")
    git_bin = find_git()
    brief = args.brief.read_text()
    allow = allowed_paths(brief)
    if bad := [p for p in allow if matches(p, PROTECTED)]:
        raise SystemExit(f"任务单允许修改了受保护路径，改由 Claude 处理：{bad}")
    if git(git_bin, "status", "--porcelain"):
        raise SystemExit("主工作区有未提交改动；先提交，worktree 才能拿到完整基线")

    worktree, branch = WORKTREES / args.slug, f"codex/{args.slug}"
    if worktree.exists():
        raise SystemExit(f"{worktree} 已存在；审查完后用 `git worktree remove` 清理再重派")
    WORKTREES.mkdir(exist_ok=True)
    git(git_bin, "worktree", "add", "-q", "-b", branch, str(worktree), args.base)
    for name in LINKED:
        if (ROOT / name).is_dir():
            (worktree / name).symlink_to(ROOT / name)

    run_dir = RUNS / time.strftime(f"%Y%m%d-%H%M%S-{args.slug}")
    run_dir.mkdir(parents=True)
    schema, report_file, log_file = run_dir / "schema.json", run_dir / "report.json", run_dir / "codex.log"
    schema.write_text(json.dumps(REPORT_SCHEMA, ensure_ascii=False))
    prompt = PREAMBLE.format(protected="、".join(PROTECTED), python=ROOT / ".venv/bin/python") + brief
    (run_dir / "prompt.md").write_text(prompt)
    command = [find_codex(), "exec", "-C", str(worktree), "-s", "workspace-write",
               "-c", f"model_reasoning_effort={json.dumps(args.effort)}",
               "--output-schema", str(schema), "-o", str(report_file), "-"]
    command[2:2] = ["-m", args.model]
    started = time.monotonic()
    with log_file.open("w") as log:  # Full transcript stays on disk, out of the reviewer's context.
        code = subprocess.run(command, input=prompt, text=True, stdout=log, stderr=subprocess.STDOUT).returncode

    changed = changed_files(git_bin, worktree)
    try:
        report = json.loads(report_file.read_text())
    except (OSError, ValueError):
        report = None
    result = dict(slug=args.slug, branch=branch, worktree=str(worktree), run_dir=str(run_dir), exit_code=code,
                  minutes=round((time.monotonic() - started) / 60, 1), allowed=allow, changed=changed,
                  claimed_matches_actual=report is not None and sorted(report["files_changed"]) == changed,
                  **audit(changed, allow), report=report,
                  diffstat=git(git_bin, "diff", "--stat", cwd=worktree).strip().splitlines()[-1:])
    (run_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if code == 0 and report and not result["protected_touched"] and not result["outside_allowed"] else 1)


if __name__ == "__main__":
    main()
