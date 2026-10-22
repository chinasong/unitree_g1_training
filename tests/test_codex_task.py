"""Codex dispatch guard: brief parsing and change audit, no Codex process involved."""
import pytest

from scripts.dev.codex_task import PROTECTED, allowed_paths, audit, matches


def test_brief_must_declare_allowed_paths():
    brief = "目标：补测试\n\n允许修改:\n- tests/test_x.py\n- `doc/*.md`\n\n验收：pytest\n"
    assert allowed_paths(brief) == ["tests/test_x.py", "doc/*.md"]
    with pytest.raises(SystemExit):
        allowed_paths("目标：没有写允许范围")


def test_safety_gate_and_robot_entry_points_are_protected():
    for path in ("g1_console/service.py", "g1_console/adapters.py", "g1_console/capabilities.py",
                 "scripts/connect_orin_console.py", "连接G1软件遥控器.command", "scripts/dev/codex_task.py", "CLAUDE.md"):
        assert matches(path, PROTECTED), path
    assert not matches("g1_console/insight.py", PROTECTED)


def test_audit_flags_protected_and_out_of_scope_changes():
    result = audit(["tests/test_x.py", "g1_console/service.py", "README.md"], ["tests/", "g1_console/service.py"])
    assert result == dict(protected_touched=["g1_console/service.py"], outside_allowed=["README.md"])
