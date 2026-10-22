import json
from pathlib import Path

ASSETS = Path(__file__).parent / "assets"
BODY = json.loads((ASSETS / "g1_23.json").read_text())
KNOWLEDGE = json.loads((ASSETS / "g1_motor_knowledge.json").read_text())
BODY["motor_knowledge"] = KNOWLEDGE
JOINTS = BODY["joints"]
INDICES = tuple(j["index"] for j in JOINTS)
assert len(INDICES) == 23 and len(set(INDICES)) == 23
assert not set(INDICES) & {13, 14, 20, 21, 27, 28, 29}


def empty_joints():
    return [dict(index=j["index"], q=None, dq=None, ddq=None, tau=None,
                 temperature=None, voltage=None, status=None, mode=None, sensor=None, reserve=None) for j in JOINTS]
