#!/usr/bin/env python3
"""Offline SDK/API, message/CRC and G1 model smoke checks; no robot connection."""

import ast
import importlib
import inspect
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def main():
    subprocess.run(
        [sys.executable, str(ROOT / "scripts/sync_unitree_deps.py"), "--check"],
        check=True,
    )
    sys.path.insert(0, str(ROOT / "externals/unitree_sdk2_python"))

    # Import the symbols used by this project's scripts without running them.
    imports = set()
    methods = set()
    sources = list(ROOT.glob("*.py")) + list(ROOT.glob("*/*.py"))
    for source in sources:
        if source.parent.name == "externals":
            continue
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and (node.module or "").startswith("unitree_sdk2py."):
                imports.update((node.module, alias.name) for alias in node.names)
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and isinstance(node.func.value, ast.Name)
                    and node.func.value.id in ("audio_client", "loco_client")):
                methods.add((node.func.value.id, node.func.attr))
    skipped = 0
    for module, symbol in sorted(imports):
        if sys.platform != "linux" and module == "unitree_sdk2py.utils.thread":
            source = ROOT / "externals/unitree_sdk2_python" / (module.replace(".", "/") + ".py")
            definitions = ast.parse(source.read_text(encoding="utf-8"))
            if not any(isinstance(node, ast.ClassDef) and node.name == symbol for node in definitions.body):
                raise RuntimeError(f"Missing SDK class: {module}.{symbol}")
            print(f"SKIP runtime {module}.{symbol}: Linux timerfd required; class definition present")
            skipped += 1
            continue
        getattr(importlib.import_module(module), symbol)
    print(f"OK {len(imports) - skipped} SDK imports; {skipped} Linux-only imports require Linux verification")

    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    clients = {"audio_client": AudioClient, "loco_client": LocoClient}
    for client, method in sorted(methods):
        if not callable(getattr(clients[client], method)):
            raise RuntimeError(f"Missing SDK method: {client}.{method}")
    inspect.signature(AudioClient.TtsMaker).bind(None, "offline", 0)
    inspect.signature(AudioClient.SetVolume).bind(None, 85)
    inspect.signature(LocoClient.ShakeHand).bind(None)
    print(f"OK {len(methods)} existing client methods and audio/handshake signatures")

    command = unitree_hg_msg_dds__LowCmd_()
    decoded = LowCmd_.deserialize(command.serialize())
    if len(decoded.motor_cmd) != 35:
        raise RuntimeError("Unexpected HG command layout")
    crc = CRC()
    original = crc.Crc(command)
    command.motor_cmd[15].q = 0.25
    changed = crc.Crc(command)
    if original == changed:
        raise RuntimeError("CRC failed to detect a changed motor command")
    print(f"OK HG message roundtrip and CRC change detection ({original} -> {changed})")

    import mujoco
    import numpy as np

    models = ROOT / "externals/unitree_rl_gym/resources/robots/g1_description"
    for dof in (23, 29):
        model = mujoco.MjModel.from_xml_path(str(models / f"g1_{dof}dof.xml"))
        if model.nu != dof or model.nq != dof + 7:
            raise RuntimeError(f"Unexpected G1 {dof} DOF model layout")
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        for _ in range(100):
            mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError(f"Non-finite G1 {dof} DOF simulation state")
        print(f"OK G1 {dof} DOF model and meshes: nq={model.nq}, nu={model.nu}, 100 headless steps")


if __name__ == "__main__":
    main()
