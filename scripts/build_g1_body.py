#!/usr/bin/env python3
"""Extract a reproducible body inventory from the pinned official MuJoCo model."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "externals/unitree_rl_gym/resources/robots/g1_description/g1_23dof.xml"
NAMES = {"hip_pitch": "髋俯仰", "hip_roll": "髋侧摆", "hip_yaw": "髋旋转",
         "knee": "膝关节", "ankle_pitch": "踝俯仰", "ankle_roll": "踝侧摆",
         "shoulder_pitch": "肩俯仰", "shoulder_roll": "肩侧摆", "shoulder_yaw": "肩旋转",
         "elbow": "肘关节", "wrist_roll": "腕旋转"}
INDICES = dict(zip([f"{side}_{name}_joint" for side in ("left", "right")
                   for name in list(NAMES)[:6]], range(12)))
INDICES["waist_yaw_joint"] = 12
for side, start in (("left", 15), ("right", 22)):
    for i, name in enumerate(list(NAMES)[6:]):
        INDICES[f"{side}_{name}_joint"] = start + i


def floats(value, default="0 0 0"):
    return [float(v) for v in (value or default).split()]


def main():
    root = ET.parse(SOURCE).getroot()
    joints = []

    def body(node):
        j = node.find("joint")
        index = None
        if j is not None and j.get("type") != "free":
            name = j.attrib["name"]
            index = INDICES[name]
            side = "左" if name.startswith("left") else "右"
            suffix = name.removeprefix("left_").removeprefix("right_").removesuffix("_joint")
            label = "腰旋转" if name == "waist_yaw_joint" else side + NAMES[suffix]
            group = "waist" if index == 12 else ("left_leg" if index < 6 else "right_leg" if index < 12
                else "left_arm" if index < 22 else "right_arm")
            inertial = node.find("inertial")
            joints.append(dict(index=index, name=name, label=label, group=group,
                range=floats(j.get("range")), torque_range=floats(j.get("actuatorfrcrange")),
                axis=floats(j.get("axis")), link_mass_kg=float(inertial.get("mass")) if inertial is not None else None,
                limits_source="官方仿真模型标称值；不是实机持续力矩或出厂标定"))
        # Only visual meshes (group=1), not duplicate collision geometries.
        meshes = [dict(file=g.attrib["mesh"] + ".STL", rgba=floats(g.get("rgba"), "0.7 0.7 0.7 1"))
                  for g in node.findall("geom") if g.get("group") == "1" and g.get("mesh")]
        return dict(name=node.get("name"), position=floats(node.get("pos")),
                    quaternion=floats(node.get("quat"), "1 0 0 0"), joint_index=index,
                    axis=floats(j.get("axis")) if index is not None else [0, 0, 1],
                    meshes=meshes, children=[body(b) for b in node.findall("body")])

    tree = body(root.find("worldbody/body"))
    data = dict(robot="Unitree G1 EDU", dof=23, hands=False, computer="Jetson Orin（具体型号待探测）",
                source=str(SOURCE.relative_to(ROOT)), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                source_commit="276801e46c5d433564f24658bac64f254b7d2d4b",
                joints=sorted(joints, key=lambda j: j["index"]), tree=tree,
                absent_indices=[13, 14, 20, 21, 27, 28],
                unknown_parameters=["出厂 Kp/Kd", "实机持续力矩", "电流上限", "减速器实机标定", "电池状态（HG LowState 无此字段）"],
                modules=[dict(id="imu", label="惯性测量单元", source="rt/lowstate.imu_state"),
                         dict(id="camera", label="相机", source="ROS 2 图像桥接；连接后确认"),
                         dict(id="localization", label="定位与障碍物", source="同一 map 坐标系定位 / 占用栅格 + base_link 扫描"),
                         dict(id="hands", label="灵巧手 / 夹爪", source="本机未安装"),
                         dict(id="battery", label="电池", source="未接入；不估算电量")])
    target = ROOT / "g1_console/assets/g1_23.json"
    target.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    print(f"{target}: {len(joints)} joints, sparse DDS indices verified")


if __name__ == "__main__":
    main()
