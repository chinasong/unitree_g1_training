import mujoco
import mujoco.viewer
import numpy as np
import pandas as pd
import time
from mujoco import MjModel, MjData
from datetime import datetime

# 加载模型
model = mujoco.MjModel.from_xml_path("g1_23dof.xml")
data = mujoco.MjData(model)

# 右臂关节映射（仿真名 -> CSV列名）
right_arm_joint_mapping = {
    "right_shoulder_pitch_joint": "R_SHOULDER_PITCH_q",
    "right_shoulder_roll_joint": "R_SHOULDER_ROLL_q",
    "right_shoulder_yaw_joint": "R_SHOULDER_YAW_q",
    "right_elbow_joint": "R_ELBOW_q",
    "right_wrist_roll_joint": "R_WRIST_ROLL_q"
}

# 获取对应 joint 的 qposadr 索引
joint_indices = {csv_name: model.joint(name).qposadr[0] for name, csv_name in right_arm_joint_mapping.items()}

# 存储数据
record = []
start_time = time.time()

# 启动可视化窗口
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("🎥 开始录制右臂轨迹，关闭窗口即可保存 CSV 文件。")
    while viewer.is_running():
        mujoco.mj_step(model, data)

        t = time.time() - start_time
        row = {"time": t}
        for csv_name, idx in joint_indices.items():
            row[csv_name] = data.qpos[idx]

        record.append(row)

# 保存为 CSV
df = pd.DataFrame(record)
ts = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"g1_right_arm_{ts}.csv"
df.to_csv(filename, index=False)

print(f"✅ 保存成功: {filename}")