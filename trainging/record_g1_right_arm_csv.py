import csv
import time
import mujoco
import mujoco.viewer
import numpy as np

# 加载 G1 模型（确保路径正确）
model = mujoco.MjModel.from_xml_path("../externals/unitree_rl_gym/resources/robots/g1_description/g1_23dof.xml")

data = mujoco.MjData(model)

# 打开 GUI 以手动拖动关节
viewer = mujoco.viewer.launch_passive(model, data)

# 要记录的右臂关节（与真实机器人命名一致）
joint_names = [
    "R_SHOULDER_PITCH_q",
    "R_SHOULDER_ROLL_q",
    "R_SHOULDER_YAW_q",
    "R_ELBOW_q",
    "R_WRIST_ROLL_q",
    "R_WRIST_PITCH_q",
    "R_WRIST_YAW_q"
]

# 计算这些关节在 qpos 中的索引
joint_indices = [model.joint(name).qposadr[0] for name in joint_names]

# CSV 文件保存路径
save_path = "g1_right_arm_manual_record.csv"

# 写入 CSV 文件头
with open(save_path, "w", newline="") as f:
    writer = csv.writer(f)
    header = ["time"] + joint_names
    writer.writerow(header)

    t = 0.0
    dt = 0.01  # 每 10ms 记录一次（对应控制频率 100Hz）

    print("开始录制动作，请在 GUI 中拖动右臂关节...")

    try:
        for _ in range(2000):  # 约 20 秒
            mujoco.mj_step(model, data)

            # 读取当前时间和各关节角度
            row = [t] + [data.qpos[i] for i in joint_indices]
            writer.writerow(row)

            t += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print("录制中断，已保存到 CSV。")

viewer.close()