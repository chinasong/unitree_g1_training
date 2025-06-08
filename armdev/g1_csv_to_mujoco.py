import argparse
import mujoco
import mujoco.viewer
import numpy as np
import pandas as pd
import time

# 目标关节（上肢 + 腰）对应 qpos 索引
TARGET_QPOS_INDICES = [
    12,  # WAIST_YAW
    15, 16, 17, 18, 19, 20, 21,  # LEFT_ARM
    22, 23, 24, 25, 26, 27, 28   # RIGHT_ARM
]

def set_initial_pose(d):
    # 设置 base 位姿（位置 + 姿态四元数）
    d.qpos[:7] = np.array([
        0.0, 0.0, 0.78,     # base: x, y, z
        1.0, 0.0, 0.0, 0.0  # base: quaternion (w, x, y, z)
    ])
    d.qvel[:6] = 0.0  # base 线速度 + 角速度归零，防止滑动或翻转

    # 设置下肢初始姿态：左腿（L）+ 右腿（R），共 8 DOF
    default_leg_qpos = [
        0.0, 0.0, 0.0,   # L_HIP pitch, roll, yaw
        0.0, 0.0,       # L_KNEE, L_ANKLE pitch
        0.0, 0.0, 0.0        # R_KNEE, R_ANKLE pitch
    ]
    assert len(default_leg_qpos) == 8, f"❌ default_leg_qpos 长度为 {len(default_leg_qpos)}，应为 8"
    d.qpos[7:15] = default_leg_qpos
    d.qvel[7:15] = 0.0  # 下肢关节速度归零，防止动态抖动

def interpolate_between_frames(frame_a, frame_b, steps):
    interp_frames = []
    for s in range(steps):
        ratio = s / steps
        frame = (1 - ratio) * frame_a + ratio * frame_b
        interp_frames.append(frame)
    return interp_frames



def replay_from_csv(csv_path, xml_path, fps=30, interp_steps=4):
    df = pd.read_csv(csv_path)
    q_columns = [col for col in df.columns if col.endswith("_q") and col != "time"]
    q_data = df[q_columns].to_numpy(dtype=np.float32)

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)

    assert len(q_columns) == len(TARGET_QPOS_INDICES), "CSV 列数与关节索引不一致"
    assert m.nq > max(TARGET_QPOS_INDICES), "模型不含足够关节"

    print(f"🎬 准备回放 {len(q_data)} 帧，每帧插 {interp_steps} 帧，共 {len(q_data) * interp_steps} 小帧")

    m.opt.timestep = 1.0 / (fps * interp_steps)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        set_initial_pose(d)
        mujoco.mj_forward(m, d)

        for i in range(len(q_data) - 1):
            frame_a = q_data[i]
            frame_b = q_data[i + 1]
            interp_frames = interpolate_between_frames(frame_a, frame_b, interp_steps)

            for q_frame in interp_frames:
                set_initial_pose(d)
                for j, idx in enumerate(TARGET_QPOS_INDICES):
                    d.qpos[idx] = q_frame[j]
                    d.qvel[idx] = 0.0
                mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(m.opt.timestep)  # 控制每一小帧播放间隔

        print("✅ 回放结束")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="CSV 路径")
    parser.add_argument("--xml", default="g1_23dof.xml")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--interp", type=int, default=4, help="每两个帧间插值帧数")
    args = parser.parse_args()

    replay_from_csv(args.csv, args.xml, args.fps, args.interp)

if __name__ == "__main__":
    main()