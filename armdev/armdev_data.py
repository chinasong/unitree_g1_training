import sys
import argparse
import pandas as pd
import time

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

# 关节名称与索引（包含腿部、腰部和手臂）
JOINT_NAMES = [
    "L_LEG_HIP_PITCH", "L_LEG_HIP_ROLL", "L_LEG_HIP_YAW",
    "L_LEG_KNEE", "L_LEG_ANKLE_PITCH", "L_LEG_ANKLE_ROLL",
    "R_LEG_HIP_PITCH", "R_LEG_HIP_ROLL", "R_LEG_HIP_YAW",
    "R_LEG_KNEE", "R_LEG_ANKLE_PITCH", "R_LEG_ANKLE_ROLL",
    "WAIST_YAW", "WAIST_ROLL", "WAIST_PITCH",
    "L_SHOULDER_PITCH", "L_SHOULDER_ROLL", "L_SHOULDER_YAW", "L_ELBOW",
    "L_WRIST_ROLL", "L_WRIST_PITCH", "L_WRIST_YAW",
    "R_SHOULDER_PITCH", "R_SHOULDER_ROLL", "R_SHOULDER_YAW", "R_ELBOW",
    "R_WRIST_ROLL", "R_WRIST_PITCH", "R_WRIST_YAW"
]
JOINT_INDEXES = list(range(29))

class CSVPlayer:
    def __init__(self, csv_path, interval=0.02):
        self.data = pd.read_csv(csv_path)
        self.index = 0
        self.interval = interval
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.done = False
        self.boot_counter = 0  # 启动阶段发送几帧 arm_sdk 启动命令

    def init_channels(self):
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()

    def start(self):
        self.thread = RecurrentThread(interval=self.interval, target=self.send_frame)
        self.thread.Start()

    def send_frame(self):
        if self.boot_counter < 10:
            self.low_cmd.motor_cmd[29].q = 1
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)
            self.boot_counter += 1
            return

        if self.index >= len(self.data) - 1:
            self.done = True
            return

        # 插值参数
        interp_steps = 4  # 每两个CSV帧之间插4帧
        step_ratio = 1.0 / interp_steps

        row_start = self.data.iloc[self.index]
        row_end = self.data.iloc[self.index + 1]

        for s in range(interp_steps):
            ratio = s * step_ratio
            for name, idx in zip(JOINT_NAMES, JOINT_INDEXES):
                q_start = row_start.get(f"{name}_q", 0.0)
                q_end = row_end.get(f"{name}_q", 0.0)
                dq_start = row_start.get(f"{name}_dq", 0.0)
                dq_end = row_end.get(f"{name}_dq", 0.0)
                tau_start = row_start.get(f"{name}_tau", 0.0)
                tau_end = row_end.get(f"{name}_tau", 0.0)

                self.low_cmd.motor_cmd[idx].q = (1 - ratio) * q_start + ratio * q_end
                self.low_cmd.motor_cmd[idx].dq = (1 - ratio) * dq_start + ratio * dq_end
                self.low_cmd.motor_cmd[idx].tau = (1 - ratio) * tau_start + ratio * tau_end
                self.low_cmd.motor_cmd[idx].kp = 60.0
                self.low_cmd.motor_cmd[idx].kd = 1.5

            self.low_cmd.motor_cmd[29].q = 1
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)
            time.sleep(self.interval)  # 内部 sleep，平滑过渡

        self.index += 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", required=True, help="Network interface connected to G1")
    parser.add_argument("--csv", required=True, help="Path to joint trajectory CSV file")
    args = parser.parse_args()

    print("WARNING: Make sure the robot's limbs are free to move.")
    input("Press Enter to begin playback...")

    # 初始化 DDS 网络接口
    ChannelFactoryInitialize(0, args.iface)

    # 播放器初始化并开始
    player = CSVPlayer(args.csv)
    player.init_channels()
    player.start()

    while not player.done:
        time.sleep(0.1)

    print("Playback finished.")

if __name__ == "__main__":
    main()