
import sys
import time
import argparse
import pandas as pd

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmd_

# G1 29 自由度中手臂控制电机
JOINT_NAMES = ['L_HIP_YAW', 'L_HIP_ROLL', 'L_HIP_PITCH', 'L_KNEE', 'L_ANKLE_PITCH', 'L_ANKLE_ROLL', 'R_HIP_YAW', 'R_HIP_ROLL', 'R_HIP_PITCH', 'R_KNEE', 'R_ANKLE_PITCH', 'R_ANKLE_ROLL', 'WAIST_YAW', 'WAIST_PITCH', 'WAIST_ROLL', 'L_SHOULDER_PITCH', 'L_SHOULDER_ROLL', 'L_SHOULDER_YAW', 'L_ELBOW', 'L_WRIST_ROLL', 'L_WRIST_PITCH', 'L_WRIST_YAW', 'R_SHOULDER_PITCH', 'R_SHOULDER_ROLL', 'R_SHOULDER_YAW', 'R_ELBOW', 'R_WRIST_ROLL', 'R_WRIST_PITCH', 'R_WRIST_YAW']
JOINT_INDEXES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]

class CSVPlayer:
    def __init__(self, csv_path, interval=0.02):
        self.data = pd.read_csv(csv_path)
        self.index = 0
        self.interval = interval
        self.low_cmd = LowCmd_()
        self.crc = CRC()
        self.done = False
        self.boot_counter = 0

        self.low_cmd.mode_pr = 0  # PR 控制模式
        self.low_cmd.mode_machine = 0

    def init_channels(self):
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()

    def start(self):
        self.thread = RecurrentThread(interval=self.interval, target=self.send_frame)
        self.thread.Start()

    def send_frame(self):
        if self.boot_counter < 10:
            for i in range(29):
                self.low_cmd.motor_cmd[i].mode = 0x01
                self.low_cmd.motor_cmd[i].q = 0
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].tau = 0
                self.low_cmd.motor_cmd[i].kp = 0
                self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)
            self.boot_counter += 1
            return

        if self.index >= len(self.data) - 1:
            self.done = True
            return

        interp_steps = 4
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

                self.low_cmd.motor_cmd[idx].mode = 0x01
                self.low_cmd.motor_cmd[idx].q = (1 - ratio) * q_start + ratio * q_end
                self.low_cmd.motor_cmd[idx].dq = (1 - ratio) * dq_start + ratio * dq_end
                self.low_cmd.motor_cmd[idx].tau = (1 - ratio) * tau_start + ratio * tau_end
                self.low_cmd.motor_cmd[idx].kp = 60.0
                self.low_cmd.motor_cmd[idx].kd = 1.5

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)
            time.sleep(self.interval)

        self.index += 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", required=True, help="Network interface connected to G1")
    parser.add_argument("--csv", required=True, help="Path to joint trajectory CSV file")
    args = parser.parse_args()

    print("WARNING: Make sure the robot's arms are free to move.")
    input("Press Enter to begin playback...")

    ChannelFactoryInitialize(0, args.iface)

    player = CSVPlayer(args.csv)
    player.init_channels()
    player.start()

    while not player.done:
        time.sleep(0.1)

    print("Playback finished.")

if __name__ == "__main__":
    main()
