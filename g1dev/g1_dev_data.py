import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np

G1_NUM_MOTOR = 29

Kp = [
    60, 60, 60, 100, 40, 40,  # legs
    60, 60, 60, 100, 40, 40,  # legs
    60, 40, 40,  # waist
    40, 40, 40, 40, 40, 40, 40,  # arms
    40, 40, 40, 40, 40, 40, 40  # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,  # legs
    1, 1, 1, 2, 1, 1,  # legs
    1, 1, 1,  # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1  # arms
]


class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13  # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13  # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14  # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14  # NOTE: INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20  # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21  # NOTE: INVALID for g1 23dof
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28  # NOTE: INVALID for g1 23dof


class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints


class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0  # [3 s]
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.update_mode_machine_ = False
        self.crc = CRC()

    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        # create subscriber #
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True

        self.counter_ += 1
        if (self.counter_ % 500 == 0):
            self.counter_ = 0
            print(self.low_state.imu_state.rpy)

    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        if self.time_ < self.duration_:
            # [Stage 1]: set robot to zero posture
            for i in range(G1_NUM_MOTOR):
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                self.low_cmd.mode_pr = Mode.PR
                self.low_cmd.mode_machine = self.mode_machine_
                self.low_cmd.motor_cmd[i].mode = 1  # 1:Enable, 0:Disable
                self.low_cmd.motor_cmd[i].tau = 0.
                self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q
                self.low_cmd.motor_cmd[i].dq = 0.
                self.low_cmd.motor_cmd[i].kp = Kp[i]
                self.low_cmd.motor_cmd[i].kd = Kd[i]




        elif self.time_ < self.duration_ + (self.traj_data["time"].iloc[-1] - self.traj_data["time"].iloc[0]):

            # 初始化轨迹

            if not hasattr(self, "traj_data"):
                import pandas as pd

                self.traj_data = pd.read_csv("../recorddata/g1_joint_framewise_20250609_140051.csv")

                self.traj_start_time = self.traj_data["time"].iloc[0]

                self.traj_end_time = self.traj_data["time"].iloc[-1]

            current_abs_time = self.traj_start_time + (self.time_ - self.duration_)

            df = self.traj_data

            if current_abs_time > self.traj_end_time:
                current_abs_time = self.traj_end_time

            # 插值准备

            future = df[df["time"] >= current_abs_time]

            past = df[df["time"] <= current_abs_time]

            if len(future) == 0 or len(past) == 0:
                print("[WARN] No valid data for interpolation")

                return

            row_next = future.iloc[0]

            row_prev = past.iloc[-1]

            t1 = row_prev["time"]

            t2 = row_next["time"]

            ratio = (current_abs_time - t1) / (t2 - t1) if t2 > t1 else 0.0

            def lerp(a, b, r):
                return (1 - r) * a + r * b

            # 控制指令填充

            self.low_cmd.mode_pr = Mode.PR

            self.low_cmd.mode_machine = self.mode_machine_

            for i in range(G1_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].mode = 1

                self.low_cmd.motor_cmd[i].tau = 0

                self.low_cmd.motor_cmd[i].dq = 0

                self.low_cmd.motor_cmd[i].kp = 80.0 if i in [G1JointIndex.LeftKnee, G1JointIndex.RightKnee] else Kp[i]

                self.low_cmd.motor_cmd[i].kd = 2.0 if i in [G1JointIndex.LeftKnee, G1JointIndex.RightKnee] else Kd[i]

            mapping = {

                G1JointIndex.LeftHipPitch: "L_LEG_HIP_PITCH_q",

                G1JointIndex.LeftKnee: "L_LEG_KNEE_q",

                G1JointIndex.LeftAnklePitch: "L_LEG_ANKLE_PITCH_q",

                G1JointIndex.RightHipPitch: "R_LEG_HIP_PITCH_q",

                G1JointIndex.RightKnee: "R_LEG_KNEE_q",

                G1JointIndex.RightAnklePitch: "R_LEG_ANKLE_PITCH_q",

            }

            for idx, col in mapping.items():
                q_prev = row_prev[col]

                q_next = row_next[col]

                q_interp = lerp(q_prev, q_next, ratio)

                self.low_cmd.motor_cmd[idx].q = q_interp

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)


if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:
        time.sleep(1)