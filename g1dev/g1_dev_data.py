import time
import sys
import pandas as pd

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

        # Trajectory preload
        self.traj_left_arm = None
        self.traj_right_arm = pd.read_csv("../recorddata/g1_right_arm_20250609_145913.csv")
        self.traj_waist = None
        self.traj_left_leg = None
        self.traj_right_leg = None

        self.traj_start_time = time.time()

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

    def lerp(self, val1, val2, r):
        return (1 - r) * val1 + r * val2

    def apply_traj(self, df, mapping):
        # 当前回放时间（确保与 CSV 的 time 对齐）
        t = time.time() - self.traj_start_time
        t = max(t, df["time"].min())
        t = min(t, df["time"].max())

        past = df[df["time"] <= t]
        future = df[df["time"] >= t]

        if past.empty or future.empty:
            return

        r1, r2 = past.iloc[-1], future.iloc[0]
        t1, t2 = r1["time"], r2["time"]
        ratio = (t - t1) / (t2 - t1) if t2 > t1 else 0

        for idx, col in mapping.items():
            if col not in r1 or col not in r2:
                continue  # 跳过缺失列

            q = self.lerp(r1[col], r2[col], ratio)
            self.low_cmd.motor_cmd[idx].mode = 1
            self.low_cmd.motor_cmd[idx].q = q
            self.low_cmd.motor_cmd[idx].dq = 0
            self.low_cmd.motor_cmd[idx].tau = 0
            self.low_cmd.motor_cmd[idx].kp = Kp[idx]
            self.low_cmd.motor_cmd[idx].kd = Kd[idx]

    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        # 初始化指令
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1
            self.low_cmd.motor_cmd[i].tau = 0
            self.low_cmd.motor_cmd[i].dq = 0
            # self.low_cmd.motor_cmd[i].kp = Kp[i]
            # self.low_cmd.motor_cmd[i].kd = Kd[i]

        # 分别处理每个部位
        if self.traj_left_arm is not None:
            self.apply_traj(self.traj_left_arm, {
                G1JointIndex.LeftShoulderPitch: "L_SHOULDER_PITCH_q",
                G1JointIndex.LeftShoulderRoll: "L_SHOULDER_ROLL_q",
                G1JointIndex.LeftShoulderYaw: "L_SHOULDER_YAW_q",
                G1JointIndex.LeftElbow: "L_ELBOW_q",
                G1JointIndex.LeftWristRoll: "L_WRIST_ROLL_q",
                G1JointIndex.LeftWristPitch: "L_WRIST_PITCH_q",
                G1JointIndex.LeftWristYaw: "L_WRIST_YAW_q"
            })

        if self.traj_right_arm is not None:
            self.apply_traj(self.traj_right_arm, {
                G1JointIndex.RightShoulderPitch: "R_SHOULDER_PITCH_q",
                G1JointIndex.RightShoulderRoll: "R_SHOULDER_ROLL_q",
                G1JointIndex.RightShoulderYaw: "R_SHOULDER_YAW_q",
                G1JointIndex.RightElbow: "R_ELBOW_q",
                G1JointIndex.RightWristRoll: "R_WRIST_ROLL_q",
                G1JointIndex.RightWristPitch: "R_WRIST_PITCH_q",
                G1JointIndex.RightWristYaw: "R_WRIST_YAW_q"
            })

        if self.traj_waist is not None:
            self.apply_traj(self.traj_waist, {
                G1JointIndex.WaistYaw: "WAIST_YAW_q"
            })

        if self.traj_left_leg is not None:
            self.apply_traj(self.traj_left_leg, {
                G1JointIndex.LeftHipPitch: "L_LEG_HIP_PITCH_q",
                G1JointIndex.LeftHipRoll: "L_LEG_HIP_ROLL_q",
                G1JointIndex.LeftHipYaw: "L_LEG_HIP_YAW_q",
                G1JointIndex.LeftKnee: "L_LEG_KNEE_q",
                G1JointIndex.LeftAnklePitch: "L_LEG_ANKLE_PITCH_q",
                G1JointIndex.LeftAnkleRoll: "L_LEG_ANKLE_ROLL_q"
            })

        if self.traj_right_leg is not None:
            self.apply_traj(self.traj_right_leg, {
                G1JointIndex.RightHipPitch: "R_LEG_HIP_PITCH_q",
                G1JointIndex.RightHipRoll: "R_LEG_HIP_ROLL_q",
                G1JointIndex.RightHipYaw: "R_LEG_HIP_YAW_q",
                G1JointIndex.RightKnee: "R_LEG_KNEE_q",
                G1JointIndex.RightAnklePitch: "R_LEG_ANKLE_PITCH_q",
                G1JointIndex.RightAnkleRoll: "R_LEG_ANKLE_ROLL_q"
            })

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