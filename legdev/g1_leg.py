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
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms 
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
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof


class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s]
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
        
        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0
            print(self.low_state.imu_state.rpy)

    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        if self.time_ < self.duration_ :
            # [Stage 1]: set robot to zero posture
            for i in range(G1_NUM_MOTOR):
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                self.low_cmd.mode_pr = Mode.PR
                self.low_cmd.mode_machine = self.mode_machine_
                self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                self.low_cmd.motor_cmd[i].tau = 0. 
                self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q 
                self.low_cmd.motor_cmd[i].dq = 0. 
                self.low_cmd.motor_cmd[i].kp = Kp[i] 
                self.low_cmd.motor_cmd[i].kd = Kd[i]

        elif self.time_ < self.duration_ * 5:

            # [Stage 4]: Alternating Slow Jogging Gait with Natural Phasing

            t = self.time_ - self.duration_ * 2

            period = 2.4  # 每条腿完整跑步周期（秒）

            phase_l = (t % period) / period  # 左腿相位

            phase_r = ((t + period / 2) % period) / period  # 右腿相位 = 左腿滞后半周期

            def gait_phase(phase):

                # phase ∈ [0, 1]

                # 0~0.3: 蹬地

                # 0.3~0.6: 抬腿

                # 0.6~1.0: 落地支撑

                if phase < 0.3:  # 蹬地

                    hip = -0.1

                    knee = 0.2

                    ankle = 0.25

                elif phase < 0.6:  # 抬腿

                    p = (phase - 0.3) / 0.3

                    hip = np.deg2rad(20) * np.sin(np.pi * p)

                    knee = np.deg2rad(35) * np.sin(np.pi * p)

                    ankle = -0.25

                else:  # 落地支撑缓冲

                    p = (phase - 0.6) / 0.4

                    hip = 0.0

                    knee = 0.2 * (1 - np.cos(np.pi * p)) / 2

                    ankle = 0.2

                return hip, knee, ankle

            # 获取左右腿期望关节角度

            L_HipPitch, L_Knee, L_Ankle = gait_phase(phase_l)

            R_HipPitch, R_Knee, R_Ankle = gait_phase(phase_r)

            # 设置控制指令

            self.low_cmd.mode_pr = Mode.PR

            self.low_cmd.mode_machine = self.mode_machine_

            for i in range(G1_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].mode = 1

                self.low_cmd.motor_cmd[i].tau = 0

                self.low_cmd.motor_cmd[i].dq = 0

                self.low_cmd.motor_cmd[i].kp = 80.0 if i in [G1JointIndex.LeftKnee, G1JointIndex.RightKnee] else Kp[i]

                self.low_cmd.motor_cmd[i].kd = 2.0 if i in [G1JointIndex.LeftKnee, G1JointIndex.RightKnee] else Kd[i]

            self.low_cmd.motor_cmd[G1JointIndex.LeftHipPitch].q = L_HipPitch

            self.low_cmd.motor_cmd[G1JointIndex.LeftKnee].q = -L_Knee

            self.low_cmd.motor_cmd[G1JointIndex.LeftAnklePitch].q = L_Ankle

            self.low_cmd.motor_cmd[G1JointIndex.RightHipPitch].q = R_HipPitch

            self.low_cmd.motor_cmd[G1JointIndex.RightKnee].q = -R_Knee

            self.low_cmd.motor_cmd[G1JointIndex.RightAnklePitch].q = R_Ankle
    

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:        
        time.sleep(1)