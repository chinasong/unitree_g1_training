from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
import time

ChannelFactoryInitialize(0, "enp61s0")
publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
publisher.Init()

cmd = LowCmd_()
motor = cmd.motor_cmd[15]
motor.q = 0.8
motor.mode = 0x0A
motor.Kp = 20.0
motor.Kd = 2.0

for _ in range(100):
    publisher.Write(cmd)
    time.sleep(0.01)