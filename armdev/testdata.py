from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
import time

def callback(msg: LowState_):
    print("Field list for motor_state[0]:")
    print(dir(msg.motor_state[0]))
    exit()

ChannelFactoryInitialize(0, "enp61s0")
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(callback, 1)

while True:
    time.sleep(0.5)