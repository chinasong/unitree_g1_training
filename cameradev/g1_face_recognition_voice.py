import sys
import os
import time
import rospy
import cv2
import face_recognition
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

bridge = CvBridge()
known_encodings = []
known_names = []
last_spoken = ""
last_spoken_time = time.time()

# 初始化手势状态标记
hand_raised = False
last_detect_time = 0
owner_present = False

def init_audio_client(net_iface: str):
    # ChannelFactoryInitialize(0, net_iface)
    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()
    audio_client.SetVolume(85)
    return audio_client

def speak(audio_client, text):
    print("📢", text)
    audio_client.TtsMaker(text, 0)

def load_known_faces(folder="faces"):
    for file in os.listdir(folder):
        path = os.path.join(folder, file)
        img = face_recognition.load_image_file(path)
        encodings = face_recognition.face_encodings(img)
        if encodings:
            for enc in encodings:
                known_encodings.append(enc)
                name = os.path.splitext(file)[0].split("_")[0]  # 截取前缀名
                known_names.append(name)
                print(f"✅ 已学习：{name}")

# 全局状态记录
name_last_time = {}
last_owner_seen_time = 0
REPEAT_DELAY = 6           # 同一个人说话间隔时间
OWNER_DISAPPEAR_DELAY = 5  # 主人离开多久后手放下

def image_callback(msg):
    global name_last_time, hand_raised, last_owner_seen_time

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    face_locations = face_recognition.face_locations(rgb_frame)
    encodings = face_recognition.face_encodings(rgb_frame, face_locations)

    current_time = time.time()
    recognized_names = []

    for encoding in encodings:
        matches = face_recognition.compare_faces(known_encodings, encoding)
        name = "陌生人"
        if True in matches:
            index = matches.index(True)
            name = known_names[index]

        recognized_names.append(name)

        if name not in name_last_time or current_time - name_last_time[name] > REPEAT_DELAY:
            speak(audio_client, f"你好，{name}")
            name_last_time[name] = current_time

            if name == "主人" and not hand_raised:
                loco_client.ShakeHand()
                hand_raised = True

        if name == "主人":
            last_owner_seen_time = current_time

    # 如果握手状态且主人已离开超时
    if hand_raised and (current_time - last_owner_seen_time > OWNER_DISAPPEAR_DELAY):
        loco_client.HighStand()  # 或者换成单独“放手”的动作
        hand_raised = False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        sys.exit(-1)

    net_iface = sys.argv[1]
    # 初始化 DDS 通道（一定要放在最前）
    ChannelFactoryInitialize(0, net_iface)

    audio_client = init_audio_client(net_iface)
    # 初始化动作客户端
    loco_client = LocoClient()
    loco_client.SetTimeout(10.0)
    loco_client.Init()

    load_known_faces()
    rospy.init_node("g1_face_recognition_node")
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    print("🎥 G1 正在进行人脸识别，按 Ctrl+C 退出")
    rospy.spin()