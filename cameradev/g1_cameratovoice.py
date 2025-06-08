import sys
import time
import rospy
import cv2
import json
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

# 加载英文->中文映射
with open("en_to_zh.js", "r") as f:
    label_map = json.load(f)

bridge = CvBridge()
last_labels = set()
last_spoken_time = time.time()

def init_audio_client(net_iface: str):
    ChannelFactoryInitialize(0, net_iface)
    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()
    audio_client.SetVolume(85)
    return audio_client

def speak(audio_client, text):
    print(f"📢 播报内容：{text}")
    audio_client.TtsMaker(text, 0)

def image_callback(msg):
    global last_labels, last_spoken_time
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    results = model(frame)
    boxes = results[0].boxes
    classes = [int(cls) for cls in boxes.cls]

    current_labels = set()
    for cls in classes:
        en = model.names[cls]
        zh = label_map.get(en, en)
        current_labels.add(zh)

    if current_labels and (current_labels != last_labels) and (time.time() - last_spoken_time > 3):
        text = "我看到了：" + "、".join(current_labels)
        speak(audio_client, text)
        last_labels = current_labels
        last_spoken_time = time.time()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        sys.exit(-1)

    net_iface = sys.argv[1]
    audio_client = init_audio_client(net_iface)

    rospy.init_node("g1_cam_object_recognition")
    model = YOLO("yolo11n.pt")
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    print("✅ 已开始识别 G1 摄像头画面，按 Ctrl+C 退出")
    rospy.spin()