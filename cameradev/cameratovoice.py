import time
import sys
import json
import cv2
from ultralytics import YOLO

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

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

def load_label_map(file_path: str):
    with open(file_path, "r", encoding="utf-8") as f:
        return json.load(f)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        sys.exit(-1)

    net_iface = sys.argv[1]
    audio_client = init_audio_client(net_iface)

    # 加载 YOLO 模型
    model = YOLO("yolo11n.pt")
    names = model.names

    # 加载英文 -> 中文标签映射
    en_to_zh = load_label_map("en_to_zh.js")

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 摄像头打开失败")
        sys.exit(-1)

    print("✅ 开始识别物体并播报，按 q 退出")

    last_labels = set()
    last_spoken_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame)
        boxes = results[0].boxes

        if boxes is None or len(boxes) == 0:
            continue

        # 提取预测类别，按置信度排序
        sorted_labels = [
            (names[int(cls)], float(score))
            for cls, score in zip(boxes.cls, boxes.conf)
        ]
        sorted_labels.sort(key=lambda x: x[1], reverse=True)

        # 取前5个最可信的标签
        current_labels = set([label for label, _ in sorted_labels[:5]])
        current_zh_labels = set([en_to_zh.get(label, label) for label in current_labels])

        if current_zh_labels != last_labels and time.time() - last_spoken_time > 3:
            text = "我看到了：" + "、".join(current_zh_labels)
            speak(audio_client, text)
            last_labels = current_zh_labels
            last_spoken_time = time.time()

        annotated = results[0].plot()
        cv2.imshow("G1 Camera", annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()