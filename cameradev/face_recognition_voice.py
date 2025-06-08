import face_recognition
import cv2
import os
import sys
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

# 初始化语音客户端
def init_audio_client(net_iface: str):
    ChannelFactoryInitialize(0, net_iface)
    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()
    audio_client.SetVolume(85)
    return audio_client

def speak(audio_client, text):
    print("📢", text)
    audio_client.TtsMaker(text, 0)

# 加载已知人脸
def load_known_faces(folder="faces"):
    known_encodings = []
    known_names = []
    for file in os.listdir(folder):
        path = os.path.join(folder, file)
        img = face_recognition.load_image_file(path)
        encodings = face_recognition.face_encodings(img)
        if encodings:
            known_encodings.append(encodings[0])
            name = os.path.splitext(file)[0]
            known_names.append(name)
            print(f"✅ 已学习：{name}")
    return known_encodings, known_names

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        sys.exit(-1)

    net_iface = sys.argv[1]
    audio_client = init_audio_client(net_iface)

    known_encodings, known_names = load_known_faces()

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 摄像头打不开")
        sys.exit(-1)

    last_spoken = ""
    last_spoken_time = 0
    speak_interval = 10  # 每个识别对象间隔多少秒重复播报一次
    print("🎥 开始识别人脸，按 q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)
        encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        for face_encoding in encodings:
            matches = face_recognition.compare_faces(known_encodings, face_encoding)
            name = "陌生人"
            if True in matches:
                first_match_index = matches.index(True)
                name = known_names[first_match_index]

            if name != last_spoken or (current_time - last_spoken_time > speak_interval):
                speak(audio_client, f"你好，{name}")
                last_spoken = name
                last_spoken_time = current_time

        # 显示画面
        for (top, right, bottom, left), face_encoding in zip(face_locations, encodings):
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Face Recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()