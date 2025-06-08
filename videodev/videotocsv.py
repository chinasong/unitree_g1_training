import cv2
import mediapipe as mp
import pandas as pd
import numpy as np
import time

# 初始化 MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False)
landmark_enum = mp_pose.PoseLandmark

# 输出字段格式（共42个字段）
JOINT_NAMES = [
    "L_SHOULDER_PITCH", "L_SHOULDER_ROLL", "L_SHOULDER_YAW", "L_ELBOW",
    "L_WRIST_ROLL", "L_WRIST_PITCH", "L_WRIST_YAW",
    "R_SHOULDER_PITCH", "R_SHOULDER_ROLL", "R_SHOULDER_YAW", "R_ELBOW",
    "R_WRIST_ROLL", "R_WRIST_PITCH", "R_WRIST_YAW"
]
WAIST_NAMES = ["WAIST_YAW"]

COLUMNS = ["time"]
for name in JOINT_NAMES + WAIST_NAMES:
    for suffix in ["q", "dq", "tau"]:
        COLUMNS.append(f"{name}_{suffix}")

def get_angle(p1, p2, p3):
    """计算三个点组成的夹角（中间点为关节）"""
    a, b, c = np.array(p1), np.array(p2), np.array(p3)
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))  # 弧度
    return angle

def extract_joint_angles(landmarks):
    """从 landmarks 计算关节角度（目前只处理 L_ELBOW 和 R_ELBOW）"""
    row = {col: 0.0 for col in COLUMNS[1:]}  # 除 time 外全置 0

    def lm(name):
        l = landmarks[landmark_enum[name]]
        return [l.x, l.y]

    try:
        # 左肘角度（肩-肘-腕）
        L_angle = get_angle(lm("LEFT_SHOULDER"), lm("LEFT_ELBOW"), lm("LEFT_WRIST"))
        row["L_ELBOW_q"] = L_angle

        # 右肘角度
        R_angle = get_angle(lm("RIGHT_SHOULDER"), lm("RIGHT_ELBOW"), lm("RIGHT_WRIST"))
        row["R_ELBOW_q"] = R_angle

        # 其他字段留空（0.0），可以扩展添加 Pitch 等角度
    except Exception as e:
        print("角度计算失败，跳过帧", e)

    return row

def process_video(video_path, output_csv):
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    frame_time = 1.0 / fps
    base_time = time.time()

    data = []

    frame_idx = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(image_rgb)

        row_data = {"time": base_time + frame_idx * frame_time}

        if results.pose_landmarks:
            joint_data = extract_joint_angles(results.pose_landmarks.landmark)
            row_data.update(joint_data)
        else:
            for col in COLUMNS[1:]:
                row_data[col] = 0.0

        data.append(row_data)
        frame_idx += 1

    cap.release()
    pose.close()

    df = pd.DataFrame(data)
    df = df[COLUMNS]
    df.to_csv(output_csv, index=False)
    print(f"✅ CSV saved: {output_csv}")

# ========== 主程序 ==========
if __name__ == "__main__":
    process_video("videoplayback.mp4", "g1_joint_output.csv")