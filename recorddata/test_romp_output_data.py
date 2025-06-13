import os
import cv2
import numpy as np
from tqdm import tqdm

def to_scalar(v):
    if isinstance(v, np.ndarray):
        return float(v.item()) if v.size == 1 else float(v[0])
    return float(v)

input_folder = "../externals/ROMP/output"   # 你的 .npz 文件所在目录
output_video_path = "output.avi"
frame_size = (1280, 720)  # 如果分辨率不对可改
fps = 30

# 视频写入器
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_writer = cv2.VideoWriter(output_video_path, fourcc, fps, frame_size)

files = sorted([f for f in os.listdir(input_folder) if f.endswith(".npz")])
print(f"Total frames: {len(files)}")

for file in tqdm(files):
    path = os.path.join(input_folder, file)
    data = np.load(path, allow_pickle=True)

    # 创建空图像用于绘制
    img = np.zeros((720, 1280, 3), dtype=np.uint8)

    if 'pj2d_org' in data:
        joints_2d = data['pj2d_org']
        for joint in joints_2d:
            try:
                x, y = int(to_scalar(joint[0])), int(to_scalar(joint[1]))
                if 0 <= x < frame_size[0] and 0 <= y < frame_size[1]:
                    cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
            except Exception as e:
                print(f"{file} Failed to draw joint: {e}")
    else:
        print(f"{file} missing pj2d_org")

    video_writer.write(img)

video_writer.release()
print("✅ 视频生成完成：", output_video_path)