import os
import cv2
import numpy as np
from tqdm import tqdm

def to_scalar(v):
    if isinstance(v, np.ndarray):
        return float(v.item()) if v.size == 1 else float(v[0])
    return float(v)

# ------------------ 配置 ------------------
input_folder = "../externals/ROMP/output"  # 修改为你的 .npz 文件目录
output_video = "romp_visualized_output.avi"
fps = 25
scale = 1.5  # 用于放大坐标以适配视频画布
point_radius = 4
# ----------------------------------------

files = sorted([f for f in os.listdir(input_folder) if f.endswith(".npz")])
if not files:
    raise ValueError("目录中找不到 .npz 文件")

# 初始化最大宽高
max_x, max_y = 0, 0

for file in files:
    data = np.load(os.path.join(input_folder, file), allow_pickle=True)
    joints = data.get('pj2d_org', data.get('pj2d', None))
    if joints is not None and joints.size > 0:
        x_coords = joints[:, 0]
        y_coords = joints[:, 1]
        if not np.isnan(x_coords).all() and not np.isnan(y_coords).all():
            max_x = max(max_x, np.nanmax(x_coords))
            max_y = max(max_y, np.nanmax(y_coords))

# 如果完全失败（数据全空），则给一个默认尺寸
if max_x == 0 or max_y == 0:
    print("⚠️ 没有有效的关键点数据，使用默认画布大小 640x480")
    max_x, max_y = 640, 480

frame_w = int(max_x * scale)
frame_h = int(max_y * scale)
print(f"画布尺寸: {frame_w}x{frame_h}")

fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_writer = cv2.VideoWriter(output_video, fourcc, fps, (frame_w, frame_h))

for idx, file in enumerate(tqdm(files)):
    data = np.load(os.path.join(input_folder, file), allow_pickle=True)
    joints = data.get('pj2d_org', data.get('pj2d', None))

    frame = np.zeros((frame_h, frame_w, 3), dtype=np.uint8)

    if frame.size == 0:
        print(f"⚠️ 跳过空画布 {file}")
        continue

    if joints is not None:
        for j in joints:
            try:
                x, y = int(to_scalar(j[0]) * scale), int(to_scalar(j[1]) * scale)
                if 0 <= x < frame_w and 0 <= y < frame_h:
                    cv2.circle(frame, (x, y), point_radius, (0, 255, 0), -1)
            except Exception as e:
                print(f"{file} Failed to draw joint: {e}")
    else:
        print(f"{file} contains no valid joints")

    # 添加帧编号
    cv2.putText(frame, f"Frame: {idx}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    video_writer.write(frame)

video_writer.release()
print(f"✅ 可视化视频已保存为：{output_video}")