import cv2
import os
import numpy as np
from tqdm import tqdm

def safe_save_video_results(output_dir, video_out_path="romp_output.mp4", fps=30):
    frame_paths = sorted([
        os.path.join(output_dir, f)
        for f in os.listdir(output_dir)
        if f.endswith('.png')
    ])

    if not frame_paths:
        print("❌ No PNG frames found.")
        return

    # 读取第一帧尺寸
    first_frame = cv2.imread(frame_paths[0])
    height, width, _ = first_frame.shape
    print(f"🖼️  Video size: {width}x{height}, Total frames: {len(frame_paths)}")

    writer = cv2.VideoWriter(
        os.path.join(output_dir, video_out_path),
        cv2.VideoWriter_fourcc(*'mp4v'),
        fps,
        (width, height)
    )

    for path in tqdm(frame_paths, desc="🧵 Writing video"):
        frame = cv2.imread(path)
        if frame is None:
            print(f"[WARN] Skipped: {path}")
            continue
        writer.write(frame)

    writer.release()
    print(f"✅ Video saved to: {os.path.join(output_dir, video_out_path)}")