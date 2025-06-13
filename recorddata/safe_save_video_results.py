import os
import cv2
import argparse
from tqdm import tqdm

def safe_save_video_results(input_dir, output_path, fps=30):
    frame_paths = sorted([
        os.path.join(input_dir, f)
        for f in os.listdir(input_dir)
        if f.lower().endswith((".png", ".jpg"))
    ])

    if not frame_paths:
        print("❌ No image frames found in", input_dir)
        return

    # 读取第一帧判断大小
    first_frame = cv2.imread(frame_paths[0])
    if first_frame is None:
        print(f"❌ Cannot read first frame: {frame_paths[0]}")
        return
    height, width, _ = first_frame.shape
    print(f"📐 Video size: {width}x{height}, Total frames: {len(frame_paths)}")

    # 创建视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    for path in tqdm(frame_paths, desc="🧵 Writing video"):
        frame = cv2.imread(path)
        if frame is None:
            print(f"[WARN] Skipping unreadable frame: {path}")
            continue
        out.write(frame)

    out.release()
    print(f"✅ Video saved to: {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Safely convert image sequence to video.")
    parser.add_argument('--input', '-i', required=True, help='Input directory containing PNG frames')
    parser.add_argument('--output', '-o', required=True, help='Output video file path (e.g., output.mp4)')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second')
    args = parser.parse_args()

    safe_save_video_results(args.input, args.output, args.fps)