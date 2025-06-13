import os
import numpy as np
import cv2
from tqdm import tqdm

ROMP_OUTPUT_DIR = "../externals/ROMP/output"
npz_files = sorted(f for f in os.listdir(ROMP_OUTPUT_DIR) if f.endswith(".npz"))

for fname in tqdm(npz_files):
    path = os.path.join(ROMP_OUTPUT_DIR, fname)
    data = np.load(path, allow_pickle=True)["results"].item()

    # 尝试读取 projected 2D joints
    joints_2d = data.get("pj2d_org")  # shape: (N, 2) or (N, 3)
    img = np.zeros((720, 1280, 3), dtype=np.uint8)

    if joints_2d is not None:
        if joints_2d.shape[1] == 3:
            joints_2d = joints_2d[:, :2]  # 丢弃 z

        # 调试坐标范围
        print(f"{fname} | min: {joints_2d.min()}, max: {joints_2d.max()}")

        # 如果值非常小（0.0~1.0），说明需要放大到像素坐标
        if joints_2d.max() < 10:
            joints_2d[:, 0] *= 1280  # x
            joints_2d[:, 1] *= 720   # y

        for joint in joints_2d:
            try:
                x, y = int(joint[0]), int(joint[1])
                if 0 <= x < 1280 and 0 <= y < 720:
                    cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
            except Exception as e:
                print(f"Failed to draw joint: {e}")
                continue

    cv2.putText(img, fname, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imshow("ROMP 2D Pose Preview", img)
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()