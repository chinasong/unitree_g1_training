import os
import numpy as np
import cv2
from tqdm import tqdm

ROMP_OUTPUT_DIR = "../externals/ROMP/output"  # 修改为你的路径
npz_files = sorted(f for f in os.listdir(ROMP_OUTPUT_DIR) if f.endswith(".npz"))

for fname in tqdm(npz_files):
    path = os.path.join(ROMP_OUTPUT_DIR, fname)
    data = np.load(path, allow_pickle=True)["results"].item()

    # 使用 pj2d_org 作为 2D 关键点
    joints_2d = data.get("pj2d_org", None)  # shape: (N, 2)
    img = np.zeros((720, 1280, 3), dtype=np.uint8)

    if joints_2d is not None and len(joints_2d.shape) == 2:
        for joint in joints_2d:
            try:
                x, y = int(joint[0]), int(joint[1])
                if 0 <= x < 1280 and 0 <= y < 720:
                    cv2.circle(img, (x, y), 4, (0, 255, 0), -1)
            except Exception as e:
                print(f"Failed to draw joint: {e}")
                continue

    cv2.putText(img, fname, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imshow("ROMP 2D Pose Preview", img)
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()