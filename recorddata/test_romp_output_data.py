import os
import numpy as np
import cv2
from tqdm import tqdm

ROMP_OUTPUT_DIR = "../externals/ROMP/output"  # 改成你的npz目录

npz_files = sorted(f for f in os.listdir(ROMP_OUTPUT_DIR) if f.endswith(".npz"))

for fname in tqdm(npz_files):
    path = os.path.join(ROMP_OUTPUT_DIR, fname)
    data = np.load(path, allow_pickle=True)["results"].item()

    joints = data.get("joints", None)
    img = np.zeros((720, 1280, 3), dtype=np.uint8)

    if joints is not None:
        for joint in joints:
            x, y = int(joint[0]), int(joint[1])
            if 0 <= x < 1280 and 0 <= y < 720:
                cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

    cv2.imshow("ROMP pose", img)
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()