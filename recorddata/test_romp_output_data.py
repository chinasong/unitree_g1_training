import numpy as np

path = "../externals/ROMP/output/00000123.npz"  # 替换为任意一帧文件路径
data = np.load(path, allow_pickle=True)['results'].item()

print("Keys:", list(data.keys()))
print("global_orient shape:", data['global_orient'].shape)
print("body_pose shape:", data['body_pose'].shape)
print("joints shape:", data['joints'].shape)
print("verts shape:", data['verts'].shape)