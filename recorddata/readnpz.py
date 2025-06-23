import numpy as np

# 你的 .npz 文件路径
npz_path = '00000001.npz'

# 加载 .npz 文件
data = np.load(npz_path, allow_pickle=True)

# 查看所有键名
print("Keys in .npz:", data.files)

# 取出结果数据（通常为 'results'）
results = data['results'].item()  # 注意是 .item()，因为是对象类型

# 打印部分关键内容
for k in results:
    print(f"{k}: {type(results[k])} shape={np.shape(results[k])}")