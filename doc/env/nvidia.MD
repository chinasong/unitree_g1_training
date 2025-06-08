✅ 检查是否有 GPU：
lspci | grep -i nvidia

✅ 如果你有 GPU，但没有 nvidia-smi：
sudo apt update
sudo apt install nvidia-driver-525
sudo reboot
（驱动版本号根据你卡的型号而定）

然后重启后再次测试：
nvidia-smi

✅ 你也可以用以下命令确认 CUDA 是否可用（在 Python 中）：
import torch
print(torch.cuda.is_available())  # True = 有 GPU 可用

重启系统
sudo reboot

之后重新运行：
nvidia-smi

✅ 2. 检查驱动模块是否加载成功
lsmod | grep nvidia
如果没有输出，说明内核没加载正确的 NVIDIA 驱动。

✅ 3. 检查加载的驱动版本 vs 用户态版本
cat /proc/driver/nvidia/version
这会告诉你当前内核使用的 NVIDIA 驱动版本。

再执行：
nvidia-smi
你看到的 “NVML library version” 应该与上面一致，如果不一致 → 就是版本冲突。