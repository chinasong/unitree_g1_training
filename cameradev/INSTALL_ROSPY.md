# 安装rospy模块指南

## 问题
运行 `g1_humorous_chat.py` 时出现错误：
```
ModuleNotFoundError: No module named 'rospy'
```

## 重要说明

根据 [G1摄像头官方文档](https://support.unitree.com/home/zh/G1_developer/depth_camera_instruction)，G1机器人的摄像头数据通过ROS话题发布。因此：

1. **必须安装ROS**：`rospy` 是ROS的核心Python库，不能单独安装
2. **推荐环境**：Ubuntu Linux + ROS Noetic/Melodic（G1官方支持的环境）
3. **macOS用户**：需要使用Docker或虚拟机运行ROS环境

## 解决方案

### 方案1：在Ubuntu Linux上安装ROS（最佳方案，强烈推荐）

**G1机器人官方推荐在Ubuntu Linux上运行**，因为：
- ROS在Linux上支持最完善
- G1的SDK和驱动主要针对Linux开发
- 摄像头数据通过ROS话题发布，需要完整的ROS环境

#### Ubuntu 20.04 (ROS Noetic) - 推荐

```bash
# 1. 设置ROS源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. 添加密钥
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 3. 更新包列表
sudo apt update

# 4. 安装ROS Noetic完整版
sudo apt install -y ros-noetic-desktop-full

# 5. 初始化rosdep
sudo rosdep init
rosdep update

# 6. 配置环境变量
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 7. 安装Python依赖
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rospy python3-cv-bridge python3-rospkg

# 8. 验证安装
python3 -c "import rospy; print('✅ rospy installed successfully')"
```

#### Ubuntu 18.04 (ROS Melodic)

```bash
# 类似步骤，将 noetic 替换为 melodic
sudo apt install -y ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

### 方案2：在macOS上使用Docker运行ROS（macOS用户推荐）

macOS上安装ROS比较复杂，推荐使用Docker：

#### 使用Docker运行ROS（最简单）

```bash
# 1. 安装Docker Desktop for Mac
# 下载地址: https://www.docker.com/products/docker-desktop

# 2. 拉取ROS镜像
docker pull osrf/ros:noetic-desktop-full

# 3. 运行ROS容器
docker run -it --rm \
  -v $(pwd):/workspace \
  -e DISPLAY=$DISPLAY \
  osrf/ros:noetic-desktop-full \
  bash

# 4. 在容器内安装依赖
apt-get update
apt-get install -y python3-rospy python3-cv-bridge

# 5. 运行程序
cd /workspace/cameradev
python3 g1_humorous_chat.py eth0
```

#### 使用Homebrew安装ROS（较复杂）

```bash
# 1. 安装Homebrew（如果还没有）
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# 2. 添加ROS tap
brew tap ros/deps

# 3. 安装ROS（注意：macOS上ROS支持有限）
# 可能需要使用虚拟机或Docker
```

### 方案3：在macOS上使用虚拟机（备选方案）

如果Docker不适合，可以使用Ubuntu虚拟机：

1. 安装VirtualBox或VMware
2. 创建Ubuntu 20.04虚拟机
3. 在虚拟机中安装ROS Noetic
4. 在虚拟机中运行程序

### 方案4：只安装rospy Python包（不推荐，通常无法工作）

```bash
# ⚠️ 警告：这种方法通常无法完全工作，因为rospy依赖ROS核心库
pip3 install rospkg catkin_pkg
pip3 install --upgrade pip setuptools

# 尝试安装rospy（通常会失败或功能不完整）
pip3 install rospy
```

**注意**：这种方法通常无法完全工作，因为：
- `rospy` 需要ROS的核心库（roscore）
- G1摄像头数据通过ROS话题发布，需要完整的ROS环境
- 缺少ROS消息类型定义（如 `sensor_msgs.msg.Image`）

如果可能，建议在Ubuntu Linux系统上运行：

```bash
# Ubuntu 20.04 (ROS Noetic)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安装Python依赖
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rospy python3-cv-bridge
```

### 方案4：使用修改版代码（不依赖ROS）

如果无法安装ROS，可以考虑修改代码使用OpenCV直接读取摄像头，而不是通过ROS话题。

---

## 推荐方案

**对于Ubuntu Linux用户**（强烈推荐）：
1. ✅ **直接安装ROS Noetic/Melodic** - 这是G1官方支持的环境
2. 按照上面的"方案1"步骤安装

**对于macOS用户**：
1. **最佳**：使用Docker运行ROS环境（见方案2）
2. **次选**：在Ubuntu虚拟机中运行（见方案3）
3. ⚠️ **不推荐**：尝试单独安装rospy（通常无法工作）

**重要提示**：
- G1摄像头数据通过ROS话题 `/camera/color/image_raw` 发布
- 需要完整的ROS环境（roscore）才能接收摄像头数据
- 仅安装Python包无法获取G1的摄像头数据

---

## 验证安装

安装后验证：

```bash
# 1. 检查ROS环境
echo $ROS_DISTRO
# 应该输出: noetic 或 melodic

# 2. 检查rospy
python3 -c "import rospy; print('✅ rospy installed successfully')"

# 3. 检查cv_bridge
python3 -c "from cv_bridge import CvBridge; print('✅ cv_bridge installed successfully')"

# 4. 检查ROS消息类型
python3 -c "from sensor_msgs.msg import Image; print('✅ sensor_msgs installed successfully')"

# 5. 检查G1摄像头话题（需要G1机器人连接并运行）
rostopic list | grep camera
# 应该能看到: /camera/color/image_raw 等话题
```

## G1摄像头话题说明

根据 [G1摄像头官方文档](https://support.unitree.com/home/zh/G1_developer/depth_camera_instruction)，G1机器人会发布以下ROS话题：

- `/camera/color/image_raw` - RGB彩色图像（程序使用的话题）
- `/camera/depth/image_raw` - 深度图像
- 其他相关话题...

**重要**：确保G1机器人已连接并启动了摄像头节点，否则话题不会存在。

---

## 快速测试（如果已安装ROS）

```bash
# 1. Source ROS环境
source /opt/ros/noetic/setup.bash  # 或 melodic

# 2. 测试导入
python3 -c "import rospy; print('✅ OK')"

# 3. 检查G1摄像头话题（需要G1已连接）
rostopic list | grep camera

# 4. 运行程序
cd cameradev
python3 g1_humorous_chat.py eth0
```

## 相关资源

- [G1摄像头官方文档](https://support.unitree.com/home/zh/G1_developer/depth_camera_instruction)
- [ROS官方安装指南](http://wiki.ros.org/ROS/Installation)
- [ROS Noetic安装指南](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [ROS Melodic安装指南](http://wiki.ros.org/melodic/Installation/Ubuntu)

