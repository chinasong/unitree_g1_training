在 Ubuntu 20.04 上安装 ROS1 Noetic 是最推荐的组合，因为 Noetic 是 ROS1 的最后一个 LTS（长期支持）版本，官方专为 Ubuntu 20.04 提供原生支持。

✅ 一键安装 ROS1 Noetic（推荐 Desktop-Full）
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

✅ 第 2 步：添加 ROS 公钥
sudo apt update
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

✅ 第 3 步：安装 ROS Noetic
安装完整桌面版本（含 RViz、Gazebo、导航、SLAM 等）：
sudo apt update
sudo apt install ros-noetic-desktop-full -y

如果你只需要基础功能，可以选择：
sudo apt install ros-noetic-ros-base

✅ 第 4 步：初始化 rosdep（依赖管理器）
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update

✅ 第 5 步：配置环境变量
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

✅ 测试安装是否成功
roscore

另开一个终端：
rosrun turtlesim turtlesim_node
如果你能看到小乌龟窗口，就说明安装成功。

没有正确添加 ROS 软件源 或 APT 索引未更新成功。
🔹 1. 检查系统版本（必须是 Ubuntu 20.04）
lsb_release -a

确认输出包含：
Distributor ID: Ubuntu
Release:        20.04

🔹 2. 添加 ROS 软件源（官方源）
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
注意：Ubuntu 20.04 的代号是 focal

🔹 3. 添加公钥（非常重要）
sudo apt update
sudo apt install curl gnupg -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

你应该看到输出：
OK

🔹 4. 更新 apt 索引
sudo apt update

确认没有报错，比如：
Hit:1 http://packages.ros.org/ros/ubuntu focal InRelease

🔹 5. 再次尝试安装 ROS Noetic
sudo apt install ros-noetic-desktop-full -y


ROS2 软件源公钥过期的问题
✅ 解决方法：更新过期的 ROS GPG 密钥
请执行以下命令：
sudo apt-key del F42ED6FBAB17C654
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

然后更新源：
sudo apt update

✅ 接下来可以继续：
安装 ROS1 Noetic：
sudo apt install ros-noetic-desktop-full -y

✅ 验证是否成功修复
你可以重新运行：
sudo apt update

✅ 一步步彻底修复方式（100%可解决）

我们将清除旧公钥 → 正确添加新的公钥 → 用 signed-by 显式绑定。

⸻

🔧 步骤 1：彻底删除旧公钥和软件源配置
sudo apt-key del F42ED6FBAB17C654 || true
sudo rm -f /etc/apt/sources.list.d/ros-latest.list
sudo rm -f /etc/apt/sources.list.d/ros2.list

🔧 步骤 2：使用新版 GPG 公钥添加 ROS1+ROS2 安全源
sudo mkdir -p /etc/apt/keyrings

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
| gpg --dearmor \
| sudo tee /etc/apt/keyrings/ros-archive-keyring.gpg > /dev/null

🔧 步骤 3：添加 ROS1 和 ROS2 的源配置（绑定新 keyring）
# ROS1 Noetic
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
| sudo tee /etc/apt/sources.list.d/ros1.list

# ROS2（可选）
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
| sudo tee /etc/apt/sources.list.d/ros2.list

🔧 步骤 4：更新 APT 缓存
sudo apt update

✅ 成功后可以执行：
sudo apt install ros-noetic-desktop-full

