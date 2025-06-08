在 Ubuntu 20.04 上安装 ROS2 Foxy（推荐版本）步骤如下：

✅ 步骤 1：设置软件源
sudo apt update && sudo apt install curl gnupg lsb-release -y

添加 ROS2 公钥：
sudo mkdir -p /etc/apt/keyrings

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
| gpg --dearmor \
| sudo tee /etc/apt/keyrings/ros-archive-keyring.gpg > /dev/null

添加 Foxy 软件源：
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list

✅ 步骤 2：更新并安装 ROS2 Foxy
sudo apt update
sudo apt install ros-foxy-desktop -y

✅ 步骤 3：配置环境变量
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

✅ 步骤 4：安装依赖管理工具（如 rosdep）
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update

✅ 测试
ros2 --help