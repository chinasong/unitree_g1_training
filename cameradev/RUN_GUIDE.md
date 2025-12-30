# G1智能幽默聊天系统 - 运行指南

## 📋 快速开始

### 方式1：使用启动脚本（推荐）

```bash
cd cameradev
chmod +x start_humorous_chat.sh
./start_humorous_chat.sh eth0
```

### 方式2：直接运行Python脚本

```bash
cd cameradev
python3 g1_humorous_chat.py eth0
```

**参数说明**：
- `eth0` 是网络接口名称，需要替换为你的实际网络接口

---

## 🔧 前置条件

### 1. 系统要求

- **操作系统**：Ubuntu 18.04+ 或 macOS
- **Python版本**：Python 3.6+
- **ROS环境**：ROS Noetic 或 ROS Melodic
- **网络连接**：G1机器人需在同一局域网

### 2. 查找网络接口名称

```bash
# Linux
ifconfig
# 或
ip addr show

# macOS
ifconfig
```

常见接口名称：
- `eth0` - 以太网接口（最常见）
- `enp3s0` - 另一个以太网接口
- `wlan0` - WiFi接口
- `en0` - macOS以太网接口

---

## 📦 安装步骤

### 步骤1：安装Python依赖

```bash
# 必需依赖
pip3 install opencv-python ultralytics rospy cv-bridge numpy

# 可选：语音识别功能
pip3 install speechrecognition pyaudio

# 可选：AI回复功能
pip3 install openai
```

**注意**：`pyaudio`可能需要系统依赖：
- Ubuntu: `sudo apt-get install portaudio19-dev python3-pyaudio`
- macOS: `brew install portaudio`

### 步骤2：配置ROS环境

```bash
# Ubuntu (ROS Noetic)
source /opt/ros/noetic/setup.bash

# Ubuntu (ROS Melodic)
source /opt/ros/melodic/setup.bash
```

### 步骤3：配置Unitree SDK

**方式A：使用本地externals文件夹（如果存在）**

```bash
# 初始化git submodule
git submodule update --init --recursive

# 安装SDK
cd externals/unitree_sdk2_python
pip3 install -e .
cd ../..
```

**方式B：使用已安装的pip包**

```bash
# 如果SDK已通过pip安装，无需额外操作
# 代码会自动检测并使用已安装的包
```

### 步骤4：配置DeepSeek API（可选但推荐）

```bash
# 临时设置（当前终端会话）
export DEEPSEEK_API_KEY="your-api-key-here"

# 永久设置（推荐）
echo 'export DEEPSEEK_API_KEY="your-api-key-here"' >> ~/.bashrc
source ~/.bashrc
```

**验证配置**：
```bash
echo $DEEPSEEK_API_KEY
```

### 步骤5：准备资源文件

确保以下文件存在于 `cameradev/` 目录：

- ✅ `en_to_zh.js` - 标签映射文件（必需）
- ✅ `yolo11n.pt` - YOLO模型文件（可选，会自动下载）

---

## 🚀 运行方法

### 方法1：使用启动脚本（最简单）

```bash
cd cameradev
./start_humorous_chat.sh eth0
```

启动脚本会自动：
- ✅ 检查Python环境
- ✅ 检查ROS环境
- ✅ 检查依赖包
- ✅ 检查环境变量
- ✅ 检查资源文件
- ✅ 启动程序

### 方法2：直接运行Python脚本

```bash
cd cameradev

# 确保ROS环境已配置
source /opt/ros/noetic/setup.bash  # 或 melodic

# 运行程序
python3 g1_humorous_chat.py eth0
```

### 方法3：在后台运行

```bash
cd cameradev
nohup python3 g1_humorous_chat.py eth0 > chat.log 2>&1 &
```

查看日志：
```bash
tail -f chat.log
```

---

## 📝 运行参数

### 必需参数

- **网络接口名称**：连接到G1机器人的网络接口
  - 示例：`eth0`, `enp3s0`, `wlan0`

### 命令行格式

```bash
python3 g1_humorous_chat.py <网络接口名称>
```

**示例**：
```bash
python3 g1_humorous_chat.py eth0
python3 g1_humorous_chat.py enp3s0
python3 g1_humorous_chat.py wlan0
```

---

## ✅ 运行前检查清单

在运行前，请确认：

- [ ] Python 3.6+ 已安装
- [ ] ROS环境已配置并source
- [ ] 必需的Python包已安装（opencv-python, ultralytics, rospy, cv-bridge）
- [ ] Unitree SDK已安装或externals文件夹存在
- [ ] G1机器人已连接并可通过网络访问
- [ ] 网络接口名称正确（通过ifconfig确认）
- [ ] ROS摄像头话题 `/camera/color/image_raw` 正在发布
- [ ] `en_to_zh.js` 文件存在于cameradev目录
- [ ] （可选）DEEPSEEK_API_KEY环境变量已设置
- [ ] （可选）语音识别包已安装（如果需要语音功能）

---

## 🔍 验证ROS摄像头话题

在运行程序前，确保摄像头话题可用：

```bash
# 检查话题是否存在
rostopic list | grep camera

# 查看话题内容（会显示图像数据）
rostopic echo /camera/color/image_raw --noarr
```

如果话题不存在，需要先启动G1机器人的摄像头节点。

---

## 🎯 运行示例

### 完整运行流程

```bash
# 1. 进入项目目录
cd /path/to/unitree_g1_training

# 2. 配置ROS环境
source /opt/ros/noetic/setup.bash

# 3. 设置API密钥（如果使用AI回复）
export DEEPSEEK_API_KEY="your-key-here"

# 4. 进入cameradev目录
cd cameradev

# 5. 运行程序
python3 g1_humorous_chat.py eth0
```

### 预期输出

```
📦 正在加载YOLO模型...
🔊 正在初始化音频客户端...
✅ G1机器人连接成功！
🎤 启动语音监听线程...（如果安装了speech_recognition）
📡 正在初始化ROS节点...
✅ G1智能幽默聊天系统已启动！
📋 功能说明：
   - 自动检测视野中的人
   - 分析人的特征（衣着、外观等）
   - 聆听人的语音
   - 使用DeepSeek AI生成幽默回复 (模型: deepseek-chat)
   - 机器人主动聊天

按 Ctrl+C 退出
```

---

## 🛠️ 故障排查

### 问题1：找不到网络接口

**错误**：`Network interface not found`

**解决**：
```bash
# 查看所有网络接口
ifconfig
# 或
ip addr show

# 使用正确的接口名称
python3 g1_humorous_chat.py <正确的接口名>
```

### 问题2：无法导入unitree_sdk2py

**错误**：`ModuleNotFoundError: No module named 'unitree_sdk2py'`

**解决**：
```bash
# 方式1：初始化submodule并安装
git submodule update --init --recursive
cd externals/unitree_sdk2_python
pip3 install -e .
cd ../..

# 方式2：从GitHub安装
pip3 install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
```

### 问题3：ROS话题不存在

**错误**：程序运行但没有检测到人

**解决**：
```bash
# 检查话题
rostopic list | grep camera

# 如果没有话题，需要启动G1的摄像头节点
# 通常通过G1的ROS启动文件启动
```

### 问题4：无法连接G1机器人

**错误**：`Timeout` 或连接失败

**解决**：
1. 检查网络连接：`ping <G1机器人IP>`
2. 确认网络接口正确
3. 检查防火墙设置
4. 确认G1机器人已开机并联网

### 问题5：YOLO模型下载失败

**错误**：模型下载超时

**解决**：
```bash
# 手动下载模型
cd cameradev
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.pt
```

### 问题6：语音识别不工作

**错误**：`⚠️ 未安装speech_recognition`

**解决**：
```bash
# 安装语音识别包
pip3 install speechrecognition pyaudio

# Ubuntu还需要系统依赖
sudo apt-get install portaudio19-dev python3-pyaudio
```

---

## 📊 运行状态说明

程序运行时会显示以下状态：

- `📦 正在加载YOLO模型...` - 加载目标检测模型
- `🔊 正在初始化音频客户端...` - 连接G1机器人
- `✅ G1机器人连接成功！` - 连接成功
- `🎤 启动语音监听线程...` - 语音识别已启用
- `📡 正在初始化ROS节点...` - 初始化ROS通信
- `👤 检测到人，特征: {...}` - 检测到人并分析特征
- `🤖 机器人说: ...` - 机器人正在说话
- `👋 人已离开视野` - 人离开摄像头视野

---

## 🎮 使用技巧

### 1. 调整聊天频率

编辑 `g1_humorous_chat.py` 中的参数：

```python
PERSON_DETECT_INTERVAL = 2.0  # 检测到人后多久开始聊天（秒）
CHAT_COOLDOWN = 5.0  # 两次聊天之间的冷却时间（秒）
```

### 2. 查看详细日志

程序会在控制台输出详细日志，包括：
- 检测到的人的特征
- 生成的回复内容
- 错误信息

### 3. 停止程序

按 `Ctrl+C` 优雅退出程序。

---

## 📚 相关文档

- [完整功能文档](G1_HUMOROUS_CHAT_README.md)
- [环境变量配置指南](env.example)
- [项目主README](../README.md)

---

## 💡 提示

1. **首次运行**：YOLO模型会自动下载，可能需要一些时间
2. **网络要求**：确保与G1机器人在同一局域网
3. **性能优化**：如果检测较慢，可以降低摄像头分辨率
4. **API密钥**：建议使用环境变量而不是硬编码在代码中

---

## 🆘 获取帮助

如果遇到问题：

1. 查看本文档的故障排查部分
2. 检查控制台输出的错误信息
3. 确认所有依赖都已正确安装
4. 验证ROS环境和网络连接

---

**祝使用愉快！** 🎉

