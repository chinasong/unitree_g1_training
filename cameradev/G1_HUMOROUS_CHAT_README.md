# G1机器人智能幽默聊天系统

## 功能概述

这是一个为G1机器人开发的智能聊天系统，能够：
- 🎥 **自动检测人**：使用YOLO模型实时检测摄像头视野中的人
- 👀 **分析人物特征**：分析人的衣着颜色、位置、大小等特征
- 🎤 **语音识别**（可选）：聆听人说的话
- 🤖 **智能回复**：根据看到的人和听到的话，生成幽默风趣的回复
- 💬 **主动聊天**：机器人会主动和人聊天，营造轻松愉快的氛围

## 系统要求

### 必需依赖
- Python 3.6+
- ROS (Robot Operating System)
- G1机器人通过局域网连接
- unitree_sdk2_python（已包含在项目中）

### 可选依赖（用于增强功能）
- **语音识别**：`speechrecognition` 和 `pyaudio`
- **AI回复**：`openai`（需要OpenAI API密钥）

## 安装步骤

### 1. 安装Python依赖

```bash
# 安装基础依赖
pip3 install opencv-python ultralytics rospy cv-bridge

# 可选：安装语音识别功能
pip3 install speechrecognition pyaudio

# 可选：安装OpenAI API（用于AI生成回复）
pip3 install openai
```

**注意**：`pyaudio`可能需要系统级别的依赖：
- Ubuntu/Debian: `sudo apt-get install portaudio19-dev python3-pyaudio`
- macOS: `brew install portaudio`

### 2. 配置AI API（DeepSeek）

系统支持使用**DeepSeek API**生成智能回复。需要设置环境变量：

```bash
export DEEPSEEK_API_KEY="your-deepseek-api-key-here"
```

如果你想使用OpenAI API作为备用，可以设置环境变量：

```bash
export OPENAI_API_KEY="your-openai-api-key-here"
```

**注意**：
- 系统会优先使用DeepSeek API（如果设置了`DEEPSEEK_API_KEY`）
- 如果DeepSeek失败且配置了OpenAI API，会自动切换到OpenAI
- 如果都没有配置或都失败，会使用内置的简单回复生成器

### 3. 确保YOLO模型文件存在

确保`cameradev/yolo11n.pt`文件存在。如果不存在，YOLO会自动下载。

### 4. 确保标签映射文件存在

确保`cameradev/en_to_zh.js`文件存在（用于中英文标签映射）。

## 使用方法

### 基本使用

```bash
cd cameradev
python3 g1_humorous_chat.py <网络接口名称>
```

**示例**：
```bash
python3 g1_humorous_chat.py eth0
```

### 查找网络接口名称

```bash
# Linux
ifconfig
# 或
ip addr

# macOS
ifconfig
```

常见的网络接口名称：
- `eth0` - 以太网接口
- `enp3s0` - 另一个常见的以太网接口
- `wlan0` - WiFi接口

### 运行要求

1. **ROS环境**：确保ROS环境已正确配置，并且G1机器人的摄像头话题正在发布
   ```bash
   # 检查摄像头话题是否可用
   rostopic list | grep camera
   ```

2. **网络连接**：确保你的电脑和G1机器人在同一网络中

3. **摄像头权限**：如果使用语音识别，确保有麦克风权限

## 功能说明

### 1. 人物检测
- 系统使用YOLO模型实时检测视野中的人
- 检测到人后，会等待2秒再开始聊天（可配置）

### 2. 特征分析
系统会分析以下特征：
- **主要颜色**：检测人物区域的主要颜色（红、蓝、绿等）
- **亮度**：判断衣着是明亮、中等还是深色
- **大小**：判断人在画面中的大小（很大、中等、较小）
- **位置**：判断人在画面中的位置（左侧、中间、右侧）

### 3. 语音识别（可选）
- 如果安装了`speechrecognition`，系统会在后台持续监听语音
- 识别到语音后，会结合视觉特征生成更相关的回复

### 4. 回复生成

#### 使用DeepSeek API（默认）
系统默认使用**DeepSeek Chat**模型生成智能回复：
- 根据人物特征生成个性化回复
- 结合对话历史生成连贯的回复
- 风格幽默风趣但不失礼貌
- 使用deepseek-chat模型，性能优秀且成本较低

#### 使用OpenAI API（备用）
如果DeepSeek API失败且设置了`OPENAI_API_KEY`，系统会自动切换到OpenAI GPT-3.5-turbo。

#### 使用内置回复生成器（备用）
如果没有OpenAI API，系统会使用内置的规则生成回复：
- 基于颜色、位置、大小等特征生成简单回复
- 包含一些预设的幽默回复模板

### 5. 对话管理
- 系统会记录最近的对话历史（最多10条）
- 两次聊天之间有5秒的冷却时间（可配置）
- 人离开视野3秒后，系统会重置状态

## 配置参数

可以在代码中修改以下参数：

```python
PERSON_DETECT_INTERVAL = 2.0  # 检测到人后多久开始聊天（秒）
CHAT_COOLDOWN = 5.0  # 两次聊天之间的冷却时间（秒）
PERSON_DISAPPEAR_TIMEOUT = 3.0  # 人消失多久后重置状态（秒）
```

## 故障排查

### 1. 无法检测到人
- 检查摄像头是否正常工作：`rostopic echo /camera/color/image_raw`
- 确保光线充足
- 检查YOLO模型是否正确加载

### 2. 机器人不说话
- 检查网络连接
- 检查音频客户端是否初始化成功
- 查看控制台是否有错误信息

### 3. 语音识别不工作
- 检查是否安装了`speechrecognition`和`pyaudio`
- 检查麦克风权限
- 检查是否有可用的麦克风设备

### 4. DeepSeek API不工作
- 检查网络连接（需要能访问api.deepseek.com）
- 查看控制台错误信息
- 如果DeepSeek失败，系统会自动尝试OpenAI API（如果配置了）
- 如果都失败，系统会使用内置的简单回复生成器

## 示例对话

**场景1：检测到穿红色衣服的人**
- 机器人："哇，你穿得这么红，是要去参加什么重要场合吗？"

**场景2：检测到人在左侧**
- 机器人："你站在左边，是想让我多看看你吗？"

**场景3：听到人说"你好"**
- 机器人："你好！很高兴见到你！"

**场景4：使用AI生成（结合多个特征）**
- 机器人："你穿蓝色衣服站在中间，看起来很有活力！想和我聊聊吗？"

## 技术架构

- **视觉检测**：YOLO v11 (ultralytics)
- **特征分析**：OpenCV图像处理
- **语音识别**：Google Speech Recognition API（通过speechrecognition库）
- **回复生成**：DeepSeek Chat（默认）或 OpenAI GPT-3.5-turbo（备用）或 内置规则引擎
- **机器人控制**：unitree_sdk2_python
- **通信**：ROS (Robot Operating System)

## 开发建议

1. **自定义回复**：可以在`generate_humorous_response`函数中添加更多回复模板
2. **增强特征分析**：可以使用更高级的视觉模型（如GPT-4V）来分析人物特征
3. **本地LLM**：可以使用本地LLM（如Ollama）替代OpenAI API
4. **多语言支持**：可以扩展支持更多语言

## 许可证

本项目遵循项目主许可证。

## 贡献

欢迎提交Issue和Pull Request！

