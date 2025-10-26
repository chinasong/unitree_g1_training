# G1 机器人文字转语音服务

通过本地网络发送文字让G1机器人说话的完整解决方案。

## 功能特性

✨ **主要功能**：
- 🗣️ **文字转语音（TTS）**: 支持中英文混合语音合成
- 🔊 **音量控制**: 实时调节机器人语音音量 (0-100)
- 💡 **LED灯控制**: RGB全彩灯带控制
- 🌐 **HTTP API**: 通过网络接口控制机器人
- 🖥️ **Web界面**: 浏览器可视化控制面板
- 💻 **交互模式**: 命令行实时交互
- 📡 **跨平台**: 支持任何能发送HTTP请求的设备

## 系统要求

- Python 3.6+
- G1机器人通过局域网连接
- unitree_sdk2_python（已包含在项目中）
- requests库（用于客户端示例）

## 快速开始

### 1. 安装依赖

```bash
# 安装requests库（用于客户端）
pip3 install requests

# 确保unitree_sdk2_python已安装
cd externals/unitree_sdk2_python
pip3 install -e .
```

### 2. 启动TTS服务

```bash
# 方式1: 启动HTTP服务器（推荐）
python3 g1_tts_service.py --iface eth0 --server

# 方式2: 交互式命令行模式
python3 g1_tts_service.py --iface eth0 --interactive

# 方式3: 直接让机器人说一句话
python3 g1_tts_service.py --iface eth0 --speak "你好，我是G1机器人"
```

**参数说明**：
- `--iface`: 连接到G1的网络接口名称（必需）
  - 常见接口: `eth0`, `enp3s0`, `wlan0`
  - 查看接口: `ifconfig` 或 `ip addr`
- `--server`: 启动HTTP服务器模式
- `--interactive`: 启动交互式命令行模式
- `--speak`: 直接说一句话
- `--port`: HTTP服务器端口（默认8080）
- `--host`: HTTP服务器监听地址（默认0.0.0.0）
- `--volume`: 设置初始音量（0-100）

### 3. 使用服务

#### 方式1: 浏览器Web界面

服务启动后，在浏览器打开：
```
http://localhost:8080
```

你会看到一个可视化控制面板，可以：
- 输入文字让机器人说话
- 调节音量
- 控制LED灯颜色

#### 方式2: Python客户端

```python
from g1_tts_client_example import G1TTSClient

# 创建客户端
client = G1TTSClient("http://localhost:8080")

# 让机器人说话
client.speak("你好，我是G1机器人！")

# 设置音量
client.set_volume(80)

# 设置LED灯为红色
client.set_led(255, 0, 0)
```

运行完整示例：
```bash
python3 g1_tts_client_example.py
```

#### 方式3: curl命令

```bash
# 让机器人说话
curl -X POST http://localhost:8080/speak \
     -H "Content-Type: application/json" \
     -d '{"text": "你好，世界！"}'

# 设置音量
curl -X POST http://localhost:8080/volume/set \
     -H "Content-Type: application/json" \
     -d '{"volume": 70}'

# 获取当前音量
curl http://localhost:8080/volume/get

# 设置LED灯颜色
curl -X POST http://localhost:8080/led/set \
     -H "Content-Type: application/json" \
     -d '{"r": 255, "g": 0, "b": 0}'
```

#### 方式4: 任何编程语言

由于使用标准HTTP API，你可以用任何语言调用：

**JavaScript (浏览器/Node.js)**:
```javascript
fetch('http://localhost:8080/speak', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({text: '你好，G1机器人！'})
})
.then(r => r.json())
.then(data => console.log(data));
```

**C++ (libcurl)**:
```cpp
CURL *curl = curl_easy_init();
curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:8080/speak");
curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "{\"text\":\"你好\"}");
curl_easy_perform(curl);
```

## API接口文档

### 1. 让机器人说话

**请求**:
```http
POST /speak
Content-Type: application/json

{
    "text": "要说的文字",
    "speaker_id": 0  // 可选，说话人ID
}
```

**响应**:
```json
{
    "success": true,
    "message": "语音发送成功",
    "text": "要说的文字"
}
```

### 2. 设置音量

**请求**:
```http
POST /volume/set
Content-Type: application/json

{
    "volume": 80  // 0-100
}
```

**响应**:
```json
{
    "success": true,
    "message": "音量设置成功: 80",
    "volume": 80
}
```

### 3. 获取音量

**请求**:
```http
GET /volume/get
```

**响应**:
```json
{
    "success": true,
    "volume": 80,
    "message": "当前音量: 80"
}
```

### 4. 设置LED灯颜色

**请求**:
```http
POST /led/set
Content-Type: application/json

{
    "r": 255,  // 红色 (0-255)
    "g": 0,    // 绿色 (0-255)
    "b": 0     // 蓝色 (0-255)
}
```

**响应**:
```json
{
    "success": true,
    "message": "LED颜色设置成功: RGB(255, 0, 0)",
    "color": {"r": 255, "g": 0, "b": 0}
}
```

### 5. 服务状态

**请求**:
```http
GET /status
```

**响应**:
```json
{
    "success": true,
    "service": "G1 TTS Service",
    "status": "running",
    "volume": 80
}
```

## 交互式命令行模式

```bash
python3 g1_tts_service.py --iface eth0 --interactive
```

在交互模式下，支持以下命令：

| 命令 | 功能 |
|------|------|
| 直接输入文字 | 让机器人说话 |
| `/volume <0-100>` | 设置音量 |
| `/led <r> <g> <b>` | 设置LED颜色 |
| `/quit` 或 `/exit` | 退出程序 |

示例：
```
>>> 你好，我是G1机器人
🗣️ 机器人说话: 你好，我是G1机器人
✅ 语音发送成功

>>> /volume 80
🔊 设置音量: 80
✅ 音量设置成功: 80

>>> /led 255 0 0
💡 设置LED颜色: RGB(255, 0, 0)
✅ LED颜色设置成功
```

## 高级用法

### 1. 在后台运行服务

```bash
# 使用nohup在后台运行
nohup python3 g1_tts_service.py --iface eth0 --server > tts.log 2>&1 &

# 查看日志
tail -f tts.log

# 停止服务
pkill -f g1_tts_service.py
```

### 2. 使用systemd服务（开机自启）

创建服务文件 `/etc/systemd/system/g1-tts.service`:

```ini
[Unit]
Description=G1 Robot TTS Service
After=network.target

[Service]
Type=simple
User=your_username
WorkingDirectory=/path/to/unitree_g1_training
ExecStart=/usr/bin/python3 g1_tts_service.py --iface eth0 --server
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

启动服务：
```bash
sudo systemctl daemon-reload
sudo systemctl enable g1-tts
sudo systemctl start g1-tts
sudo systemctl status g1-tts
```

### 3. 远程访问（跨设备控制）

如果要从其他设备访问TTS服务：

1. 启动服务时指定监听所有接口：
```bash
python3 g1_tts_service.py --iface eth0 --server --host 0.0.0.0 --port 8080
```

2. 在客户端指定服务器IP：
```python
client = G1TTSClient("http://192.168.1.100:8080")
```

### 4. 集成到自己的程序

```python
import sys
sys.path.append('./externals/unitree_sdk2_python')
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

# 初始化
ChannelFactoryInitialize(0, "eth0")
audio = AudioClient()
audio.SetTimeout(10.0)
audio.Init()

# 使用
audio.SetVolume(80)
audio.TtsMaker("你好，世界！", 0)
audio.LedControl(255, 0, 0)
```

## 常见问题

### Q1: 无法连接到机器人？

**A**: 检查以下几点：
1. 确认机器人已开机并连接到网络
2. 检查网络接口名称是否正确 (`ifconfig` 查看)
3. 确认防火墙没有阻止连接
4. 尝试 ping 机器人的IP地址

### Q2: 说话没有声音？

**A**: 
1. 检查音量设置：`client.get_volume()`
2. 确认机器人扬声器工作正常
3. 尝试调高音量：`client.set_volume(100)`

### Q3: HTTP服务无法访问？

**A**:
1. 检查端口是否被占用：`netstat -tuln | grep 8080`
2. 尝试更换端口：`--port 8081`
3. 检查防火墙设置

### Q4: 中文语音不清晰？

**A**: 这取决于机器人的TTS引擎，可以尝试：
1. 调整语速（在文字中添加标点符号）
2. 使用更短的句子
3. 查看G1官方文档了解TTS参数

## 性能优化

1. **减少延迟**: TTS服务直接与机器人通信，延迟通常<100ms
2. **批量操作**: 可以连续发送多个命令而无需等待
3. **音频流**: 对于长音频，考虑使用 `PlayStream` API

## 安全建议

⚠️ **生产环境使用注意事项**：

1. **访问控制**: 默认配置允许所有IP访问，建议：
   - 使用防火墙限制访问IP
   - 添加身份验证机制
   - 使用HTTPS加密通信

2. **输入验证**: 当前版本已包含基本验证，但建议：
   - 限制文本长度
   - 过滤敏感内容
   - 添加频率限制

3. **网络安全**: 
   - 不要将服务暴露到公网
   - 使用VPN或专用网络
   - 定期更新SDK

## 扩展开发

基于此服务可以开发：

1. **语音助手**: 结合ASR（语音识别）实现对话
2. **ChatGPT集成**: 让机器人说出AI生成的回答
3. **智能家居控制**: 通过语音控制机器人
4. **多机器人协同**: 控制多台G1机器人
5. **移动端APP**: 开发手机控制应用

示例代码见 `examples/` 目录（待添加）

## 更新日志

### v1.0.0 (2024-10)
- ✅ 初始版本发布
- ✅ HTTP API服务器
- ✅ Web可视化界面
- ✅ 交互式命令行模式
- ✅ TTS、音量、LED控制
- ✅ Python客户端示例

## 致谢

- [Unitree Robotics](https://www.unitree.com/) - G1机器人及SDK
- unitree_sdk2_python 社区

## 许可证

本项目遵循与主项目相同的许可证。

## 联系方式

有问题或建议？欢迎提交Issue或Pull Request！

---

**Happy Coding! 🤖✨**

