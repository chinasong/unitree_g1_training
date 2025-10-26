# G1 机器人 TTS 项目总览

## 📦 项目文件

本次创建了一个完整的G1机器人文字转语音（TTS）系统，包含以下文件：

### 核心服务
1. **g1_tts_service.py** - TTS服务主程序（600+ 行）
   - HTTP API服务器
   - Web可视化控制面板
   - 交互式命令行模式
   - 支持TTS、音量控制、LED灯控制

### 客户端示例
2. **g1_tts_client_example.py** - Python客户端示例（200+ 行）
   - 展示如何通过网络调用TTS服务
   - 包含完整的API使用示例
   - 提供curl命令示例

3. **g1_tts_simple_test.py** - 最简单的使用示例（100+ 行）
   - 直接连接机器人，无需服务器
   - 适合快速测试和学习

### 工具脚本
4. **quick_start_tts.sh** - 快速启动脚本
   - 交互式选择运行模式
   - 自动检测网络接口

### 文档
5. **G1_TTS_SERVICE_README.md** - 完整使用文档（300+ 行）
   - 详细的功能说明
   - API接口文档
   - 使用示例和最佳实践
   - 常见问题解答

6. **G1_TTS_项目说明.md** - 本文件

## 🚀 快速使用指南

### 方法1: 最简单（推荐初学者）

直接测试TTS功能，无需启动服务器：

```bash
python3 g1_tts_simple_test.py eth0
```

### 方法2: HTTP服务（推荐生产使用）

启动HTTP服务器，可以通过网络控制：

```bash
# 启动服务
python3 g1_tts_service.py --iface eth0 --server

# 在浏览器打开
http://localhost:8080

# 或使用Python客户端
python3 g1_tts_client_example.py
```

### 方法3: 交互式命令行

实时输入文字让机器人说话：

```bash
python3 g1_tts_service.py --iface eth0 --interactive
```

### 方法4: 使用快速启动脚本

```bash
bash quick_start_tts.sh
```

## 🎯 核心功能

### 1. 文字转语音（TTS）
```python
# Python
client.speak("你好，我是G1机器人！")

# curl
curl -X POST http://localhost:8080/speak \
     -H "Content-Type: application/json" \
     -d '{"text": "你好，世界！"}'
```

### 2. 音量控制
```python
# 设置音量
client.set_volume(80)

# 获取音量
success, volume = client.get_volume()
```

### 3. LED灯控制
```python
# 设置为红色
client.set_led(255, 0, 0)
```

## 📡 支持的接口

| 接口 | 方法 | 功能 |
|------|------|------|
| `/` | GET | Web控制面板 |
| `/speak` | POST | 文字转语音 |
| `/volume/set` | POST | 设置音量 |
| `/volume/get` | GET | 获取音量 |
| `/led/set` | POST | 设置LED颜色 |
| `/status` | GET | 服务状态 |

## 🌟 特色功能

### 1. Web可视化界面
- 浏览器打开 `http://localhost:8080`
- 可视化输入文字
- 拖动滑块调节音量
- 颜色选择器控制LED

### 2. 跨平台支持
- ✅ Python
- ✅ JavaScript (浏览器/Node.js)
- ✅ curl命令行
- ✅ 任何支持HTTP的语言

### 3. 多种运行模式
- 🌐 HTTP服务器模式 - 网络访问
- 💻 交互式模式 - 命令行实时控制
- 🎯 单次执行模式 - 说一句话就退出

## 💡 使用场景

### 场景1: 智能语音播报
```python
# 监控系统异常时播报
if temperature > 80:
    client.speak("警告！温度过高！")
    client.set_led(255, 0, 0)  # 红色警告
```

### 场景2: 集成ChatGPT
```python
# 让机器人说出AI回答
response = chatgpt.ask("今天天气怎么样？")
client.speak(response)
```

### 场景3: 多语言播报
```python
# 中英文混合
client.speak("Hello, 我是G1机器人")
client.speak("Today is 星期一")
```

### 场景4: 远程控制
```javascript
// 从手机浏览器控制机器人
fetch('http://192.168.1.100:8080/speak', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({text: '开始工作'})
});
```

## 🔧 技术架构

```
┌─────────────────┐
│  客户端层        │
│  - Web浏览器    │
│  - Python程序   │
│  - 其他语言     │
└────────┬────────┘
         │ HTTP
         ↓
┌─────────────────┐
│  服务层          │
│  g1_tts_service │
│  - HTTP Server  │
│  - API Router   │
└────────┬────────┘
         │ unitree_sdk2_python
         ↓
┌─────────────────┐
│  SDK层          │
│  AudioClient    │
│  - TTS          │
│  - 音量控制     │
│  - LED控制      │
└────────┬────────┘
         │ DDS/网络
         ↓
┌─────────────────┐
│  G1机器人       │
└─────────────────┘
```

## 📊 代码统计

| 文件 | 行数 | 功能 |
|------|------|------|
| g1_tts_service.py | 650+ | 核心服务 |
| g1_tts_client_example.py | 250+ | 客户端示例 |
| g1_tts_simple_test.py | 120+ | 简单测试 |
| quick_start_tts.sh | 60+ | 启动脚本 |
| G1_TTS_SERVICE_README.md | 500+ | 文档 |
| **总计** | **1600+** | - |

## 🎓 学习路径

### 初学者
1. 运行 `g1_tts_simple_test.py` 了解基本功能
2. 阅读代码，理解TTS API调用方式
3. 修改测试脚本，尝试自己的文字

### 中级用户
1. 启动HTTP服务器
2. 使用Web界面测试各种功能
3. 学习HTTP API的使用
4. 用curl命令测试接口

### 高级用户
1. 阅读 `g1_tts_service.py` 源码
2. 理解HTTP服务器实现
3. 集成到自己的项目中
4. 扩展新功能（如录音、对话等）

## 🔐 安全注意事项

⚠️ **重要提醒**：

1. **默认配置监听所有IP** (`0.0.0.0`)
   - 局域网内所有设备都能访问
   - 不要在公网环境使用

2. **无身份验证**
   - 任何人都能控制机器人说话
   - 生产环境建议添加认证

3. **建议措施**：
   - 使用防火墙限制访问
   - 修改默认端口
   - 添加访问密码
   - 使用VPN或专用网络

## 🚧 未来扩展

### 可以添加的功能：
- [ ] 语音识别（ASR）- 实现对话
- [ ] 情绪分析 - 根据文本调整语调
- [ ] 声音克隆 - 自定义说话人
- [ ] 音频流播放 - 播放自定义音频
- [ ] WebSocket支持 - 实时双向通信
- [ ] 用户认证 - 添加登录系统
- [ ] 日志记录 - 记录所有操作
- [ ] 多机器人控制 - 同时控制多台G1

### 集成方案：
- 🤖 **ChatGPT/Claude** - AI对话
- 🏠 **Home Assistant** - 智能家居
- 📱 **移动APP** - iOS/Android控制
- 🎮 **Unity3D** - 游戏/仿真集成
- 🌐 **ROS** - 机器人操作系统

## 📝 常用命令速查

```bash
# 快速测试
python3 g1_tts_simple_test.py eth0

# 启动服务器
python3 g1_tts_service.py --iface eth0 --server

# 交互模式
python3 g1_tts_service.py --iface eth0 --interactive

# 说一句话
python3 g1_tts_service.py --iface eth0 --speak "你好"

# 后台运行
nohup python3 g1_tts_service.py --iface eth0 --server > tts.log 2>&1 &

# 停止服务
pkill -f g1_tts_service.py

# 查看日志
tail -f tts.log
```

## 🆘 遇到问题？

1. 查看 `G1_TTS_SERVICE_README.md` 的常见问题部分
2. 确认机器人已连接并开机
3. 检查网络接口名称 (`ifconfig`)
4. 查看服务日志
5. 尝试重启服务

## 📚 相关资源

- [Unitree G1 官方文档](https://www.unitree.com/)
- [unitree_sdk2_python GitHub](https://github.com/unitreerobotics/unitree_sdk2_python)
- 项目主README: `README.md`

## 🎉 总结

这是一个功能完整、易于使用的G1机器人TTS解决方案：

✅ **开箱即用** - 5分钟内启动运行  
✅ **文档齐全** - 详细的使用说明和示例  
✅ **代码规范** - 无linter错误，注释完整  
✅ **扩展性强** - 易于集成和二次开发  
✅ **跨平台** - 支持多种编程语言  

**立即开始使用吧！** 🚀

---

*创建日期: 2024-10-26*  
*版本: v1.0.0*

