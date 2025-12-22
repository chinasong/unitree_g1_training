#!/usr/bin/env python3
"""
G1机器人智能幽默聊天系统
功能：
1. 摄像头检测人
2. 分析人的特征（衣着、外观等）
3. 语音识别听人说话
4. 生成幽默风趣的回复
5. 机器人说话
"""

import sys
import os
import time
import json
import rospy
import cv2
import numpy as np
import threading
import queue
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

# 添加SDK路径
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sdk_path = os.path.join(project_root, 'externals', 'unitree_sdk2_python')

# 尝试从本地externals文件夹导入SDK
if os.path.exists(sdk_path):
    sys.path.append(sdk_path)
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
    except ImportError as e:
        print(f"⚠️ 无法从本地路径导入SDK: {e}")
        print("   尝试从已安装的包导入...")
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
        except ImportError:
            print("❌ 无法导入unitree_sdk2py，请确保:")
            print("   1. externals/unitree_sdk2_python 文件夹存在，或")
            print("   2. 已通过pip安装: pip install unitree_sdk2_python")
            sys.exit(1)
else:
    # externals文件夹不存在，尝试从已安装的包导入
    print("⚠️ 未找到本地externals/unitree_sdk2_python文件夹")
    print("   尝试从已安装的包导入...")
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
    except ImportError:
        print("❌ 无法导入unitree_sdk2py，请执行以下操作之一:")
        print("   1. 初始化git submodule: git submodule update --init --recursive")
        print("   2. 或通过pip安装: cd externals/unitree_sdk2_python && pip install -e .")
        sys.exit(1)

# 尝试导入语音识别和LLM相关库
try:
    import speech_recognition as sr
    HAS_SPEECH_RECOGNITION = True
except ImportError:
    HAS_SPEECH_RECOGNITION = False
    print("⚠️ 未安装speech_recognition，语音识别功能将不可用")

try:
    import openai
    HAS_OPENAI = True
except ImportError:
    HAS_OPENAI = False
    print("⚠️ 未安装openai，将使用简单的回复生成器")

# DeepSeek API配置
# 从环境变量读取API密钥（必须设置）
DEEPSEEK_API_KEY = os.getenv("DEEPSEEK_API_KEY")
DEEPSEEK_API_BASE = "https://api.deepseek.com"
DEEPSEEK_MODEL = "deepseek-chat"

bridge = CvBridge()

# 全局变量
person_detected = False
last_person_time = 0
first_person_detected_time = 0  # 第一次检测到人的时间
last_chat_time = 0
person_features = {}
conversation_history = []
is_speaking = False
audio_queue = queue.Queue()

# 配置参数
PERSON_DETECT_INTERVAL = 2.0  # 检测到人后多久开始聊天（秒）
CHAT_COOLDOWN = 5.0  # 两次聊天之间的冷却时间（秒）
PERSON_DISAPPEAR_TIMEOUT = 3.0  # 人消失多久后重置状态（秒）

# 加载YOLO模型
print("📦 正在加载YOLO模型...")
yolo_model_path = os.path.join(script_dir, "yolo11n.pt")
if os.path.exists(yolo_model_path):
    yolo_model = YOLO(yolo_model_path)
else:
    print("⚠️ 未找到本地yolo11n.pt，YOLO将自动下载模型")
    yolo_model = YOLO("yolo11n.pt")  # YOLO会自动下载

# 加载标签映射
label_map = {}
label_map_path = os.path.join(script_dir, "en_to_zh.js")
try:
    with open(label_map_path, "r", encoding="utf-8") as f:
        label_map = json.load(f)
except FileNotFoundError:
    print("⚠️ 未找到en_to_zh.js，将使用英文标签")


def init_audio_client(net_iface: str):
    """初始化音频客户端"""
    ChannelFactoryInitialize(0, net_iface)
    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()
    audio_client.SetVolume(85)
    return audio_client


def speak(audio_client, text):
    """让机器人说话"""
    global is_speaking
    if is_speaking:
        return  # 如果正在说话，跳过
    
    is_speaking = True
    print(f"🤖 机器人说: {text}")
    try:
        audio_client.TtsMaker(text, 0)
        # 估算说话时间（中文字符大约每个0.5秒）
        speak_duration = len(text) * 0.5
        time.sleep(min(speak_duration, 10))  # 最多等待10秒
    except Exception as e:
        print(f"❌ 说话失败: {e}")
    finally:
        is_speaking = False


def analyze_person_features(frame, person_box):
    """分析人的特征（衣着、外观等）"""
    x1, y1, x2, y2 = person_box.xyxy[0].cpu().numpy()
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    
    # 提取人物区域
    person_roi = frame[y1:y2, x1:x2]
    
    if person_roi.size == 0:
        return {}
    
    features = {}
    
    # 分析颜色（简单的颜色分析）
    hsv = cv2.cvtColor(person_roi, cv2.COLOR_BGR2HSV)
    
    # 检测主要颜色
    hist_h = cv2.calcHist([hsv], [0], None, [180], [0, 180])
    dominant_hue = np.argmax(hist_h)
    
    # 颜色描述
    color_names = {
        (0, 10): "红色", (10, 25): "橙色", (25, 35): "黄色",
        (35, 85): "绿色", (85, 130): "蓝色", (130, 180): "紫色"
    }
    
    color_name = "彩色"
    for (low, high), name in color_names.items():
        if low <= dominant_hue <= high:
            color_name = name
            break
    
    features['dominant_color'] = color_name
    
    # 分析亮度
    brightness = np.mean(cv2.cvtColor(person_roi, cv2.COLOR_BGR2GRAY))
    if brightness > 180:
        features['brightness'] = "明亮"
    elif brightness < 80:
        features['brightness'] = "深色"
    else:
        features['brightness'] = "中等"
    
    # 分析尺寸（相对于画面）
    person_area = (x2 - x1) * (y2 - y1)
    frame_area = frame.shape[0] * frame.shape[1]
    area_ratio = person_area / frame_area
    
    if area_ratio > 0.3:
        features['size'] = "很大"
    elif area_ratio > 0.15:
        features['size'] = "中等"
    else:
        features['size'] = "较小"
    
    # 位置信息
    frame_center_x = frame.shape[1] / 2
    person_center_x = (x1 + x2) / 2
    
    if person_center_x < frame_center_x - 100:
        features['position'] = "左侧"
    elif person_center_x > frame_center_x + 100:
        features['position'] = "右侧"
    else:
        features['position'] = "中间"
    
    return features


def generate_humorous_response(features, user_speech=None, conversation_history=None):
    """生成幽默风趣的回复"""
    
    # 构建提示词（所有API都使用相同的提示词）
    system_prompt = """你是一个幽默风趣的机器人，喜欢用轻松幽默的方式和人聊天。
你会根据看到的人的特征（衣着、外观等）和对方说的话，生成有趣、友好的回复。
回复要简短（1-2句话），幽默但不失礼貌，用中文回答。"""
    
    user_prompt = f"""我看到一个人，特征如下：
- 主要颜色：{features.get('dominant_color', '未知')}
- 亮度：{features.get('brightness', '未知')}
- 大小：{features.get('size', '未知')}
- 位置：{features.get('position', '未知')}"""
    
    if user_speech:
        user_prompt += f"\n\n这个人说：{user_speech}"
    
    if conversation_history:
        user_prompt += f"\n\n之前的对话：\n" + "\n".join(conversation_history[-3:])
    
    # 优先使用DeepSeek API
    if HAS_OPENAI and DEEPSEEK_API_KEY and DEEPSEEK_API_KEY.strip():
        try:
            # 使用DeepSeek API
            client = openai.OpenAI(
                api_key=DEEPSEEK_API_KEY,
                base_url=DEEPSEEK_API_BASE
            )
            
            response = client.chat.completions.create(
                model=DEEPSEEK_MODEL,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=150,
                temperature=0.8
            )
            
            return response.choices[0].message.content.strip()
        except Exception as e:
            print(f"⚠️ DeepSeek API调用失败: {e}，尝试使用OpenAI API")
            # 如果DeepSeek失败，尝试使用OpenAI API（如果配置了）
            if os.getenv("OPENAI_API_KEY"):
                try:
                    client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
                    response = client.chat.completions.create(
                        model="gpt-3.5-turbo",
                        messages=[
                            {"role": "system", "content": system_prompt},
                            {"role": "user", "content": user_prompt}
                        ],
                        max_tokens=100,
                        temperature=0.8
                    )
                    return response.choices[0].message.content.strip()
                except Exception as e2:
                    print(f"⚠️ OpenAI API调用也失败: {e2}，使用简单回复生成器")
            else:
                print("⚠️ 使用简单回复生成器")
    
    # 简单的回复生成器（备用方案）
    responses = []
    
    # 基于特征的回复
    color = features.get('dominant_color', '')
    if color:
        color_responses = {
            '红色': "哇，你穿得这么红，是要去参加什么重要场合吗？",
            '蓝色': "蓝色很清爽啊，看起来很有活力！",
            '绿色': "绿色很自然，你是在追求环保吗？",
            '黄色': "黄色很亮眼，你一定是人群中最闪亮的那个！",
            '橙色': "橙色很有活力，你看起来精神不错！",
            '紫色': "紫色很神秘，你是个有故事的人吧？"
        }
        if color in color_responses:
            responses.append(color_responses[color])
    
    # 基于位置的回复
    position = features.get('position', '')
    if position == '左侧':
        responses.append("你站在左边，是想让我多看看你吗？")
    elif position == '右侧':
        responses.append("你在右边，我们正好可以面对面聊天！")
    
    # 基于大小的回复
    size = features.get('size', '')
    if size == '很大':
        responses.append("你离我很近啊，是想和我近距离交流吗？")
    elif size == '较小':
        responses.append("你离我有点远，不过我还是能看到你！")
    
    # 基于用户说话的回复
    if user_speech:
        if '你好' in user_speech or 'hello' in user_speech.lower():
            responses.append("你好！很高兴见到你！")
        elif '谢谢' in user_speech or 'thank' in user_speech.lower():
            responses.append("不客气！能和你聊天我也很开心！")
        elif '再见' in user_speech or 'bye' in user_speech.lower():
            responses.append("再见！期待下次见面！")
        else:
            responses.append(f"你说'{user_speech}'，听起来很有趣！")
    
    # 默认回复
    if not responses:
        responses = [
            "你好！很高兴看到你！",
            "今天天气不错，你看起来心情也很好！",
            "你看起来很有趣，想和我聊聊吗？",
            "我注意到你了，要不要聊聊天？"
        ]
    
    import random
    return random.choice(responses)


def listen_to_speech():
    """监听语音输入"""
    if not HAS_SPEECH_RECOGNITION:
        return None
    
    try:
        r = sr.Recognizer()
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=0.5)
            print("🎤 正在聆听...")
            audio = r.listen(source, timeout=2, phrase_time_limit=5)
        
        try:
            # 使用Google语音识别（中文）
            text = r.recognize_google(audio, language='zh-CN')
            print(f"👤 听到: {text}")
            return text
        except sr.UnknownValueError:
            print("⚠️ 无法识别语音")
            return None
        except sr.RequestError as e:
            print(f"⚠️ 语音识别服务错误: {e}")
            return None
    except Exception as e:
        print(f"⚠️ 语音识别异常: {e}")
        return None


def speech_listener_thread():
    """语音监听线程"""
    global audio_queue
    while True:
        try:
            speech_text = listen_to_speech()
            if speech_text:
                audio_queue.put(('speech', speech_text))
        except Exception as e:
            print(f"⚠️ 语音监听线程错误: {e}")
            time.sleep(1)


def image_callback(msg):
    """图像回调函数"""
    global person_detected, last_person_time, first_person_detected_time, last_chat_time, person_features, conversation_history, is_speaking
    
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    current_time = time.time()
    
    # 使用YOLO检测人
    results = yolo_model(frame, verbose=False)
    
    # 检查是否检测到人
    person_boxes = []
    for result in results:
        boxes = result.boxes
        if boxes is not None:
            for box in boxes:
                cls = int(box.cls[0])
                class_name = yolo_model.names[cls]
                if class_name == 'person':
                    person_boxes.append(box)
    
    # 处理检测到的人
    if person_boxes:
        if not person_detected:
            # 第一次检测到人
            first_person_detected_time = current_time
            person_detected = True
        
        last_person_time = current_time
        
        # 分析第一个人的特征
        if person_boxes:
            person_features = analyze_person_features(frame, person_boxes[0])
            print(f"👤 检测到人，特征: {person_features}")
        
        # 检查是否可以开始聊天
        time_since_detection = current_time - first_person_detected_time
        time_since_last_chat = current_time - last_chat_time
        
        if (time_since_detection >= PERSON_DETECT_INTERVAL and 
            time_since_last_chat >= CHAT_COOLDOWN and 
            not is_speaking):
            
            # 检查是否有新的语音输入
            user_speech = None
            try:
                while not audio_queue.empty():
                    msg_type, data = audio_queue.get_nowait()
                    if msg_type == 'speech':
                        user_speech = data
            except:
                pass
            
            # 生成回复
            response = generate_humorous_response(
                person_features, 
                user_speech, 
                conversation_history
            )
            
            if response:
                # 记录对话
                if user_speech:
                    conversation_history.append(f"用户: {user_speech}")
                conversation_history.append(f"机器人: {response}")
                
                # 保持对话历史不超过10条
                if len(conversation_history) > 10:
                    conversation_history = conversation_history[-10:]
                
                # 让机器人说话
                speak(audio_client, response)
                last_chat_time = current_time
    else:
        # 人消失了
        if person_detected and (current_time - last_person_time > PERSON_DISAPPEAR_TIMEOUT):
            person_detected = False
            first_person_detected_time = 0
            person_features = {}
            print("👋 人已离开视野")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        print("Example: python3 {sys.argv[0]} eth0")
        sys.exit(-1)
    
    net_iface = sys.argv[1]
    
    # 初始化音频客户端
    print("🔊 正在初始化音频客户端...")
    audio_client = init_audio_client(net_iface)
    
    # 启动语音监听线程（如果可用）
    if HAS_SPEECH_RECOGNITION:
        print("🎤 启动语音监听线程...")
        speech_thread = threading.Thread(target=speech_listener_thread, daemon=True)
        speech_thread.start()
    else:
        print("⚠️ 语音识别不可用，将仅基于视觉特征生成回复")
    
    # 初始化ROS节点
    print("📡 正在初始化ROS节点...")
    rospy.init_node("g1_humorous_chat")
    
    # 订阅摄像头话题
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    
    print("✅ G1智能幽默聊天系统已启动！")
    print("📋 功能说明：")
    print("   - 自动检测视野中的人")
    print("   - 分析人的特征（衣着、外观等）")
    if HAS_SPEECH_RECOGNITION:
        print("   - 聆听人的语音")
    if HAS_OPENAI and DEEPSEEK_API_KEY and DEEPSEEK_API_KEY.strip():
        print(f"   - 使用DeepSeek AI生成幽默回复 (模型: {DEEPSEEK_MODEL})")
    elif HAS_OPENAI and os.getenv("OPENAI_API_KEY"):
        print("   - 使用OpenAI AI生成幽默回复")
    else:
        print("   - 使用简单规则生成回复")
        if HAS_OPENAI:
            print("   ⚠️  提示: 设置DEEPSEEK_API_KEY环境变量可使用AI生成回复")
    print("   - 机器人主动聊天")
    print("\n按 Ctrl+C 退出")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n👋 正在退出...")
        sys.exit(0)

