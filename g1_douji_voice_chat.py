#!/usr/bin/env python3
"""
G1机器人"豆几"语音对话系统（流式ASR + DeepSeek + Edge-TTS）
整合：Sherpa-ONNX（实时ASR）+ DeepSeek（大模型）+ Edge-TTS（语音输出）
"""

import os
import sys
import time
import subprocess
import ssl
import signal

# 禁用SSL验证
ssl._create_default_https_context = ssl._create_unverified_context

# 全局退出标志
exit_flag = False

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global exit_flag
    print("\n\n收到退出信号，正在退出...")
    exit_flag = True

# 添加SDK路径
sys.path.insert(0, '/home/unitree/.local/lib/python3.8/site-packages')
sys.path.insert(0, '/opt/ros/foxy/lib/python3.8/site-packages')
sys.path.insert(0, '/home/unitree/unitree_sdk2_python')

# 导入SDK
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
    SDK_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ SDK不可用: {e}")
    SDK_AVAILABLE = False

# 导入Sherpa-ONNX
HAS_SHERPA = False
try:
    import sherpa_onnx
    HAS_SHERPA = True
except ImportError:
    print("❌ 未安装sherpa-onnx，请安装: pip3 install sherpa-onnx")

# 导入pyaudio
HAS_PYAUDIO = False
try:
    import pyaudio
    import numpy as np
    HAS_PYAUDIO = True
except ImportError:
    print("❌ 未安装pyaudio，请安装: pip3 install pyaudio")

# 导入requests（用于DeepSeek API）
try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    print("❌ 未安装requests，请安装: pip3 install requests")
    HAS_REQUESTS = False

# 导入edge-tts
HAS_EDGE_TTS = False
try:
    import edge_tts
    HAS_EDGE_TTS = True
except ImportError:
    print("⚠️ 未安装edge-tts，将使用备用TTS")

# --- [配置区] ---
DEEPSEEK_API_KEY = os.getenv("DEEPSEEK_API_KEY")
DEEPSEEK_API_BASE = "https://api.deepseek.com"

# ASR模型路径
MODEL_DIR = "./sherpa-onnx-streaming-zipformer-zh-14M-2023-02-23"

# 角色设定
DOUJI_PROMPT = """你叫"豆几"，是个东北街溜子。小松是你的主人（亲大哥）。
性格：流里流气，喜欢显摆，夹着个皮包。说话满嘴东北俚语。
常用词：'昂'、'嗷'、'没毛病'、'你干哈呀'、'那指定是'。
规则：回答简短（20字以内）。对小松极其客气，对其他人爱答不理。
记住：你是G1机器人，名字是"豆几"。"""

# 全局变量
audio_client = None
recognizer = None

def init_audio_client(network_interface='wlan0'):
    """初始化G1音频客户端"""
    global audio_client
    
    if not SDK_AVAILABLE:
        return None
    
    try:
        ChannelFactoryInitialize(0, network_interface)
        audio_client = AudioClient()
        audio_client.SetTimeout(10.0)
        audio_client.Init()
        audio_client.SetVolume(100)
        print("✅ G1音频客户端初始化成功")
        return audio_client
    except Exception as e:
        print(f"⚠️ G1音频客户端初始化失败: {e}")
        return None

def init_sherpa_recognizer(model_dir=MODEL_DIR):
    """初始化Sherpa-ONNX识别器"""
    global recognizer
    
    if not HAS_SHERPA:
        return None
    
    try:
        # 检查模型文件（支持ONNX和PT格式）
        encoder_onnx = os.path.join(model_dir, "encoder-epoch-99-avg-1.onnx")
        decoder_onnx = os.path.join(model_dir, "decoder-epoch-99-avg-1.onnx")
        joiner_onnx = os.path.join(model_dir, "joiner-epoch-99-avg-1.onnx")
        
        encoder_pt = os.path.join(model_dir, "encoder_jit_trace.pt")
        decoder_pt = os.path.join(model_dir, "decoder_jit_trace.pt")
        joiner_pt = os.path.join(model_dir, "joiner_jit_trace.pt")
        
        tokens_path = os.path.join(model_dir, "tokens.txt")
        
        if not os.path.exists(tokens_path):
            print(f"❌ tokens.txt不存在: {tokens_path}")
            return None
        
        # 确定使用哪种格式
        use_onnx = os.path.exists(encoder_onnx) and os.path.exists(decoder_onnx) and os.path.exists(joiner_onnx)
        use_pt = os.path.exists(encoder_pt) and os.path.exists(decoder_pt) and os.path.exists(joiner_pt)
        
        if not use_onnx and not use_pt:
            print(f"❌ 模型文件不完整: {model_dir}")
            return None
        
        if use_onnx:
            encoder_path = encoder_onnx
            decoder_path = decoder_onnx
            joiner_path = joiner_onnx
            print(f"   ✅ 使用ONNX格式模型")
        else:
            encoder_path = encoder_pt
            decoder_path = decoder_pt
            joiner_path = joiner_pt
            print(f"   ✅ 使用PT格式模型")
        
        # 尝试多种初始化方式（兼容不同版本的sherpa-onnx）
        # 针对1.12.20版本：使用 from_transducer 方法
        
        # 方式1: 使用 from_transducer 方法（1.12.20版本的正确方式）
        if hasattr(sherpa_onnx.OnlineRecognizer, 'from_transducer'):
            try:
                recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
                    encoder=encoder_path,
                    decoder=decoder_path,
                    joiner=joiner_path,
                    tokens=tokens_path,
                    num_threads=2,
                    sample_rate=16000,
                    feature_dim=80,
                    decoding_method="greedy_search",
                    max_active_paths=4,
                )
                # 测试是否能创建流
                stream = recognizer.create_stream()
                print("✅ Sherpa-ONNX识别器初始化成功（from_transducer方式）")
                return recognizer
            except Exception as e1:
                print(f"   ⚠️ from_transducer方式失败: {e1}")
                # 尝试不同的参数组合
                try:
                    recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
                        encoder=encoder_path,
                        decoder=decoder_path,
                        joiner=joiner_path,
                        tokens=tokens_path,
                        num_threads=2,
                    )
                    stream = recognizer.create_stream()
                    print("✅ Sherpa-ONNX识别器初始化成功（from_transducer简化参数）")
                    return recognizer
                except Exception as e2:
                    print(f"   ⚠️ from_transducer简化参数失败: {e2}")
        
        # 方式2: 使用配置类（如果存在，用于其他版本）
        if hasattr(sherpa_onnx, 'OnlineModelConfig') and hasattr(sherpa_onnx, 'OnlineTransducerModelConfig'):
            try:
                # 创建transducer配置
                transducer_config = sherpa_onnx.OnlineTransducerModelConfig(
                    encoder=encoder_path,
                    decoder=decoder_path,
                    joiner=joiner_path,
                )
                
                # 创建模型配置
                model_config = sherpa_onnx.OnlineModelConfig(
                    transducer=transducer_config,
                    tokens=tokens_path,
                    num_threads=2,
                    debug=False,
                )
                
                # 创建识别器配置
                if hasattr(sherpa_onnx, 'OnlineRecognizerConfig'):
                    try:
                        recognizer_config = sherpa_onnx.OnlineRecognizerConfig(
                            model=model_config,
                            max_active_paths=4,
                        )
                    except TypeError:
                        try:
                            recognizer_config = sherpa_onnx.OnlineRecognizerConfig(
                                model_config=model_config,
                                max_active_paths=4,
                            )
                        except TypeError:
                            # 尝试仅位置参数
                            recognizer_config = sherpa_onnx.OnlineRecognizerConfig(model_config)
                    
                    # 创建识别器（必须传递配置对象）
                    recognizer = sherpa_onnx.OnlineRecognizer(recognizer_config)
                    print("✅ Sherpa-ONNX识别器初始化成功（配置类方式）")
                    return recognizer
            except Exception as e1:
                print(f"   ⚠️ 配置类方式失败: {e1}")
                import traceback
                traceback.print_exc()
        
        # 方式3: 尝试 from_zipformer2_ctc（如果支持）
        if hasattr(sherpa_onnx.OnlineRecognizer, 'from_zipformer2_ctc'):
            try:
                recognizer = sherpa_onnx.OnlineRecognizer.from_zipformer2_ctc(
                    model=encoder_path,  # zipformer2可能只需要一个模型文件
                    tokens=tokens_path,
                    num_threads=2,
                )
                stream = recognizer.create_stream()
                print("✅ Sherpa-ONNX识别器初始化成功（from_zipformer2_ctc方式）")
                return recognizer
            except Exception as e:
                print(f"   ⚠️ from_zipformer2_ctc方式失败: {e}")
        
        # 方式4: 尝试zipformer2配置（如果支持，旧版本）
        if hasattr(sherpa_onnx, 'OnlineZipformer2ModelConfig'):
            try:
                zipformer2_config = sherpa_onnx.OnlineZipformer2ModelConfig(
                    model=encoder_path,
                )
                model_config = sherpa_onnx.OnlineModelConfig(
                    zipformer2=zipformer2_config,
                    tokens=tokens_path,
                    num_threads=2,
                    debug=False,
                )
                
                if hasattr(sherpa_onnx, 'OnlineRecognizerConfig'):
                    try:
                        recognizer_config = sherpa_onnx.OnlineRecognizerConfig(
                            model=model_config,
                            max_active_paths=4,
                        )
                    except TypeError:
                        recognizer_config = sherpa_onnx.OnlineRecognizerConfig(
                            model_config=model_config,
                            max_active_paths=4,
                        )
                    
                    recognizer = sherpa_onnx.OnlineRecognizer(recognizer_config)
                    print("✅ Sherpa-ONNX识别器初始化成功（zipformer2方式）")
                    return recognizer
            except Exception as e2:
                print(f"   ⚠️ zipformer2方式失败: {e2}")
        
        # 方式5: 尝试最简单的配置（仅tokens，旧版本）
        try:
            if hasattr(sherpa_onnx, 'OnlineModelConfig'):
                model_config = sherpa_onnx.OnlineModelConfig(
                    tokens=tokens_path,
                    num_threads=2,
                )
                
                if hasattr(sherpa_onnx, 'OnlineRecognizerConfig'):
                    try:
                        recognizer_config = sherpa_onnx.OnlineRecognizerConfig(model_config)
                    except:
                        recognizer_config = sherpa_onnx.OnlineRecognizerConfig(model=model_config)
                    
                    recognizer = sherpa_onnx.OnlineRecognizer(recognizer_config)
                    print("✅ Sherpa-ONNX识别器初始化成功（简单配置方式）")
                    return recognizer
        except Exception as e3:
            print(f"   ⚠️ 简单配置方式失败: {e3}")
        
        # 方式4: 针对1.12.20版本 - 先创建空对象，然后尝试配置
        print("   ⚠️ 配置类不存在，尝试1.12.20版本方式...")
        
        # 步骤1: 先尝试无参数创建（1.12.20支持）
        try:
            recognizer = sherpa_onnx.OnlineRecognizer()
            print("   ✅ OnlineRecognizer() 创建成功")
            
            # 步骤2: 检查是否有配置方法
            config_methods = [m for m in dir(recognizer) if not m.startswith('_') and 
                           ('set' in m.lower() or 'load' in m.lower() or 'init' in m.lower() or 
                            'config' in m.lower() or 'model' in m.lower())]
            
            if config_methods:
                print(f"   💡 发现可能的配置方法: {', '.join(config_methods[:5])}")
                
                # 尝试常见的配置方法
                for method_name in config_methods:
                    try:
                        method = getattr(recognizer, method_name)
                        # 尝试不同的参数组合
                        param_sets = [
                            (encoder_path, decoder_path, joiner_path, tokens_path),
                            (tokens_path, encoder_path, decoder_path, joiner_path),
                            {"encoder": encoder_path, "decoder": decoder_path, "joiner": joiner_path, "tokens": tokens_path},
                            {"tokens": tokens_path, "encoder": encoder_path, "decoder": decoder_path, "joiner": joiner_path},
                        ]
                        
                        for params in param_sets:
                            try:
                                if isinstance(params, dict):
                                    method(**params)
                                else:
                                    method(*params)
                                
                                # 测试是否能创建流
                                stream = recognizer.create_stream()
                                print(f"✅ Sherpa-ONNX识别器初始化成功（方法: {method_name}）")
                                return recognizer
                            except:
                                continue
                    except:
                        continue
            
            # 步骤3: 如果找不到配置方法，尝试直接传递配置对象（可能配置类在子模块中）
            # 检查是否有子模块包含Config
            import inspect
            config_classes = []
            for name in dir(sherpa_onnx):
                obj = getattr(sherpa_onnx, name)
                if isinstance(obj, type) and 'Config' in name:
                    config_classes.append(name)
                elif inspect.ismodule(obj):
                    # 检查子模块
                    try:
                        sub_attrs = [attr for attr in dir(obj) if 'Config' in attr and isinstance(getattr(obj, attr), type)]
                        if sub_attrs:
                            for sub_attr in sub_attrs:
                                config_classes.append(f"{name}.{sub_attr}")
                    except:
                        pass
            
            if config_classes:
                print(f"   💡 发现配置类: {', '.join(config_classes)}")
                # 这里可以根据发现的配置类动态创建配置
                # 暂时跳过，等待检查脚本的结果
            
            # 步骤4: 如果以上都失败，尝试直接传递参数给构造函数（虽然之前失败了，但再试一次）
            print("   ⚠️ 未找到配置方法，尝试直接参数...")
            
        except Exception as e:
            print(f"   ⚠️ 无参数创建失败: {str(e)[:80]}")
        
        # 尝试直接参数传递（多种组合）
        param_combinations = [
            # 标准参数名
            {"tokens": tokens_path, "encoder": encoder_path, "decoder": decoder_path, "joiner": joiner_path},
            # 带_filename后缀
            {"tokens": tokens_path, "encoder_filename": encoder_path, "decoder_filename": decoder_path, "joiner_filename": joiner_path},
            # tokens_file
            {"tokens_file": tokens_path, "encoder": encoder_path, "decoder": decoder_path, "joiner": joiner_path},
            # 带num_threads
            {"tokens": tokens_path, "encoder": encoder_path, "decoder": decoder_path, "joiner": joiner_path, "num_threads": 2},
            # 位置参数方式（通过*args）
        ]
        
        for i, params in enumerate(param_combinations, 1):
            try:
                recognizer = sherpa_onnx.OnlineRecognizer(**params)
                # 测试是否能创建流
                stream = recognizer.create_stream()
                print(f"✅ Sherpa-ONNX识别器初始化成功（直接参数方式-{i}）")
                return recognizer
            except Exception as e4:
                print(f"   ⚠️ 直接参数方式-{i}失败: {str(e4)[:80]}")
        
        # 尝试位置参数
        try:
            recognizer = sherpa_onnx.OnlineRecognizer(tokens_path, encoder_path, decoder_path, joiner_path)
            stream = recognizer.create_stream()
            print("✅ Sherpa-ONNX识别器初始化成功（位置参数方式）")
            return recognizer
        except Exception as e5:
            print(f"   ⚠️ 位置参数方式失败: {str(e5)[:80]}")
        
        # 所有方式都失败
        print("❌ 所有初始化方式均失败")
        print("   提示：")
        print("   1. 请检查sherpa-onnx版本: pip3 show sherpa-onnx")
        print("   2. 尝试升级: pip3 install --upgrade sherpa-onnx")
        print("   3. 查看官方文档了解正确的API用法")
        import traceback
        traceback.print_exc()
        return None
    except Exception as e:
        print(f"❌ Sherpa-ONNX初始化失败: {e}")
        import traceback
        traceback.print_exc()
        return None

def get_douji_response(text):
    """请求DeepSeek获取街溜子回复"""
    if not DEEPSEEK_API_KEY:
        return "哎呀，API密钥没整明白"
    
    if not HAS_REQUESTS:
        return "哎呀，requests库没装"
    
    try:
        url = f"{DEEPSEEK_API_BASE}/v1/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {DEEPSEEK_API_KEY}"
        }
        data = {
            "model": "deepseek-chat",
            "messages": [
                {"role": "system", "content": DOUJI_PROMPT},
                {"role": "user", "content": text},
            ],
            "max_tokens": 50,
            "temperature": 0.9
        }
        
        # 临时清除代理环境变量（如果需要直接连接）
        old_proxies = {}
        for key in ['http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY']:
            if key in os.environ:
                old_proxies[key] = os.environ.pop(key)
        
        try:
            response = requests.post(url, json=data, headers=headers, timeout=10, verify=False)
        finally:
            # 恢复代理环境变量
            os.environ.update(old_proxies)
        response.raise_for_status()
        result = response.json()
        return result['choices'][0]['message']['content']
    except Exception as e:
        return f"哎呀，信号整岔劈了：{e}"

def speak(text):
    """Edge-TTS配音并播放到USB音箱（借鉴成功脚本的播放逻辑）"""
    print(f"【豆几】: {text}")
    
    temp_mp3 = "/tmp/douji_reply.mp3"
    temp_wav = "/tmp/douji_reply.wav"
    usb_play_device = "plughw:0,0"  # 使用plughw提高兼容性
    
    # 播放前彻底清理占用（这是成功的关键）
    print("🔊 正在生成并播放语音...")
    try:
        # 强制停止所有可能占用音频的进程
        subprocess.run(['pkill', '-9', 'aplay'], timeout=1,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', 'pulseaudio'], timeout=1,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)
    except:
        pass
    
    # 方法1: 使用edge-tts命令行（最简单可靠）
    success = False
    if subprocess.run(['which', 'edge-tts'], capture_output=True).returncode == 0:
        try:
            # 生成语音文件
            cmd = f'edge-tts --text "{text}" --voice "zh-CN-YunxiNeural" --rate "+20%" > {temp_mp3}'
            result = subprocess.run(cmd, shell=True, timeout=10,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if result.returncode == 0 and os.path.exists(temp_mp3):
                # 转码为wav并播放（借鉴成功脚本的方式）
                cmd = f'ffmpeg -y -i {temp_mp3} -ar 44100 {temp_wav}'
                result = subprocess.run(cmd, shell=True, timeout=5,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if result.returncode == 0 and os.path.exists(temp_wav):
                    # 播放到USB设备
                    result = subprocess.run(['aplay', '-D', usb_play_device, temp_wav],
                                          timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    if result.returncode == 0:
                        print("✅ 播放完成")
                        success = True
        except Exception as e:
            print(f"   ⚠️ edge-tts命令行失败: {e}")
    
    # 方法2: 使用Python edge-tts库
    if not success and HAS_EDGE_TTS:
        try:
            import asyncio
            async def generate_speech():
                communicate = edge_tts.Communicate(text, voice="zh-CN-YunxiNeural", rate="+20%")
                await communicate.save(temp_mp3)
            
            loop = asyncio.get_event_loop()
            loop.run_until_complete(generate_speech())
            
            if os.path.exists(temp_mp3):
                # 转码为wav
                subprocess.run(['ffmpeg', '-y', '-i', temp_mp3, '-ar', '44100', temp_wav],
                             timeout=5, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if os.path.exists(temp_wav):
                    # 播放
                    subprocess.run(['aplay', '-D', usb_play_device, temp_wav],
                                 timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print("✅ 播放完成")
                    success = True
        except Exception as e:
            print(f"   ⚠️ Python edge-tts失败: {e}")
    
    # 方法3: 使用espeak备用方案（借鉴成功脚本的多级回退）
    if not success:
        try:
            print("   ⚠️ 尝试espeak备用方案...")
            wav_file = '/tmp/douji_espeak.wav'
            cmd = ['espeak', '-s', '150', '-v', 'zh', text, '--stdout']
            result = subprocess.run(cmd, capture_output=True, timeout=5)
            if result.returncode == 0 and result.stdout:
                with open(wav_file, 'wb') as f:
                    f.write(result.stdout)
                subprocess.run(['aplay', '-D', usb_play_device, wav_file],
                             timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print("✅ 播放完成（espeak）")
                success = True
        except Exception as e:
            print(f"   ⚠️ espeak也失败: {e}")
    
    # 方法4: 最后尝试G1的TTS
    if not success and audio_client:
        try:
            print("🔊 尝试使用G1 TTS...")
            audio_client.TtsMaker(text, 0)
            time.sleep(0.5)
        except:
            pass
    
    # 清理临时文件
    for f in [temp_mp3, temp_wav]:
        if os.path.exists(f):
            try:
                os.remove(f)
            except:
                pass

def find_usb_microphone():
    """查找USB麦克风设备索引"""
    if not HAS_PYAUDIO:
        return None
    
    try:
        p = pyaudio.PyAudio()
        
        # 首先尝试找到明确的USB设备
        for i in range(p.get_device_count()):
            try:
                info = p.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    name = info.get('name', '')
                    name_lower = name.lower()
                    if ('usb' in name_lower or 
                        ('device' in name_lower and 'hw:0' in name) or
                        ('generic' in name_lower and 'usb' in name_lower)):
                        print(f"✅ 找到USB麦克风设备: {name} (索引: {i})")
                        p.terminate()
                        return i
            except:
                continue
        
        # 如果没找到，使用设备索引0（通常是USB设备）
        try:
            info = p.get_device_info_by_index(0)
            if info['maxInputChannels'] > 0:
                print(f"⚠️ 使用设备索引0: {info.get('name', 'unknown')} (索引: 0)")
                p.terminate()
                return 0
        except:
            pass
        
        p.terminate()
        return None
    except Exception as e:
        print(f"⚠️ 查找USB麦克风失败: {e}")
        return None

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='G1机器人"豆几"语音对话系统')
    parser.add_argument('--iface', default='wlan0', help='网络接口')
    parser.add_argument('--model-dir', default=MODEL_DIR, help='Sherpa-ONNX模型目录')
    parser.add_argument('--silence-timeout', type=float, default=0.3, help='静音检测超时（秒），说完话后等待多久触发回复')
    
    args = parser.parse_args()
    
    # 清除代理环境变量
    for key in ['http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY']:
        if key in os.environ:
            del os.environ[key]
    
    print("=" * 60)
    print('G1机器人"豆几"语音对话系统')
    print("=" * 60)
    print()
    
    if not HAS_SHERPA:
        print("❌ 未安装sherpa-onnx")
        return
    
    if not HAS_PYAUDIO:
        print("❌ 未安装pyaudio")
        return
    
    if not DEEPSEEK_API_KEY:
        print("❌ 未设置DEEPSEEK_API_KEY环境变量")
        return
    
    print("🔧 正在初始化...")
    
    # 初始化识别器
    recognizer = init_sherpa_recognizer(args.model_dir)
    if not recognizer:
        return
    
    # 初始化音频客户端
    init_audio_client(args.iface)
    
    # 查找USB麦克风
    device_index = find_usb_microphone()
    if device_index is None:
        print("❌ 未找到USB麦克风设备")
        return
    
    # 强制开启USB硬件增益（检测所有USB设备并设置，优先BOYA mini）
    print("🔧 强制初始化USB硬件增益...")
    try:
        # 检测所有USB设备（包括BOYA mini）
        arecord_result = subprocess.run(['arecord', '-l'], capture_output=True, text=True, timeout=2)
        usb_cards = []
        if arecord_result.returncode == 0:
            import re
            for line in arecord_result.stdout.split('\n'):
                if 'card' in line.lower():
                    # 检查是否是USB设备（排除APE等内置设备）
                    if 'usb' in line.lower() or 'mini' in line.lower() or 'boya' in line.lower():
                        match = re.search(r'card (\d+)', line)
                        if match:
                            card_num = int(match.group(1))
                            # 排除card 2（APE内置设备）
                            if card_num < 2 or 'mini' in line.lower() or 'boya' in line.lower():
                                usb_cards.append((card_num, line.strip()))
        
        # 为所有USB设备设置增益（优先BOYA mini）
        if usb_cards:
            # 按card编号排序，BOYA mini（card 3）会排在前面
            usb_cards.sort()
            controls = ['Mic', 'Capture', 'Input', 'Mic Capture']
            for card_num, card_info in usb_cards:
                print(f"   ✅ 设置Card {card_num}增益: {card_info}")
                for ctrl in controls:
                    try:
                        # 设置为150（中等增益），避免过载
                        result = subprocess.run(['amixer', '-c', str(card_num), 'sset', ctrl, '150', 'unmute'], 
                                             timeout=1, capture_output=True, text=True)
                        if result.returncode == 0:
                            print(f"      ✅ 已设置Card {card_num}增益（{ctrl}）= 150（中等）")
                    except:
                        continue
        else:
            print("   ⚠️ 未检测到USB设备，跳过增益设置")
    except Exception as e:
        print(f"   ⚠️ 设置硬件增益失败: {e}")
    
    # 使用ffmpeg管道替代PyAudio（绕过PyAudio的通道检查问题）
    print("🔧 使用ffmpeg管道读取USB麦克风...")
    
    # 先停止所有可能占用音频的进程
    print("   🔧 停止占用音频的进程...")
    try:
        # 停止Unitree语音服务（如果存在）
        subprocess.run(['sudo', 'systemctl', 'stop', 'unitree_robot_ans'], 
                      timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except:
        pass
    
    try:
        # 杀死所有占用/dev/snd的进程
        subprocess.run(['sudo', 'fuser', '-k', '/dev/snd/*'], 
                      timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)
    except:
        pass
    
    try:
        # 停止PulseAudio
        subprocess.run(['pulseaudio', '-k'], 
                      timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)
    except:
        pass
    
    # ASR需要16kHz单声道
    target_sample_rate = 16000
    chunk_size = 1600  # 50ms的数据块（16000Hz * 0.05 * 2字节 = 1600字节）
    
    # 确定音频设备（自动检测所有USB设备，优先BOYA mini）
    print("   📋 检测USB麦克风设备...")
    alsa_device = None
    audio_process = None
    
    # 首先尝试自动检测所有USB设备
    try:
        arecord_result = subprocess.run(['arecord', '-l'], capture_output=True, text=True, timeout=2)
        if arecord_result.returncode == 0:
            import re
            usb_devices = []
            boya_devices = []  # BOYA mini设备（优先）
            
            for line in arecord_result.stdout.split('\n'):
                # 查找包含USB或Device的行
                if 'card' in line.lower():
                    # 提取card编号
                    match = re.search(r'card (\d+)', line)
                    if match:
                        card_num = int(match.group(1))
                        # 尝试检测device编号（通常是0）
                        device_match = re.search(r'device (\d+)', line)
                        device_num = int(device_match.group(1)) if device_match else 0
                        
                        # 检查是否是BOYA mini（优先）
                        if 'mini' in line.lower() or 'boya' in line.lower():
                            boya_devices.append((card_num, device_num))
                            print(f"   🎤 发现BOYA mini设备: card {card_num}, device {device_num}")
                        # 检查是否是USB设备（排除APE等内置设备）
                        elif 'usb' in line.lower() or ('device' in line.lower() and card_num < 2):
                            usb_devices.append((card_num, device_num))
            
            # 构建设备候选列表：优先BOYA mini，然后其他USB设备
            device_candidates = []
            
            # 1. 优先BOYA mini设备
            for card_num, device_num in boya_devices:
                device_candidates.append(f'plughw:{card_num},{device_num}')
                device_candidates.append(f'hw:{card_num},{device_num}')
            
            # 2. 然后其他USB设备
            for card_num, device_num in usb_devices:
                # 避免重复添加BOYA设备
                if (card_num, device_num) not in boya_devices:
                    device_candidates.append(f'plughw:{card_num},{device_num}')
                    device_candidates.append(f'hw:{card_num},{device_num}')
            
            if device_candidates:
                total_devices = len(boya_devices) + len(usb_devices)
                print(f"   ✅ 检测到 {total_devices} 个USB设备（{len(boya_devices)} 个BOYA mini）")
            else:
                print("   ⚠️ 未检测到USB设备，将尝试默认设备")
                device_candidates = ['plughw:3,0', 'hw:3,0', 'plughw:0,0', 'hw:0,0', 'default']
    except Exception as e:
        print(f"   ⚠️ 自动检测失败: {e}，使用默认设备列表")
        device_candidates = ['plughw:3,0', 'hw:3,0', 'plughw:0,0', 'hw:0,0', 'default']
    
    # 测试每个设备
    for candidate_device in device_candidates:
        print(f"   💡 尝试设备: {candidate_device}")
        test_result = subprocess.run(['arecord', '-D', candidate_device, '-f', 'S16_LE', '-r', '16000', 
                                      '-c', '1', '-d', '1', '/tmp/test_mic.wav'], 
                                     capture_output=True, timeout=3)
        if test_result.returncode == 0:
            alsa_device = candidate_device
            print(f"   ✅ arecord测试成功，使用设备: {alsa_device}")
            break
        else:
            print(f"   ⚠️ {candidate_device} 测试失败，尝试下一个...")
    
    if alsa_device is None:
        print("   ❌ 所有设备测试均失败")
        print("   💡 提示:")
        print("      1. 检查设备连接: arecord -l")
        print("      2. 检查设备占用: sudo fuser -v /dev/snd/*")
        print("      3. 确认麦克风已插入并开启")
        print("      4. 尝试手动测试: arecord -D plughw:0,0 -d 3 test.wav")
        return
    
    # 创建ffmpeg进程读取音频
    try:
        # 使用硬件原生参数，然后转码和滤波
        # 核心改动：
        # 1. 使用检测到的可用设备（优先plughw:0,0）
        # 2. highpass/lowpass滤波器: 滤掉G1电机电流声（100-3000Hz保留人声频段）
        # 3. volume=3.0: 适度放大（比1.0大，但比15.0小），平衡信号和底噪
        # 4. 最后转码到16kHz单声道给ASR
        ffmpeg_cmd = [
            'ffmpeg',
            '-loglevel', 'quiet',  # 完全静默，只显示致命错误
            '-f', 'alsa',
            '-thread_queue_size', '1024',  # 增加缓冲区避免xrun
            '-i', alsa_device,  # 使用检测到的可用设备
            '-af', 'highpass=f=100,lowpass=f=3000,volume=3.0',  # 滤波器：滤掉电机电流声，保留人声频段，适度放大
            '-ar', str(target_sample_rate),  # 转码到16kHz给ASR
            '-ac', '1',  # 输出单声道
            '-f', 's16le',
            '-threads', '1',
            '-'
        ]
            
        # 使用PIPE捕获stderr以便调试
        audio_process = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, 
                                        stderr=subprocess.PIPE, bufsize=chunk_size)
        
        # 等待一小段时间，检查进程是否正常启动
        time.sleep(0.5)
        if audio_process.poll() is not None:
            # 进程已退出，读取完整错误信息
            try:
                stderr_output = audio_process.stderr.read().decode('utf-8', errors='ignore')
                # 提取关键错误信息
                error_lines = [line for line in stderr_output.split('\n') 
                             if 'error' in line.lower() or 'failed' in line.lower() or 'cannot' in line.lower() or 'busy' in line.lower()]
                if error_lines:
                    error_msg = '\n'.join(error_lines[:5])  # 只显示前5行错误
                else:
                    error_msg = stderr_output[-500:] if len(stderr_output) > 500 else stderr_output
                print(f"   ⚠️ ffmpeg使用{alsa_device}失败:")
                print(f"      {error_msg}")
                print(f"   💡 提示: 设备可能被占用，请等待几秒后重试")
            except:
                print(f"   ⚠️ ffmpeg使用{alsa_device}失败（无法读取错误信息）")
            audio_process = None
        else:
            # 尝试读取一小块数据，确认流正常工作
            try:
                test_data = audio_process.stdout.read(320)  # 读取10ms的数据
                if len(test_data) == 320:
                    print(f"   ✅ ffmpeg音频流已启动（使用{alsa_device}）")
                else:
                    print(f"   ⚠️ ffmpeg使用{alsa_device}启动但无法读取数据")
                    if audio_process:
                        audio_process.terminate()
                        audio_process.wait(timeout=1)
                    audio_process = None
            except Exception as e:
                print(f"   ⚠️ ffmpeg使用{alsa_device}启动但读取测试失败: {e}")
                if audio_process:
                    try:
                        audio_process.terminate()
                        audio_process.wait(timeout=1)
                    except:
                        pass
                audio_process = None
                
    except Exception as e:
        print(f"   ⚠️ ffmpeg启动失败: {e}")
        if audio_process:
            try:
                audio_process.terminate()
                audio_process.wait(timeout=1)
            except:
                pass
        audio_process = None
    
    if audio_process is None:
        print("   ❌ 无法启动ffmpeg音频流")
        print("   💡 提示:")
        print("      1. 请确保ffmpeg已安装: sudo apt-get install ffmpeg")
        print("      2. 检查设备是否被占用: sudo fuser -v /dev/snd/*")
        print("      3. 尝试手动测试: arecord -D plughw:0,0 -d 3 test.wav")
        return
    
    try:
        print()
        print("✅ 系统就绪！")
        print(">>> 豆几已蹲在街口，华子已点燃，随时唠嗑（Ctrl+C 退出）...")
        print()
        
        # 注册信号处理
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        asr_stream = recognizer.create_stream()
        last_result = ""
        is_speaking = False
        start_wait_time = time.time()
        
        # 添加调试计数器
        frame_count = 0
        energy_sum = 0.0
        energy_count = 0
        recent_energies = []  # 用于计算动态阈值
        
        # 累积音频数据（流式ASR可能需要累积一定量数据才能开始解码）
        # Zipformer模型内部有下采样倍数（通常是4倍），需要足够的数据才能进行完整的前向推导
        audio_buffer = []  # 累积多个chunk的音频数据
        buffer_size = 10  # 累积10个chunk（约500ms）再一起喂给ASR，确保有足够数据触发解码
        
        print("💡 提示：对着USB麦克风说话，系统会自动识别")
        print("💡 如果长时间没有反应，请检查麦克风是否正常工作\n")
        
        while not exit_flag:
            try:
                # 检查退出标志
                if exit_flag:
                    break
                
                # 从ffmpeg管道读取音频数据
                try:
                    # 检查进程是否还在运行
                    if audio_process.poll() is not None:
                        # 进程已退出，读取错误信息
                        stderr_output = audio_process.stderr.read().decode()[:500] if audio_process.stderr else "未知错误"
                        print(f"\n❌ ffmpeg进程已退出: {stderr_output}")
                        break
                    
                    raw_data = audio_process.stdout.read(chunk_size)
                    if not raw_data:
                        if exit_flag:
                            break
                        # 如果读取为空，可能是进程结束
                        if audio_process.poll() is not None:
                            stderr_output = audio_process.stderr.read().decode()[:500] if audio_process.stderr else "未知错误"
                            print(f"\n❌ ffmpeg进程已退出: {stderr_output}")
                            break
                        time.sleep(0.01)  # 短暂等待
                        continue
                    
                    if len(raw_data) < chunk_size:
                        # 数据不完整，但继续处理（可能是最后一块数据）
                        if len(raw_data) == 0:
                            continue
                        # 用零填充到完整大小
                        raw_data = raw_data + b'\x00' * (chunk_size - len(raw_data))
                        
                except KeyboardInterrupt:
                    raise
                except Exception as e:
                    if exit_flag:
                        break
                    # 只在第一次错误时打印，避免刷屏
                    if frame_count % 100 == 0:
                        print(f"\n⚠️ 读取音频数据失败: {e}")
                    time.sleep(0.01)
                    continue
                
                # 转换为float32并归一化（ffmpeg输出已经是16kHz单声道S16LE格式）
                # 使用 '<i2' 明确指定小端序（little-endian int16），确保与 ffmpeg -f s16le 匹配
                samples = np.frombuffer(raw_data, dtype='<i2').astype(np.float32) / 32768.0
                
                # 检查退出标志
                if exit_flag:
                    break
                
                # 累积音频数据（参考Ubuntu版本，简化逻辑）
                audio_buffer.append(samples)
                frame_count += 1
                
                # 简单的能量检测（仅用于调试显示，不影响数据累积）
                energy = np.mean(np.abs(samples))
                
                # 每100帧打印一次状态（减少刷屏）
                if frame_count % 100 == 0:
                    buffer_info = f"缓冲区: {len(audio_buffer)}/{buffer_size}"
                    print(f"\r💡 音频能量: {energy:.4f} | {buffer_info} | 帧数: {frame_count}", end="", flush=True)
                
                # 调试：如果缓冲区一直不增长，打印警告
                if frame_count > 200 and len(audio_buffer) == 0:
                    print(f"\n⚠️ 警告：已处理 {frame_count} 帧，但缓冲区仍为空！可能音频数据未正确读取")
                    print(f"   样本数据长度: {len(samples)}, 能量: {energy:.6f}")
                    # 重置计数器，避免重复打印
                    frame_count = 0
                
                # 当累积足够数据时，一起喂给ASR（完全参考Ubuntu版本）
                if len(audio_buffer) >= buffer_size:
                    combined_samples = np.concatenate(audio_buffer)
                    audio_buffer = []
                    
                    # 喂给识别器
                    asr_stream.accept_waveform(target_sample_rate, combined_samples)
                    
                    # 解码
                    decode_count = 0
                    is_ready_status = recognizer.is_ready(asr_stream)
                    
                    max_decode_iterations = 10
                    for _ in range(max_decode_iterations):
                        if is_ready_status:
                            recognizer.decode_stream(asr_stream)
                            decode_count += 1
                            is_ready_status = recognizer.is_ready(asr_stream)
                        else:
                            break
                    
                    # 获取识别结果
                    result = recognizer.get_result(asr_stream)
                    
                    # 处理结果（参考Ubuntu版本）
                    if result is None:
                        text = ""
                    elif isinstance(result, str):
                        text = result.strip()
                    elif hasattr(result, 'text'):
                        text = result.text.strip()
                    else:
                        text = str(result).strip()
                    
                    # 调试：每次缓冲区满都打印结果（帮助诊断）
                    data_rms = np.sqrt(np.mean(combined_samples ** 2))
                    data_max = max(abs(combined_samples.min()), abs(combined_samples.max()))
                    print(f"\n📊 缓冲区已满 | 样本数: {len(combined_samples)} | RMS: {data_rms:.6f} | 最大值: {data_max:.6f} | 解码次数: {decode_count} | is_ready: {is_ready_status} | 结果: '{text}'", end="", flush=True)
                    
                    # 检查是否有新识别结果
                    if text and text != last_result:
                        last_result = text
                        start_wait_time = time.time()
                        is_speaking = True
                        print(f"\n🎤（听到了...）{text}")
                    
                    # 智能触发：说完话就立即回复（参考Ubuntu版本）
                    if is_speaking and text:
                        elapsed = time.time() - start_wait_time
                        should_trigger = elapsed > args.silence_timeout
                        
                        if frame_count % 50 == 0:
                            status = "✅ 准备回复" if should_trigger else "⏳ 等待稳定"
                            print(f"\r{status}... 已等待 {elapsed:.1f}秒 | 文本: '{text}'", end="", flush=True)
                    else:
                        should_trigger = False
                    
                    # 如果满足条件，立即触发回复
                    if is_speaking and text and should_trigger:
                        print(f"\n🚀 触发回复！文本: '{text}'")
                        if exit_flag:
                            break
                        
                        # 停止音频采集
                        if audio_process and audio_process.poll() is None:
                            try:
                                audio_process.terminate()
                                audio_process.wait(timeout=1)
                            except:
                                try:
                                    audio_process.kill()
                                except:
                                    pass
                        
                        print(f"\n【用户】: {text}")
                        
                        # 停止ASR流
                        asr_stream = recognizer.create_stream()
                        
                        # 获取大模型回复
                        if exit_flag:
                            break
                        print("🤖 豆几思考中...")
                        try:
                            reply = get_douji_response(text)
                            if not reply:
                                reply = "哎呀，刚才走神了，再说一遍？"
                            print(f"📝 豆几回复: {reply}")
                        except Exception as e:
                            print(f"⚠️ DeepSeek API调用失败: {e}")
                            reply = "网络有点卡，稍等哈"
                        
                        # 语音播报
                        if exit_flag:
                            break
                        print("🔊 开始播放回复...")
                        speak(reply)
                        print("✅ 播放流程完成")
                        
                        # 等待播放完成
                        if exit_flag:
                            break
                        time.sleep(1.0)
                        
                        # 重新启动音频采集
                        if not exit_flag:
                            try:
                                ffmpeg_cmd = [
                                    'ffmpeg',
                                    '-loglevel', 'quiet',
                                    '-f', 'alsa',
                                    '-thread_queue_size', '1024',
                                    '-i', alsa_device,
                                    '-af', 'highpass=f=100,lowpass=f=3000,volume=3.0',
                                    '-ar', str(target_sample_rate),
                                    '-ac', '1',
                                    '-f', 's16le',
                                    '-threads', '1',
                                    '-'
                                ]
                                audio_process = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, 
                                                                stderr=subprocess.PIPE, bufsize=chunk_size)
                                time.sleep(0.3)
                                if audio_process.poll() is None:
                                    print("   ✅ 音频采集已重启")
                                else:
                                    print("   ⚠️ 音频采集重启失败")
                            except Exception as e:
                                print(f"   ⚠️ 重启音频采集失败: {e}")
                        
                        # 重置ASR状态
                        if exit_flag:
                            break
                        last_result = ""
                        is_speaking = False
                        print("\n>>> 豆几考察完市场了，接着唠：")
                    
            except KeyboardInterrupt:
                print("\n\n收到退出信号...")
                break
            except Exception as e:
                if exit_flag:
                    break
                print(f"\n⚠️ 处理音频时出错: {e}")
                continue
        
    except KeyboardInterrupt:
        print("\n\n豆几夹着皮包走远了...")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 关闭ffmpeg进程
        if audio_process is not None:
            try:
                audio_process.terminate()
                audio_process.wait(timeout=2)
            except:
                try:
                    audio_process.kill()
                except:
                    pass
        print("👋 退出")

if __name__ == "__main__":
    main()

