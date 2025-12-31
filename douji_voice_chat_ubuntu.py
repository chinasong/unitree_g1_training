#!/usr/bin/env python3
"""
Ubuntu系统"豆几"语音对话系统（流式ASR + DeepSeek + Edge-TTS）
整合：Sherpa-ONNX（实时ASR）+ DeepSeek（大模型）+ Edge-TTS（语音输出）
适配Ubuntu系统，无需G1 SDK
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
recognizer = None

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
        
        # 方式1: 使用 from_transducer 方法（推荐）
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
                stream = recognizer.create_stream()
                print("✅ Sherpa-ONNX识别器初始化成功（from_transducer方式）")
                return recognizer
            except Exception as e1:
                print(f"   ⚠️ from_transducer方式失败: {e1}")
                # 尝试简化参数
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
        
        # 方式2: 使用配置类（如果存在）
        if hasattr(sherpa_onnx, 'OnlineModelConfig') and hasattr(sherpa_onnx, 'OnlineTransducerModelConfig'):
            try:
                transducer_config = sherpa_onnx.OnlineTransducerModelConfig(
                    encoder=encoder_path,
                    decoder=decoder_path,
                    joiner=joiner_path,
                )
                
                model_config = sherpa_onnx.OnlineModelConfig(
                    transducer=transducer_config,
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
                        try:
                            recognizer_config = sherpa_onnx.OnlineRecognizerConfig(
                                model_config=model_config,
                                max_active_paths=4,
                            )
                        except TypeError:
                            recognizer_config = sherpa_onnx.OnlineRecognizerConfig(model_config)
                    
                    recognizer = sherpa_onnx.OnlineRecognizer(recognizer_config)
                    print("✅ Sherpa-ONNX识别器初始化成功（配置类方式）")
                    return recognizer
            except Exception as e1:
                print(f"   ⚠️ 配置类方式失败: {e1}")
        
        print("❌ 所有初始化方式均失败")
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
    """Edge-TTS配音并播放（Ubuntu系统使用PulseAudio或ALSA）"""
    print(f"【豆几】: {text}")
    
    temp_mp3 = "/tmp/douji_reply.mp3"
    temp_wav = "/tmp/douji_reply.wav"
    
    print("🔊 正在生成并播放语音...")
    
    # 方法1: 使用edge-tts命令行（最简单可靠）
    success = False
    if subprocess.run(['which', 'edge-tts'], capture_output=True).returncode == 0:
        try:
            # 生成语音文件
            cmd = f'edge-tts --text "{text}" --voice "zh-CN-YunxiNeural" --rate "+20%" > {temp_mp3}'
            result = subprocess.run(cmd, shell=True, timeout=10,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if result.returncode == 0 and os.path.exists(temp_mp3):
                # 转码为wav并播放
                cmd = f'ffmpeg -y -i {temp_mp3} -ar 44100 {temp_wav}'
                result = subprocess.run(cmd, shell=True, timeout=5,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if result.returncode == 0 and os.path.exists(temp_wav):
                    # 尝试使用PulseAudio播放（Ubuntu默认）
                    if subprocess.run(['which', 'paplay'], capture_output=True).returncode == 0:
                        result = subprocess.run(['paplay', temp_wav],
                                              timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    else:
                        # 回退到aplay
                        result = subprocess.run(['aplay', temp_wav],
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
                    if subprocess.run(['which', 'paplay'], capture_output=True).returncode == 0:
                        subprocess.run(['paplay', temp_wav],
                                     timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    else:
                        subprocess.run(['aplay', temp_wav],
                                     timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print("✅ 播放完成")
                    success = True
        except Exception as e:
            print(f"   ⚠️ Python edge-tts失败: {e}")
    
    # 方法3: 使用espeak备用方案
    if not success:
        try:
            print("   ⚠️ 尝试espeak备用方案...")
            wav_file = '/tmp/douji_espeak.wav'
            cmd = ['espeak', '-s', '150', '-v', 'zh', text, '--stdout']
            result = subprocess.run(cmd, capture_output=True, timeout=5)
            if result.returncode == 0 and result.stdout:
                with open(wav_file, 'wb') as f:
                    f.write(result.stdout)
                if subprocess.run(['which', 'paplay'], capture_output=True).returncode == 0:
                    subprocess.run(['paplay', wav_file],
                                 timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                else:
                    subprocess.run(['aplay', wav_file],
                                 timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print("✅ 播放完成（espeak）")
                success = True
        except Exception as e:
            print(f"   ⚠️ espeak也失败: {e}")
    
    # 清理临时文件
    for f in [temp_mp3, temp_wav]:
        if os.path.exists(f):
            try:
                os.remove(f)
            except:
                pass

def find_audio_input_device():
    """查找音频输入设备（Ubuntu系统）"""
    if not HAS_PYAUDIO:
        return None
    
    try:
        p = pyaudio.PyAudio()
        
        # 首先尝试找到默认输入设备
        try:
            default_info = p.get_default_input_device_info()
            if default_info['maxInputChannels'] > 0:
                print(f"✅ 找到默认输入设备: {default_info.get('name', 'unknown')} (索引: {default_info['index']})")
                p.terminate()
                return default_info['index']
        except:
            pass
        
        # 查找所有可用的输入设备
        for i in range(p.get_device_count()):
            try:
                info = p.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    name = info.get('name', '')
                    print(f"✅ 找到输入设备: {name} (索引: {i})")
                    p.terminate()
                    return i
            except:
                continue
        
        p.terminate()
        return None
    except Exception as e:
        print(f"⚠️ 查找音频输入设备失败: {e}")
        return None

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Ubuntu系统"豆几"语音对话系统')
    parser.add_argument('--model-dir', default=MODEL_DIR, help='Sherpa-ONNX模型目录')
    parser.add_argument('--silence-timeout', type=float, default=0.3, help='静音检测超时（秒）')
    parser.add_argument('--audio-device', type=str, default='default', help='音频设备（default/plughw:0,0等）')
    
    args = parser.parse_args()
    
    # 清除代理环境变量
    for key in ['http_proxy', 'https_proxy', 'all_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY']:
        if key in os.environ:
            del os.environ[key]
    
    print("=" * 60)
    print('Ubuntu系统"豆几"语音对话系统')
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
        print("   请设置: export DEEPSEEK_API_KEY='your_key'")
        return
    
    print("🔧 正在初始化...")
    
    # 初始化识别器
    recognizer = init_sherpa_recognizer(args.model_dir)
    if not recognizer:
        return
    
    # 查找音频输入设备
    device_index = find_audio_input_device()
    if device_index is None:
        print("⚠️ 未找到音频输入设备，将尝试使用系统默认设备")
        device_index = None
    
    # 使用ffmpeg管道读取音频（更可靠）
    print("🔧 使用ffmpeg管道读取音频...")
    
    # ASR需要16kHz单声道
    target_sample_rate = 16000
    chunk_size = 1600  # 50ms的数据块
    
    # 确定音频设备
    if args.audio_device == 'default':
        # 尝试检测默认设备
        alsa_device = 'default'
        # 检查是否有USB设备
        arecord_result = subprocess.run(['arecord', '-l'], capture_output=True, text=True, timeout=2)
        if arecord_result.returncode == 0:
            for line in arecord_result.stdout.split('\n'):
                if 'card' in line.lower() and ('usb' in line.lower() or 'device' in line.lower()):
                    # 提取card编号
                    import re
                    match = re.search(r'card (\d+)', line)
                    if match:
                        card_num = match.group(1)
                        alsa_device = f'plughw:{card_num},0'
                        print(f"   ✅ 检测到USB设备，使用: {alsa_device}")
                        break
    else:
        alsa_device = args.audio_device
    
    # 测试arecord
    print("   📋 测试音频设备...")
    test_result = subprocess.run(['arecord', '-D', alsa_device, '-f', 'S16_LE', '-r', '16000', 
                                  '-c', '1', '-d', '1', '/tmp/test_mic.wav'], 
                                 capture_output=True, timeout=3)
    if test_result.returncode == 0:
        print(f"   ✅ arecord测试成功，音频设备可用: {alsa_device}")
    else:
        print(f"   ⚠️ arecord测试失败，尝试使用default设备")
        alsa_device = 'default'
    
    # 创建ffmpeg进程读取音频
    audio_process = None
    
    try:
        ffmpeg_cmd = [
            'ffmpeg',
            '-loglevel', 'quiet',
            '-f', 'alsa',
            '-thread_queue_size', '1024',
            '-i', alsa_device,
            '-af', 'highpass=f=100,lowpass=f=3000,volume=3.0',  # 滤波器+适度增益
            '-ar', str(target_sample_rate),
            '-ac', '1',
            '-f', 's16le',
            '-threads', '1',
            '-'
        ]
        
        audio_process = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, 
                                        stderr=subprocess.PIPE, bufsize=chunk_size)
        
        # 等待进程启动
        time.sleep(0.5)
        if audio_process.poll() is not None:
            stderr_output = audio_process.stderr.read().decode('utf-8', errors='ignore')[:500]
            print(f"   ⚠️ ffmpeg启动失败: {stderr_output}")
            audio_process = None
        else:
            # 测试读取数据
            try:
                test_data = audio_process.stdout.read(320)
                if len(test_data) == 320:
                    print(f"   ✅ ffmpeg音频流已启动（使用{alsa_device}）")
                else:
                    print(f"   ⚠️ ffmpeg启动但无法读取数据")
                    audio_process.terminate()
                    audio_process.wait(timeout=1)
                    audio_process = None
            except Exception as e:
                print(f"   ⚠️ ffmpeg读取测试失败: {e}")
                if audio_process:
                    audio_process.terminate()
                    audio_process.wait(timeout=1)
                audio_process = None
                
    except Exception as e:
        print(f"   ⚠️ ffmpeg启动失败: {e}")
        audio_process = None
    
    if audio_process is None:
        print("   ❌ 无法启动ffmpeg音频流")
        print("   💡 提示:")
        print("      1. 请确保ffmpeg已安装: sudo apt-get install ffmpeg")
        print("      2. 检查音频设备: arecord -l")
        print("      3. 尝试手动测试: arecord -D default -d 3 test.wav")
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
        
        # 调试计数器
        frame_count = 0
        audio_buffer = []
        buffer_size = 10  # 累积10个chunk（约500ms）
        
        print("💡 提示：对着麦克风说话，系统会自动识别")
        print("💡 如果长时间没有反应，请检查麦克风是否正常工作\n")
        
        while not exit_flag:
            try:
                if exit_flag:
                    break
                
                # 从ffmpeg管道读取音频数据
                try:
                    if audio_process.poll() is not None:
                        stderr_output = audio_process.stderr.read().decode()[:500] if audio_process.stderr else "未知错误"
                        print(f"\n❌ ffmpeg进程已退出: {stderr_output}")
                        break
                    
                    raw_data = audio_process.stdout.read(chunk_size)
                    if not raw_data:
                        if exit_flag:
                            break
                        if audio_process.poll() is not None:
                            stderr_output = audio_process.stderr.read().decode()[:500] if audio_process.stderr else "未知错误"
                            print(f"\n❌ ffmpeg进程已退出: {stderr_output}")
                            break
                        time.sleep(0.01)
                        continue
                    
                    if len(raw_data) < chunk_size:
                        if len(raw_data) == 0:
                            continue
                        raw_data = raw_data + b'\x00' * (chunk_size - len(raw_data))
                        
                except KeyboardInterrupt:
                    raise
                except Exception as e:
                    if exit_flag:
                        break
                    if frame_count % 100 == 0:
                        print(f"\n⚠️ 读取音频数据失败: {e}")
                    time.sleep(0.01)
                    continue
                
                # 转换为float32并归一化
                samples = np.frombuffer(raw_data, dtype='<i2').astype(np.float32) / 32768.0
                
                if exit_flag:
                    break
                
                # 累积音频数据
                audio_buffer.append(samples)
                frame_count += 1
                
                # 当累积足够数据时，一起喂给ASR
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
                    
                    # 处理结果
                    if result is None:
                        text = ""
                    elif isinstance(result, str):
                        text = result.strip()
                    elif hasattr(result, 'text'):
                        text = result.text.strip()
                    else:
                        text = str(result).strip()
                    
                    # 检查是否有新识别结果
                    if text and text != last_result:
                        last_result = text
                        start_wait_time = time.time()
                        is_speaking = True
                        print(f"\n🎤（听到了...）{text}")
                    
                    # 智能触发：说完话就立即回复
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

