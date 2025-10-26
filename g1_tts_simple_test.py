#!/usr/bin/env python3
"""
G1 TTS 最简单的使用示例
直接连接机器人，无需启动服务器
"""

import sys
import time

sys.path.append('./externals/unitree_sdk2_python')
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


def main():
    """最简单的TTS示例"""
    
    # 检查参数
    if len(sys.argv) < 2:
        print("使用方法: python3 g1_tts_simple_test.py <网络接口>")
        print("例如: python3 g1_tts_simple_test.py eth0")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    
    print("🤖 初始化G1机器人...")
    
    # 初始化通信
    ChannelFactoryInitialize(0, network_interface)
    
    # 创建音频客户端
    audio = AudioClient()
    audio.SetTimeout(10.0)
    audio.Init()
    
    print("✅ 连接成功！\n")
    
    # 测试1: 说中文
    print("测试1: 说中文")
    audio.TtsMaker("你好，我是宇树科技G1人形机器人！", 0)
    time.sleep(3)
    
    # 测试2: 说英文
    print("测试2: 说英文")
    audio.TtsMaker("Hello! I am Unitree G1 humanoid robot.", 0)
    time.sleep(3)
    
    # 测试3: 调整音量
    print("测试3: 调整音量")
    audio.SetVolume(100)
    audio.TtsMaker("现在是最大音量", 0)
    time.sleep(2)
    
    audio.SetVolume(50)
    audio.TtsMaker("现在是中等音量", 0)
    time.sleep(2)
    
    # 测试4: LED灯光
    print("测试4: LED灯光效果")
    audio.TtsMaker("接下来测试LED灯光", 0)
    time.sleep(2)
    
    print("  红色...")
    audio.LedControl(255, 0, 0)
    time.sleep(1)
    
    print("  绿色...")
    audio.LedControl(0, 255, 0)
    time.sleep(1)
    
    print("  蓝色...")
    audio.LedControl(0, 0, 255)
    time.sleep(1)
    
    # 测试5: 说长句子
    print("测试5: 说长句子")
    long_text = """
    宇树科技G1是一款先进的人形机器人，
    具有语音交互、视觉识别、自主导航等多种功能。
    我很高兴能够为您服务！
    """
    audio.TtsMaker(long_text.strip(), 0)
    time.sleep(8)
    
    # 完成
    audio.TtsMaker("所有测试完成，谢谢！", 0)
    time.sleep(2)
    
    print("\n✅ 所有测试完成！")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 程序中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

