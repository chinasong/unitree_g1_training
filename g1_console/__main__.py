import argparse
from dataclasses import replace
import logging
import threading
import webbrowser
import uvicorn
from .app import create_app
from .config import Settings


def main():
    parser = argparse.ArgumentParser(description="G1 EDU 23 图形身体控制台")
    parser.add_argument("--demo", action="store_true", help="启动明确标记的合成演示环境；不会加载 DDS")
    parser.add_argument("--read-only", action="store_true", help="仅订阅身体状态和查询能力；禁止所有控制写入")
    parser.add_argument("--manual-remote", action="store_true", help="启用人工遥控授权入口；不会自动取得控制权或执行动作")
    parser.add_argument("--verify-only", action="store_true", help="人工遥控接口验证：允许控制权租约和参数核对，禁止全部实机写入")
    parser.add_argument("--accessory-test", action="store_true", help="仅开放官方语音和 RGB 灯光测试；禁止全部电机、模式、音量和音频流写入")
    parser.add_argument("--damping-test", action="store_true", help="仅允许一次从零力矩切换官方阻尼；不开放起身、站立或行走")
    parser.add_argument("--open", action="store_true", help="自动打开图形界面")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    settings = Settings(demo=args.demo)
    if args.read_only:
        settings = replace(settings, read_only=True)
    if args.manual_remote:
        settings = replace(settings, manual_remote=True)
    if args.verify_only:
        settings = replace(settings, verify_only=True)
    if args.accessory_test:
        settings = replace(settings, accessory_test=True)
    if args.damping_test:
        settings = replace(settings, damping_test=True)
    if args.port:
        settings = replace(settings, port=args.port)
    settings.validate()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if args.open:
        threading.Timer(1.2, lambda: webbrowser.open(f"http://127.0.0.1:{settings.port}")).start()
    uvicorn.run(create_app(settings), host=settings.host, port=settings.port, access_log=False, workers=1)


if __name__ == "__main__":
    main()
