#!/usr/bin/env python3
"""
快速GPSD测试脚本 - 简化版本
用于快速查看GNSS和IMU数据
"""

import subprocess
import signal
import sys
import time

def signal_handler(signum, frame):
    print("\n正在停止...")
    sys.exit(0)

def main():
    print("=== 快速GPSD数据测试 ===")
    print("按 Ctrl+C 停止\n")
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        process = subprocess.Popen(
            ['gpspipe', '-r'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        while True:
            line = process.stdout.readline()
            if line:
                line = line.strip()
                
                # 只显示重要的数据
                if line.startswith('$GNGGA'):
                    print(f"📍 GPS位置: {line}")
                elif line.startswith('$GNRMC'):
                    print(f"🧭 GPS导航: {line}")
                elif line.startswith('$PQTMPVT'):
                    print(f"🛰️  PVT数据: {line}")
                elif line.startswith('$PQTMSENMSG'):
                    print(f"📊 IMU数据: {line}")
                elif line.startswith('$GNGLL'):
                    print(f"🌍 位置信息: {line}")
                elif line.startswith('$GNVTG'):
                    print(f"🚗 速度信息: {line}")
    
    except FileNotFoundError:
        print("错误: 找不到gpspipe命令。请确保已安装gpsd工具。")
    except KeyboardInterrupt:
        print("\n测试已停止")
    finally:
        if 'process' in locals():
            process.terminate()
            process.wait()

if __name__ == "__main__":
    main()
