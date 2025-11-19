#!/usr/bin/env python3
"""
快速GPSD测试脚本 - 简化版本
用于快速查看GNSS和IMU数据、底盘车速数据等
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
            if not line:
                continue
            line = line.strip()

            # 始终输出GPSD的原始NMEA/PQTM行
            print(line)

            # 同时对常见语句做醒目标注（不影响原始输出）
            try:
                if line.startswith('$GNGGA'):
                    print(f"📍 GGA位置↑")
                elif line.startswith('$GNRMC'):
                    print(f"🧭 RMC导航↑")
                elif line.startswith('$PQTMPVT'):
                    print(f"🛰️  PVT数据↑")
                elif line.startswith('$PQTMSENMSG'):
                    print(f"📊 IMU数据↑")
                elif line.startswith('$GNGLL'):
                    print(f"🌍 GLL位置↑")
                elif line.startswith('$GNVTG'):
                    print(f"🚗 VTG速度信息↑")
                elif line.startswith('$PQTMDRPVA'):
                    print(f"🔍 DRPVA数据↑")
                elif line.startswith('$PQTMVEHMSG'):
                    print(f"🚗 VEHMSG底盘车速↑")
            except Exception:
                # 保障打印不中断
                pass
    
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
