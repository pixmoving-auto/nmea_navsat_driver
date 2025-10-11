#!/usr/bin/env python3
"""
测试自动串口检测功能
"""

import sys
import os

# 添加包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from libnmea_navsat_driver.nodes.nmea_serial_driver import find_gps_ports, test_port, find_working_gps_port

def main():
    print("=== NMEA GPS 自动串口检测测试 ===\n")
    
    # 1. 查找所有可能的端口
    print("1. 查找所有串口设备:")
    ports = find_gps_ports()
    if ports:
        for i, port in enumerate(ports, 1):
            print(f"   {i}. {port}")
    else:
        print("   未找到任何串口设备")
    
    print(f"\n总共找到 {len(ports)} 个串口设备\n")
    
    # 2. 测试每个端口
    print("2. 测试端口可用性:")
    working_ports = []
    for port in ports:
        print(f"   测试 {port}...", end=" ")
        if test_port(port):
            print("✓ 可用")
            working_ports.append(port)
        else:
            print("✗ 不可用")
    
    print(f"\n找到 {len(working_ports)} 个可用端口: {', '.join(working_ports) if working_ports else '无'}\n")
    
    # 3. 使用自动检测函数
    print("3. 自动检测最佳端口:")
    best_port = find_working_gps_port()
    if best_port:
        print(f"   推荐端口: {best_port}")
    else:
        print("   未找到可用端口")
    
    print("\n=== 测试完成 ===")

if __name__ == '__main__':
    main()
