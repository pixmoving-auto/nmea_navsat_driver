#!/usr/bin/env python3
"""
GPSD测试脚本 - 获取并解析GNSS和IMU数据
支持从gpspipe命令获取数据并解析NMEA格式的GPS数据和自定义IMU数据
"""

import subprocess
import json
import re
import time
import signal
import sys
from datetime import datetime
from typing import Dict, Any, Optional

class GPSDDataParser:
    """GPSD数据解析器"""
    
    def __init__(self):
        self.gnss_data = {}
        self.imu_data = {}
        self.running = True
        
    def parse_nmea_sentence(self, sentence: str) -> Optional[Dict[str, Any]]:
        """解析NMEA句子"""
        try:
            if sentence.startswith('$GNGGA'):
                return self.parse_gga(sentence)
            elif sentence.startswith('$GNRMC'):
                return self.parse_rmc(sentence)
            elif sentence.startswith('$GNGLL'):
                return self.parse_gll(sentence)
            elif sentence.startswith('$GNVTG'):
                return self.parse_vtg(sentence)
            elif sentence.startswith('$GNGST'):
                return self.parse_gst(sentence)
            elif sentence.startswith('$PQTMPVT'):
                return self.parse_pqtmpvt(sentence)
            elif sentence.startswith('$PQTMSENMSG'):
                return self.parse_pqtmsenmsg(sentence)
        except Exception as e:
            print(f"解析NMEA句子时出错: {e}")
        return None
    
    def parse_gga(self, sentence: str) -> Dict[str, Any]:
        """解析GGA句子 - 全球定位系统定位数据"""
        parts = sentence.split(',')
        if len(parts) < 15:
            return {}
        
        return {
            'type': 'GGA',
            'time': parts[1] if parts[1] else None,
            'latitude': self.parse_coordinate(parts[2], parts[3]) if parts[2] and parts[3] else None,
            'longitude': self.parse_coordinate(parts[4], parts[5]) if parts[4] and parts[5] else None,
            'quality': parts[6] if parts[6] else None,
            'satellites': int(parts[7]) if parts[7] else 0,
            'hdop': float(parts[8]) if parts[8] else None,
            'altitude': float(parts[9]) if parts[9] else None,
            'altitude_units': parts[10] if parts[10] else None,
            'geoid_height': float(parts[11]) if parts[11] else None,
            'geoid_units': parts[12] if parts[12] else None,
            'age': parts[13] if parts[13] else None,
            'station_id': parts[14].split('*')[0] if parts[14] else None
        }
    
    def parse_rmc(self, sentence: str) -> Dict[str, Any]:
        """解析RMC句子 - 推荐最小定位信息"""
        parts = sentence.split(',')
        if len(parts) < 12:
            return {}
        
        return {
            'type': 'RMC',
            'time': parts[1] if parts[1] else None,
            'status': parts[2] if parts[2] else None,
            'latitude': self.parse_coordinate(parts[3], parts[4]) if parts[3] and parts[4] else None,
            'longitude': self.parse_coordinate(parts[5], parts[6]) if parts[5] and parts[6] else None,
            'speed': float(parts[7]) if parts[7] else None,
            'course': float(parts[8]) if parts[8] else None,
            'date': parts[9] if parts[9] else None,
            'magnetic_variation': float(parts[10]) if parts[10] else None,
            'variation_direction': parts[11].split('*')[0] if parts[11] else None
        }
    
    def parse_gll(self, sentence: str) -> Dict[str, Any]:
        """解析GLL句子 - 地理定位信息"""
        parts = sentence.split(',')
        if len(parts) < 7:
            return {}
        
        return {
            'type': 'GLL',
            'latitude': self.parse_coordinate(parts[1], parts[2]) if parts[1] and parts[2] else None,
            'longitude': self.parse_coordinate(parts[3], parts[4]) if parts[3] and parts[4] else None,
            'time': parts[5] if parts[5] else None,
            'status': parts[6].split('*')[0] if parts[6] else None
        }
    
    def parse_vtg(self, sentence: str) -> Dict[str, Any]:
        """解析VTG句子 - 地面速度信息"""
        parts = sentence.split(',')
        if len(parts) < 10:
            return {}
        
        return {
            'type': 'VTG',
            'course_true': float(parts[1]) if parts[1] else None,
            'course_magnetic': float(parts[3]) if parts[3] else None,
            'speed_knots': float(parts[5]) if parts[5] else None,
            'speed_kmh': float(parts[7]) if parts[7] else None,
            'mode': parts[9].split('*')[0] if parts[9] else None
        }
    
    def parse_gst(self, sentence: str) -> Dict[str, Any]:
        """解析GST句子 - GPS伪距噪声统计"""
        parts = sentence.split(',')
        if len(parts) < 8:
            return {}
        
        return {
            'type': 'GST',
            'time': parts[1] if parts[1] else None,
            'range_rms': float(parts[2]) if parts[2] else None,
            'std_major': float(parts[3]) if parts[3] else None,
            'std_minor': float(parts[4]) if parts[4] else None,
            'orientation': float(parts[5]) if parts[5] else None,
            'std_lat': float(parts[6]) if parts[6] else None,
            'std_lon': float(parts[7].split('*')[0]) if parts[7] else None
        }
    
    def parse_pqtmpvt(self, sentence: str) -> Dict[str, Any]:
        """解析PQTMPVT句子 - 自定义PVT数据"""
        parts = sentence.split(',')
        if len(parts) < 20:
            return {}
        
        return {
            'type': 'PQTMPVT',
            'message_id': parts[1] if parts[1] else None,
            'timestamp': parts[2] if parts[2] else None,
            'date': parts[3] if parts[3] else None,
            'time': parts[4] if parts[4] else None,
            'fix_type': int(parts[5]) if parts[5] else None,
            'num_satellites': int(parts[6]) if parts[6] else None,
            'latitude': float(parts[7]) if parts[7] else None,
            'longitude': float(parts[8]) if parts[8] else None,
            'altitude': float(parts[9]) if parts[9] else None,
            'north_velocity': float(parts[10]) if parts[10] else None,
            'east_velocity': float(parts[11]) if parts[11] else None,
            'up_velocity': float(parts[12]) if parts[12] else None,
            'north_acceleration': float(parts[13]) if parts[13] else None,
            'east_acceleration': float(parts[14]) if parts[14] else None,
            'up_acceleration': float(parts[15]) if parts[15] else None,
            'heading': float(parts[16]) if parts[16] else None,
            'heading_accuracy': float(parts[17]) if parts[17] else None,
            'speed_accuracy': float(parts[18].split('*')[0]) if parts[18] else None
        }
    
    def parse_pqtmsenmsg(self, sentence: str) -> Dict[str, Any]:
        """解析PQTMSENMSG句子 - 自定义IMU传感器数据"""
        parts = sentence.split(',')
        if len(parts) < 10:
            return {}
        
        return {
            'type': 'PQTMSENMSG',
            'message_id': parts[1] if parts[1] else None,
            'timestamp': parts[2] if parts[2] else None,
            'temperature': float(parts[3]) if parts[3] else None,
            'accel_x': float(parts[4]) if parts[4] else None,
            'accel_y': float(parts[5]) if parts[5] else None,
            'accel_z': float(parts[6]) if parts[6] else None,
            'gyro_x': float(parts[7]) if parts[7] else None,
            'gyro_y': float(parts[8]) if parts[8] else None,
            'gyro_z': float(parts[9].split('*')[0]) if parts[9] else None
        }
    
    def parse_coordinate(self, coord_str: str, direction: str) -> Optional[float]:
        """解析坐标字符串"""
        if not coord_str or not direction:
            return None
        
        try:
            # 将DDMM.MMMM格式转换为十进制度数
            coord = float(coord_str)
            degrees = int(coord // 100)
            minutes = coord % 100
            decimal_degrees = degrees + minutes / 60.0
            
            if direction.upper() in ['S', 'W']:
                decimal_degrees = -decimal_degrees
            
            return decimal_degrees
        except (ValueError, TypeError):
            return None
    
    def format_gnss_data(self, data: Dict[str, Any]) -> str:
        """格式化GNSS数据显示"""
        if not data:
            return ""
        
        output = f"\n=== GNSS数据 ({data.get('type', 'Unknown')}) ===\n"
        
        if data.get('type') == 'GGA':
            output += f"时间: {data.get('time', 'N/A')}\n"
            output += f"纬度: {data.get('latitude', 'N/A'):.8f}°\n"
            output += f"经度: {data.get('longitude', 'N/A'):.8f}°\n"
            output += f"定位质量: {data.get('quality', 'N/A')}\n"
            output += f"卫星数量: {data.get('satellites', 'N/A')}\n"
            output += f"水平精度因子: {data.get('hdop', 'N/A')}\n"
            output += f"海拔高度: {data.get('altitude', 'N/A')} {data.get('altitude_units', '')}\n"
        
        elif data.get('type') == 'RMC':
            output += f"时间: {data.get('time', 'N/A')}\n"
            output += f"状态: {data.get('status', 'N/A')}\n"
            output += f"纬度: {data.get('latitude', 'N/A'):.8f}°\n"
            output += f"经度: {data.get('longitude', 'N/A'):.8f}°\n"
            output += f"速度: {data.get('speed', 'N/A')} 节\n"
            output += f"航向: {data.get('course', 'N/A')}°\n"
            output += f"日期: {data.get('date', 'N/A')}\n"
        
        elif data.get('type') == 'PQTMPVT':
            output += f"时间戳: {data.get('timestamp', 'N/A')}\n"
            output += f"日期: {data.get('date', 'N/A')}\n"
            output += f"时间: {data.get('time', 'N/A')}\n"
            output += f"定位类型: {data.get('fix_type', 'N/A')}\n"
            output += f"卫星数量: {data.get('num_satellites', 'N/A')}\n"
            output += f"纬度: {data.get('latitude', 'N/A'):.8f}°\n"
            output += f"经度: {data.get('longitude', 'N/A'):.8f}°\n"
            output += f"海拔: {data.get('altitude', 'N/A')} 米\n"
            output += f"北向速度: {data.get('north_velocity', 'N/A')} m/s\n"
            output += f"东向速度: {data.get('east_velocity', 'N/A')} m/s\n"
            output += f"垂直速度: {data.get('up_velocity', 'N/A')} m/s\n"
            output += f"航向: {data.get('heading', 'N/A')}°\n"
        
        return output
    
    def format_imu_data(self, data: Dict[str, Any]) -> str:
        """格式化IMU数据显示"""
        if not data or data.get('type') != 'PQTMSENMSG':
            return ""
        
        output = f"\n=== IMU数据 ===\n"
        output += f"时间戳: {data.get('timestamp', 'N/A')}\n"
        output += f"温度: {data.get('temperature', 'N/A')}°C\n"
        output += f"加速度 X: {data.get('accel_x', 'N/A')} g\n"
        output += f"加速度 Y: {data.get('accel_y', 'N/A')} g\n"
        output += f"加速度 Z: {data.get('accel_z', 'N/A')} g\n"
        output += f"角速度 X: {data.get('gyro_x', 'N/A')} rad/s\n"
        output += f"角速度 Y: {data.get('gyro_y', 'N/A')} rad/s\n"
        output += f"角速度 Z: {data.get('gyro_z', 'N/A')} rad/s\n"
        
        return output
    
    def signal_handler(self, signum, frame):
        """信号处理器"""
        print("\n\n正在停止GPSD数据获取...")
        self.running = False
    
    def run_gpsd_test(self, duration: int = 60):
        """运行GPSD测试"""
        print("=== GPSD数据获取测试脚本 ===")
        print(f"开始获取GPSD数据，持续 {duration} 秒...")
        print("按 Ctrl+C 提前停止\n")
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        
        try:
            # 启动gpspipe进程
            process = subprocess.Popen(
                ['gpspipe', '-r'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            start_time = time.time()
            line_count = 0
            
            while self.running and (time.time() - start_time) < duration:
                if process.poll() is not None:
                    print("gpspipe进程已结束")
                    break
                
                try:
                    line = process.stdout.readline()
                    if line:
                        line = line.strip()
                        line_count += 1
                        
                        # 解析NMEA句子
                        parsed_data = self.parse_nmea_sentence(line)
                        
                        if parsed_data:
                            # 显示GNSS数据
                            if parsed_data.get('type') in ['GGA', 'RMC', 'GLL', 'VTG', 'GST', 'PQTMPVT']:
                                print(self.format_gnss_data(parsed_data))
                            
                            # 显示IMU数据
                            if parsed_data.get('type') == 'PQTMSENMSG':
                                print(self.format_imu_data(parsed_data))
                        
                        # 每10行显示一次原始数据（用于调试）
                        if line_count % 10 == 0:
                            print(f"\n--- 原始数据 (第{line_count}行) ---")
                            print(f"{line}")
                            print("-" * 50)
                
                except Exception as e:
                    print(f"读取数据时出错: {e}")
                    continue
            
            # 清理进程
            process.terminate()
            process.wait()
            
            print(f"\n=== 测试完成 ===")
            print(f"总共处理了 {line_count} 行数据")
            print(f"运行时间: {time.time() - start_time:.2f} 秒")
            
        except FileNotFoundError:
            print("错误: 找不到gpspipe命令。请确保已安装gpsd工具。")
        except Exception as e:
            print(f"运行测试时出错: {e}")

def main():
    """主函数"""
    parser = GPSDDataParser()
    
    # 获取运行时间参数
    duration = 60  # 默认60秒
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print("警告: 无效的时间参数，使用默认值60秒")
    
    # 运行测试
    parser.run_gpsd_test(duration)

if __name__ == "__main__":
    main()
