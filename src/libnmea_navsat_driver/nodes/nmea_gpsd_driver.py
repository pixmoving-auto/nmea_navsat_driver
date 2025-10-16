# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import subprocess
import threading
import time
from typing import Optional

import rclpy
from libnmea_navsat_driver.driver import Ros2NMEADriver


class GPSDReader:
    """
    GPSD数据读取器，通过gpspipe命令获取NMEA数据
    """
    
    def __init__(self, timeout: float = 2.0):
        self.timeout = timeout
        self.process = None
        self.running = False
        self.lock = threading.Lock()
        
    def open(self) -> bool:
        """
        启动gpspipe进程
        """
        try:
            # 启动gpspipe进程
            self.process = subprocess.Popen(
                ['gpspipe', '-r'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            self.running = True
            return True
            
        except FileNotFoundError:
            print("错误: 找不到gpspipe命令。请确保已安装gpsd工具。")
            return False
        except Exception as e:
            print(f"启动gpspipe进程时出错: {e}")
            return False
    
    def close(self):
        """
        关闭gpspipe进程
        """
        with self.lock:
            self.running = False
            if self.process:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                self.process = None
    
    def readline(self) -> Optional[str]:
        """
        读取一行NMEA数据
        """
        if not self.process or not self.running:
            return None
            
        try:
            with self.lock:
                if not self.running or not self.process:
                    return None
                    
                # 检查进程是否还在运行
                if self.process.poll() is not None:
                    return None
                
                # 读取一行数据
                line = self.process.stdout.readline()
                if line:
                    line = line.strip()
                    
                    # 跳过非NMEA格式的数据（如JSON设备信息）
                    if not line.startswith('$'):
                        return None
                    
                    # 确保NMEA句子格式正确
                    if line and not line.startswith('$'):
                        line = '$' + line
                    return line
                    
                return None
                
        except Exception as e:
            print(f"从gpspipe读取数据时出错: {e}")
            return None
    
    def read(self, size: int = 1) -> Optional[str]:
        """
        读取指定数量的字符
        """
        if not self.process or not self.running:
            return None
            
        try:
            with self.lock:
                if not self.running or not self.process:
                    return None
                    
                # 检查进程是否还在运行
                if self.process.poll() is not None:
                    return None
                
                # 读取指定数量的字符
                data = self.process.stdout.read(size)
                return data
                
        except Exception as e:
            print(f"从gpspipe读取数据时出错: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    frame_id = driver.get_frame_id()

    # 获取配置参数 - 与现有架构保持一致
    timeout = driver.declare_parameter('timeout', 2.0).value
    auto_detect = driver.declare_parameter('auto_detect', True).value

    # 创建GPSD读取器
    gpsd_reader = GPSDReader(timeout)
    
    if not gpsd_reader.open():
        driver.get_logger().fatal("Could not start gpspipe process. Please ensure GPSD is running and GPS device is connected.")
        return

    driver.get_logger().info("Successfully connected to GPSD via gpspipe")
    driver.get_logger().info("GPSD driver is now reading NMEA data from GPS device")
    
    try:
        while rclpy.ok():
            data = gpsd_reader.readline()
            if data:
                try:
                    # 使用现有的解析逻辑
                    driver.add_sentence(data, frame_id)
                except ValueError as e:
                    driver.get_logger().warn(
                        "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
                        "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file "
                        "with the NMEA sentences that caused it." % e)
            else:
                # 如果没有数据，短暂休眠避免CPU占用过高
                time.sleep(0.001)

    except KeyboardInterrupt:
        driver.get_logger().info("GPSD driver stopped by user")
    except Exception as e:
        driver.get_logger().error("GPSD driver error: {0}".format(e))
    finally:
        gpsd_reader.close()
        driver.get_logger().info("GPSD connection closed")


if __name__ == '__main__':
    main()
