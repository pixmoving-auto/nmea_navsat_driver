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

import serial
import glob
import os
import time

import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver


def find_gps_ports():
    """
    查找可能的GPS串口设备
    返回按优先级排序的端口列表
    """
    gps_ports = []
    
    # 检查常见的USB串口设备
    usb_patterns = [
        '/dev/ttyUSB*',
        '/dev/ttyACM*',
        '/dev/ttyAMA*'
    ]
    
    for pattern in usb_patterns:
        ports = glob.glob(pattern)
        for port in ports:
            if os.path.exists(port):
                gps_ports.append(port)
    
    # 按优先级排序 (USB设备优先)
    priority_order = ['ttyUSB', 'ttyACM', 'ttyAMA', 'ttyS']
    def get_priority(port):
        for i, prefix in enumerate(priority_order):
            if prefix in port:
                return i
        return len(priority_order)
    
    gps_ports.sort(key=get_priority)
    return gps_ports


def test_port(port, baud_rate=460800, timeout=2):
    """
    测试串口是否可用
    """
    try:
        with serial.Serial(port, baud_rate, timeout=timeout) as ser:
            # 尝试读取一些数据来验证设备是否响应
            time.sleep(0.5)
            data = ser.read(100)
            if data:
                return True
            else:
                return False
    except Exception:
        return False


def find_working_gps_port(baud_rate=460800, logger=None):
    """
    查找可用的GPS端口
    """
    if logger:
        logger.info("正在搜索可用的串口设备...")
    
    # 查找所有可能的端口
    all_ports = find_gps_ports()
    
    if not all_ports:
        if logger:
            logger.error("未找到任何串口设备")
        return None
    
    if logger:
        logger.info(f"发现 {len(all_ports)} 个串口设备: {', '.join(all_ports)}")
    
    # 测试每个端口
    for port in all_ports:
        if logger:
            logger.info(f"测试端口: {port}")
        if test_port(port, baud_rate):
            if logger:
                logger.info(f"✓ 找到可用端口: {port}")
            return port
    
    if logger:
        logger.error("未找到可用的GPS设备")
    return None


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    frame_id = driver.get_frame_id()

    # 获取配置参数
    auto_detect = driver.declare_parameter('auto_detect', True).value
    serial_port = driver.declare_parameter('port', '/dev/ttyUSB1').value
    serial_baud = driver.declare_parameter('baud', 460800).value

    # 如果启用自动检测，尝试自动查找可用端口
    if auto_detect:
        driver.get_logger().info("启用自动串口检测...")
        detected_port = find_working_gps_port(serial_baud, driver.get_logger())
        if detected_port:
            serial_port = detected_port
            driver.get_logger().info(f"自动检测到GPS设备: {serial_port}")
        else:
            driver.get_logger().warn(f"自动检测失败，使用配置的端口: {serial_port}")

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
        driver.get_logger().info("Successfully connected to {0} at {1}.".format(serial_port, serial_baud))
        try:
            while rclpy.ok():
                data = GPS.readline().strip()
                try:
                    if isinstance(data, bytes):
                        data = data.decode("utf-8")
                    driver.add_sentence(data, frame_id)
                except ValueError as e:
                    driver.get_logger().warn(
                        "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
                        "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file "
                        "with the NMEA sentences that caused it." % e)

        except Exception as e:
            driver.get_logger().error("Ros error: {0}".format(e))
            GPS.close()  # Close GPS serial port
    except serial.SerialException as ex:
        driver.get_logger().fatal("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
