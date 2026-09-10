# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Rein Appeldoorn
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

import socket
import sys
import time

import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver


def main(args=None):
    rclpy.init(args=args)
    driver = Ros2NMEADriver()

    try:
        local_ip = driver.declare_parameter('ip', '0.0.0.0').value
        local_port = driver.declare_parameter('port', 10110).value
        buffer_size = driver.declare_parameter('buffer_size', 4096).value
        timeout = driver.declare_parameter('timeout_sec', 2).value
        # protocol: udp=绑定本机端口接收(UDP惯导), tcp=主动连接惯导设备(TCP惯导)
        protocol = driver.declare_parameter('protocol', 'udp').value
    except KeyError as e:
        driver.get_logger().err("Parameter %s not found" % e)
        sys.exit(1)

    frame_id = driver.get_frame_id()

    driver.get_logger().info(
        " Using parameters ip {} port {} buffer_size {} timeout_sec {} protocol {}"
        .format(local_ip, local_port, buffer_size, timeout, protocol))

    # Connection-loop: connect and keep receiving. If receiving fails, reconnect
    while rclpy.ok():
        socket_ = None
        try:
            # Create a socket
            socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM if protocol == 'tcp' else socket.SOCK_DGRAM)

            # Set timeout
            socket_.settimeout(timeout)

            if protocol == 'tcp':
                # TCP client: 主动连接惯导设备
                socket_.connect((local_ip, local_port))
                driver.get_logger().info("TCP connected to {}:{}".format(local_ip, local_port))
            else:
                # UDP: 绑定本机端口接收惯导数据
                socket_.bind((local_ip, local_port))
        except socket.error as exc:
            driver.get_logger().error("Caught exception socket.error when setting up socket: %s" % exc)
            if socket_:
                socket_.close()
            if protocol == 'tcp':
                # TCP 连接失败（设备未就绪/网络中断）时自动重试，进程不退出
                time.sleep(timeout)
                continue
            # UDP 绑定失败属于配置错误，退出
            sys.exit(1)

        # recv-loop: When we're connected, keep receiving stuff until that fails
        partial = ""
        while rclpy.ok():
            try:
                if protocol == 'tcp':
                    data = socket_.recv(buffer_size)
                    if not data:
                        # 对端关闭连接，触发重连
                        raise socket.error("connection closed by peer")
                else:
                    data, remote_address = socket_.recvfrom(buffer_size)

                # strip the data
                partial += data.decode("ascii")

                if not partial:
                    continue

                # strip the data
                lines = partial.splitlines()
                if partial.endswith('\n'):
                    data_list = lines
                    partial = ""
                else:
                    data_list = lines[:-1]
                    partial = lines[-1]

                for data in data_list:

                    try:
                        driver.add_sentence(data, frame_id)
                    except ValueError as e:
                        driver.get_logger().warn(
                            "Value error, likely due to missing fields in the NMEA message. "
                            "Error was: %s. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, "
                            "including a bag file with the NMEA sentences that caused it." % e)

            except socket.timeout:
                if protocol == 'tcp':
                    # TCP 超时仅表示设备暂时无数据，保持连接继续等待
                    continue
                driver.get_logger().error("Caught exception socket.error during recvfrom: %s" % "timed out")
                socket_.close()
                # This will break out of the recv-loop so we start another iteration of the connection-loop
                break

            except socket.error as exc:
                driver.get_logger().error("Caught exception socket.error during recvfrom: %s" % exc)
                socket_.close()
                # This will break out of the recv-loop so we start another iteration of the connection-loop
                break

        socket_.close()  # Close socket
