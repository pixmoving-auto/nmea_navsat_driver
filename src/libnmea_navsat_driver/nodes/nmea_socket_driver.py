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
    except KeyError as e:
        driver.get_logger().err("Parameter %s not found" % e)
        sys.exit(1)

    frame_id = driver.get_frame_id()

    driver.get_logger().info(
        " Using parameters ip {} port {} buffer_size {} timeout_sec {}"
        .format(local_ip, local_port, buffer_size, timeout))

    # Connection-loop: connect and keep receiving. If receiving fails, reconnect
    while rclpy.ok():
        try:
            # Create a socket
            socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # Bind the socket to the port
            socket_.bind((local_ip, local_port))

            # Set timeout
            socket_.settimeout(timeout)
        except socket.error as exc:
            driver.get_logger().error("Caught exception socket.error when setting up socket: %s" % exc)
            socket_.settimeout(timeout)
            # continue
            sys.exit(1)

        # recv-loop: When we're connected, keep receiving stuff until that fails
        driver.stream_decoder.reset()
        while rclpy.ok():
            try:
                data, remote_address = socket_.recvfrom(buffer_size)

                driver.add_bytes(data, frame_id)

            except socket.error as exc:
                driver.get_logger().error("Caught exception socket.error during recvfrom: %s" % exc)
                socket_.close()
                # This will break out of the recv-loop so we start another iteration of the connection-loop
                break

        socket_.close()  # Close socket
