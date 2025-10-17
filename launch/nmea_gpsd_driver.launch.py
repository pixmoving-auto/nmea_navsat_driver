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

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros import actions


def generate_launch_description():
    """Generate a launch description for the GPSD NMEA driver."""
    # 声明launch参数 - 与现有架构保持一致
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='2.0',
        description='Timeout for GPSD connection'
    )
    
    auto_detect_arg = DeclareLaunchArgument(
        'auto_detect',
        default_value='true',
        description='Enable automatic GPSD detection'
    )
    
    # 使用现有的配置文件
    config_file = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config", "nmea_gpsd_driver.yaml")
    
    # 创建GPSD驱动节点
    driver_node = actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_gpsd_driver',
        name='nmea_gpsd_driver',
        output='screen',
        parameters=[config_file, {
            'timeout': LaunchConfiguration('timeout'),
            'auto_detect': LaunchConfiguration('auto_detect'),
        }],
        remappings=[
            ('fix', '/sensing/gnss/adcu/fix'),
            ('chc/imu', '/sensing/gnss/adcu/imu'),
            ('chc/heading', 'adcu/heading'),
            ('vel', 'adcu/vel'),
            ('time_reference', 'adcu/time_reference'),
        ],
    )

    return LaunchDescription([
        timeout_arg,
        auto_detect_arg,
        driver_node,
    ])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)