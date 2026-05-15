# Copyright 2024 Pixmoving, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from ament_index_python import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
def generate_launch_description():
    launch_arguments = []
    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    # 创建组件节点----------------------------------------------------------------------------------------
    nmea_imu_component_node = ComposableNode(
        package="nmea_udp_navsat_driver_cpp",
        plugin="NmeaUdpImuDriverNode",
        name="nmea_udp_navsat_driver",
        namespace="",
        parameters=[{
            'bind_address': '192.168.1.103',
            'port': 9906,
            'frame_id': 'gnss',
            'imu_topic': '/sensing/gnss/imu',
            'exit_on_bind_fail': True,
            'cov_sigma0': 1.5,
        }],
    )
    
    
    # 加载组件节点-----------------------------------------------------------------------------------------
    nmea_imu_loader = LoadComposableNodes(
        composable_node_descriptions=[
            nmea_imu_component_node,
            ],
        target_container="/pointcloud_container",
    )
    return launch.LaunchDescription([
        *launch_arguments,
        nmea_imu_loader,
        ]
    )


