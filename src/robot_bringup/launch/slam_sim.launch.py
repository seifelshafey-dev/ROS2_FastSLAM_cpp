# Copyright 2026 Seif ElShafey
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    slam_config_path = os.path.join(
        get_package_share_directory('slam_cpp'), 'config', 'slam_params.yaml')

    # Same arguments as sim.launch.py, passed straight through
    forwarded = ['gui', 'rviz', 'teleop']
    forwarded_args = [
        DeclareLaunchArgument(
            name, default_value='true',
            description=f'Passed to sim.launch.py as {name}')
        for name in forwarded
    ]

    # 1. Simulator, robot, controllers, EKF
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            name: LaunchConfiguration(name) for name in forwarded
        }.items(),
    )

    # 2. SLAM
    slam_node = Node(
        package='slam_cpp',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[slam_config_path, {'use_sim_time': True}],
    )

    return LaunchDescription(forwarded_args + [sim, slam_node])
