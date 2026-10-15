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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    package_name = 'robot_bringup'
    pkg_share = get_package_share_directory(package_name)

    # Paths
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'my_robot.rviz')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # With gui, rviz and teleop all false, nothing needs a display
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start the Ignition GUI client')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz')
    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='true',
        description='Start the keyboard teleop in its own xterm')

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # 2. Launch Ignition Gazebo Fortress
    # -r: run immediately, -s: server only (no GUI process that can crash),
    # --headless-rendering: allows sensor rendering without a GUI window
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': f'-r -s --headless-rendering {world_path}',
        }.items(),
    )

    # Launch Ignition GUI separately (not required: won't kill the sim if it crashes)
    ignition_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # 3. Spawn robot into Ignition from robot_description topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-z', '0.2',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 4. ros_gz_bridge: bridge Ignition topics to ROS2
    # The last entry is the simulator's true pose of every moving model,
    # remapped so it does not land on /tf.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/warehouse/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/world/warehouse/dynamic_pose/info', '/ground_truth/poses'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 5. Controller spawners, one after the other: the spawn starts
    # joint_broad when it exits, and joint_broad's spawner starts diff_cont.
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        parameters=[controller_params_file, {'use_sim_time': True}],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        parameters=[controller_params_file, {'use_sim_time': True}],
    )

    joint_broad_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )

    diff_drive_after_joint_broad = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # 6. EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
    )

    # 7. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # 8. Teleop
    my_teleop = Node(
        package='robot_bringup',
        executable='teleop',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -geometry 80x20 -e',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        teleop_arg,
        robot_state_publisher,
        ignition_gazebo,
        ignition_gui,
        bridge,
        spawn_entity,
        joint_broad_after_spawn,
        diff_drive_after_joint_broad,
        rviz_node,
        ekf_node,
        my_teleop,
    ])
