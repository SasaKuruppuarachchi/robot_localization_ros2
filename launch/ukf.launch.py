# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ukf_agipix.yaml')],
    )
    odom_to_path = Node(
        package="robot_localization",
        executable="odom_to_path_node.py",
        name="odom_to_path_ekf",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[],
    )
    publish_to_px4_node = Node(
        package="robot_localization",
        executable="publish_to_px4_node.py",
        name="publish_to_px4_ekf",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[],
    )
    return LaunchDescription([publish_to_px4_node])

