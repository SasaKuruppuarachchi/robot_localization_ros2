#    Odom transformer - ROS 2 Node for performing odometry transformations.
#    Copyright (C) 2023  Karelics Oy
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program. If not, see <https://www.gnu.org/licenses/>.

"""Example launch file for tansform_pub"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch tansform_pub. -> translation + orientation + [parent, child]"""

    # Map
    transform_drone0map_to_camerainit = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_pub',
        output='screen',
        arguments = [ '0.0795', '0', '0.0323', '0', '0', '0', '0', 'drone0/map', 'camera_init'] 
    )
    # Lidar IMU
    transform_body_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.011', '0.02329', '-0.04412', '0', '0.3826834', '0', '0.9238795', 'body', 'livox_frame']
    )
    # Camera front
    transform_px4_to_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.117', '0', '-0.025', '0', '0', '0', '0', 'px4_frame', 'camera_frame']
    )
    # px4
    transform_body_to_px4_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['-0.0795', '0', '-0.0323', '0', '0', '0', '0', 'body', 'px4_frame']
    )

    return LaunchDescription([transform_drone0map_to_camerainit, \
        transform_px4_to_camera_frame, \
        transform_body_to_livox_frame,\
        transform_body_to_px4_frame]\
        )
