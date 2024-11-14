#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor

from scipy.spatial.transform import Rotation as R
import numpy as np

class VisualInertialOdometryPublisher(Node):
    def __init__(self):
        super().__init__('publish_to_px4_ekf')
        self.topic_out = '/odometry/filtered'
        # Quaternion for 180-degree rotation around the X-axis
        self.rotation_quaternion = R.from_euler('x', 180, degrees=True).as_quat()
        self.odometry_publisher_ = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.odometry_subscription_ = self.create_subscription(
            Odometry,
            self.topic_out,
            self.odometry_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.timer = self.create_timer(10, self.timer_callback)  # Timer callback every 10 seconds
        
    def odometry_callback(self, msg):
        vehicle_odometry_msg = VehicleOdometry()
        vehicle_odometry_msg.timestamp = int((msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)//1e3)  #microseconds

        vehicle_odometry_msg.pose_frame = 2
        vehicle_odometry_msg.velocity_frame = 2
        ## Set the position information
        position = msg.pose.pose.position
        # Rotate the position vector by 180 degrees around the x-axis
        rotated_position = Vector3()
        rotated_position.x = position.x
        rotated_position.y = -position.y
        rotated_position.z = -position.z

        vehicle_odometry_msg.position[0] = rotated_position.x
        vehicle_odometry_msg.position[1] = rotated_position.y
        vehicle_odometry_msg.position[2] = rotated_position.z
        
        ## Set the linear velocity information
        velocity = msg.twist.twist.linear
        
        # Rotate the velocity vector by 180 degrees around the x-axis
        rotated_velocity = Vector3()
        rotated_velocity.x = velocity.x
        rotated_velocity.y = -velocity.y
        rotated_velocity.z = -velocity.z

        vehicle_odometry_msg.velocity[0] = rotated_velocity.x
        vehicle_odometry_msg.velocity[1] = rotated_velocity.y
        vehicle_odometry_msg.velocity[2] = rotated_velocity.z

        ## Set the angular velocity information
        angular_velocity = msg.twist.twist.angular
        
        # Rotate the velocity vector by 180 degrees around the x-axis
        rotated_angular_velocity = Vector3()
        rotated_angular_velocity.x = angular_velocity.x
        rotated_angular_velocity.y = -angular_velocity.y
        rotated_angular_velocity.z = -angular_velocity.z

        vehicle_odometry_msg.angular_velocity[0] = rotated_angular_velocity.x
        vehicle_odometry_msg.angular_velocity[1] = rotated_angular_velocity.y
        vehicle_odometry_msg.angular_velocity[2] = rotated_angular_velocity.z

        # Set the orientation information
        orientation = msg.pose.pose.orientation
        original_quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        rotated_quaternion = R.from_quat(self.rotation_quaternion) * R.from_quat(original_quaternion)
        rotated_quat = rotated_quaternion.as_quat()
        vehicle_odometry_msg.q[0] = rotated_quaternion[0]
        vehicle_odometry_msg.q[1] = rotated_quaternion[1]
        vehicle_odometry_msg.q[2] = rotated_quaternion[2]
        vehicle_odometry_msg.q[3] = rotated_quaternion[3]
        
        # set variences
        vehicle_odometry_msg.position_variance = [msg.pose.covariance[0],msg.pose.covariance[7],msg.pose.covariance[14]]
        vehicle_odometry_msg.orientation_variance = [msg.pose.covariance[21],msg.pose.covariance[28],msg.pose.covariance[35]]
        vehicle_odometry_msg.velocity_variance = [msg.twist.covariance[0],msg.twist.covariance[7],msg.twist.covariance[14]]

        # Publish the vehicle odometry message
        self.odometry_publisher_.publish(vehicle_odometry_msg)

    def timer_callback(self):
        self.get_logger().info('Publishing filtered odometry to px4')


def main(args=None):
    rclpy.init()
    publisher = VisualInertialOdometryPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(publisher)
    try:
        executor.spin()
    except KeyboardInterrupt:
        publisher.get_logger().info("Shutting down.")
        publisher.on_shutdown()
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
