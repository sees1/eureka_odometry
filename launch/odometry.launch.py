import os

# normal imports
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  static_transform_pub = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_camera_depth_optical',
    arguments=['0.43', '0', '0',
                # the order is yaw, pitch, roll
                '-3.14159265358979323846/2', '0', '-3.14159265358979323846/2',
                'base_link', 'camera_depth_optical_frame'],
    output='screen',
  )

  eureka_odometry_node = Node(
    package='eureka_odometry',
    executable='eureka_odometry',
    output='screen'
  )

  ld = LaunchDescription()
  ld.add_action(static_transform_pub)
  ld.add_action(eureka_odometry_node)
  return ld
