import os

# normal imports
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  #TODO: add M_PI constant for evaluate qx and etc
  static_transform_pub_1 = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_camera',
    arguments=['--x', '0.43',
               '--y', '0.0',
               '--z', '0.0',
               '--qx',  '-1.570796327',
               '--qy',  '0.0',
               '--qz',  '-1.570796327',
               '--qw',  '1.0',
               '--frame-id',       'base_link',
               '--child-frame-id', 'camera_link'
              ],
    output='screen'
  )

  static_transform_pub_2 = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_camera_depth_optical',
    arguments=['--x', '0.0',
              '--y',  '0.0',
              '--z',  '0.0',
               '--qx',  '0.0',
               '--qy',  '0.0',
               '--qz',  '0.0',
               '--qw',  '1.0',
              '--frame-id',       'camera_link',
              '--child-frame-id', 'camera_depth_optical_frame'
              ],
    output='screen'
  )

  eureka_odometry_node = Node(
    package='eureka_odometry',
    executable='eureka_odometry',
    output='screen'
  )

  ld = LaunchDescription()
  ld.add_action(static_transform_pub_1)
  ld.add_action(static_transform_pub_2)
  ld.add_action(eureka_odometry_node)
  return ld
