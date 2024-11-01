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
               '--y', '0.03',
               '--z', '0.125',
               '--qx',  '0.00',
               '--qy',  '0.00',
               '--qz',  '0.00',
               '--qw',  ' 1.00',
               '--frame-id',       'base_link',
               '--child-frame-id', 'camera_link'
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
  ld.add_action(eureka_odometry_node)
  return ld
