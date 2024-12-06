import os

# normal imports
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

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
               '--qw',  ' 1.0',
               '--frame-id',       'base_footprint',
               '--child-frame-id', 'camera_link'
              ],
    output='screen'
  )

  imu_config = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config')

  imu_filter = Node(
      package='imu_filter_madgwick',
      executable='imu_filter_madgwick_node',
      name='imu_filter',
      output='screen',
      parameters=[os.path.join(imu_config, 'imu_filter.yaml')],
      remappings=[
          ('/imu/data_raw', '/camera/camera/imu')
      ]
  )

  imu_transformer = Node(
      package='imu_transformer',
      executable='imu_transformer_node',
      name='imu_transformer_node',
      parameters=[{'target_frame': 'base_footprint'}],
      remappings=[
          ('imu_in', '/imu/data'),
          ('imu_out', 'imu/data/base')
      ],
      output='screen'
  )

  
  eureka_odometry_node = Node(
    package='eureka_odometry',
    executable='eureka_odometry',
    output='screen',
    parameters=[os.path.join(get_package_share_directory("eureka_odometry"), 'config', 'odometry.yaml')],
  )

  ekf_for_odom = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("eureka_odometry"), 'config', 'ekf.yaml')],
  )

  use_sim_time_param = SetParameter(name='use_sim_time', value=False)

  ld = LaunchDescription()
  ld.add_action(static_transform_pub_1)
  ld.add_action(imu_filter)
  ld.add_action(eureka_odometry_node)
  ld.add_action(imu_transformer)
  ld.add_action(ekf_for_odom)
  ld.add_action(use_sim_time_param)
  return ld
