#include <eureka_odometry/eureka_odometry.hpp>
#include <numeric>

#define M_PI 3.14159265358979323846

namespace eureka_odometry
{
  EurekaOdometry::EurekaOdometry()
  : rclcpp::Node("eureka_odometry"),
    wheel_radius(0.11),
    wheel_base(0.795),
    wheel_track(0.778),
    measure_error(0.15),
    current_roll(0.0),
    current_pitch(0.0),
    current_yaw(0.0)
  {
    odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>(
    "/odometry", rclcpp::SystemDefaultsQoS());

    odometry_tf_publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
 
    joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
      "/wheel_states",
      rclcpp::SensorDataQoS(),
      std::bind(&EurekaOdometry::joint_state_subscriber_callback, this, std::placeholders::_1));

    imu_subscriber = create_subscription<sensor_msgs::msg::Imu>(
      "/camera/camera/imu_with_quat",
      rclcpp::SensorDataQoS(),
      std::bind(&EurekaOdometry::imu_subscriber_callback, this, std::placeholders::_1));

    odometry.set_wheel_params(wheel_radius, wheel_base, wheel_track);
    odometry.init(this->now());
  }

  void EurekaOdometry::imu_subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::fromMsg(msg->orientation, orientation);
    orientation_matrix.setRotation(orientation);
    orientation_matrix.getRPY(current_roll, current_pitch, current_yaw);
  }


  void EurekaOdometry::joint_state_subscriber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    size_t joint_count = msg->name.size();

    double steer_angle_deg_accumulated = msg->position[0] + msg->position[3] - msg->position[2] - msg->position[5]; 
    double steer_average_angle_deg = steer_angle_deg_accumulated / 4;
    double steer_average_angle_rad = steer_average_angle_deg * M_PI / 180.0;

    double wheel_angular_velocity_accumulated =  msg->velocity[0] + msg->velocity[1] + msg->velocity[2] - msg->velocity[3] - msg->velocity[4] - msg->velocity[5];
    double wheel_average_angular_velocity = wheel_angular_velocity_accumulated / joint_count;

    double robot_linear_velocity  = wheel_average_angular_velocity * (1 - measure_error) * wheel_radius;
    double robot_angular_velocity = std::tan(steer_average_angle_rad) * robot_linear_velocity / wheel_base;

    if (msg->position[0] < -30 && msg->position[3] > 30 &&
        msg->position[2] > 30  && msg->position[5] < -30)
    {
      steer_angle_deg_accumulated = msg->position[3] - msg->position[0] + msg->position[2] - msg->position[5];
      steer_average_angle_deg = steer_angle_deg_accumulated / 4.0;
      steer_average_angle_rad = steer_average_angle_deg * M_PI / 180.0; 
      wheel_angular_velocity_accumulated = msg->velocity[0] + msg->velocity[2] + msg->velocity[3] + msg->velocity[5];
      wheel_average_angular_velocity = -wheel_angular_velocity_accumulated / 4.0;
      robot_linear_velocity = wheel_average_angular_velocity * (1 - measure_error) * wheel_radius;
      robot_angular_velocity = 2 * std::tan(steer_average_angle_rad) * robot_linear_velocity / wheel_base;
      robot_linear_velocity = 0.0;
    }

    //TODO: add dt corresponding to the topic rate
    odometry.update_open_loop(robot_linear_velocity, robot_angular_velocity, current_pitch, 0.05);

    orientation.setRPY(current_roll, current_pitch, odometry.get_heading());

    odometry_msg.header.stamp = this->now();
    odometry_msg.pose.pose.position.x = odometry.get_x();
    odometry_msg.pose.pose.position.y = odometry.get_y();
    odometry_msg.pose.pose.orientation = tf2::toMsg(orientation);
    odometry_msg.twist.twist.linear.x = odometry.get_linear();
    odometry_msg.twist.twist.angular.z = odometry.get_angular();
    odometry_publisher->publish(odometry_msg);

    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id="odom";
    transform_msg.child_frame_id="base_link";
    transform_msg.transform.translation.x=odometry.get_x();
    transform_msg.transform.translation.y=odometry.get_y();
    transform_msg.transform.translation.z=0.0;
    transform_msg.transform.rotation=tf2::toMsg(orientation);
    odometry_tf_publisher->sendTransform(transform_msg);
  }
} // namespace eureka_odometry