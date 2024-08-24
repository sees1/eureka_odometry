#include <eureka_odometry/eureka_odometry.hpp>
#include <numeric>

#define M_PI 3.14159265358979323846

namespace eureka_odometry
{
  
  EurekaOdometry::EurekaOdometry()
  : rclcpp::Node("eureka_odometry"),
    wheel_radius(0.11),
    wheel_base(0.795),
    wheel_track(0.9),
    measure_error(0.15)
  {
    odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>(
    "/odometry", rclcpp::SystemDefaultsQoS());

    odometry_transform_publisher = this->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::SystemDefaultsQoS());
 
    joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
      "/wheel_states",
      rclcpp::SensorDataQoS(),
      std::bind(&EurekaOdometry::joint_state_subscriber_callback, this, std::placeholders::_1));

    odometry.set_wheel_params(wheel_radius, wheel_base, wheel_track);
    odometry.init(this->now());
  }


  void EurekaOdometry::joint_state_subscriber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    size_t joint_count = msg->name.size();
    double steer_angle_deg_accumulated = std::accumulate(msg->position.begin(), msg->position.end(), 0.0);
    double steer_average_angle_deg = steer_angle_deg_accumulated / joint_count;
    double steer_average_angle_rad = steer_average_angle_deg * M_PI / 180.0;
    double wheel_angular_velocity_accumulated = std::accumulate(msg->velocity.begin(), msg->velocity.end(), 0.0);
    double wheel_average_angular_velocity = wheel_angular_velocity_accumulated / joint_count;

    double robot_linear_velocity = wheel_average_angular_velocity * (1 - measure_error) * wheel_radius;
    double robot_angular_velocity = std::tan(steer_average_angle_rad) * robot_linear_velocity / wheel_base;

    odometry.update_open_loop(robot_linear_velocity, robot_angular_velocity, 0.01);

    orientation.setRPY(0.0, 0.0, odometry.get_heading());

    odometry_msg.header.stamp = this->now();
    odometry_msg.pose.pose.position.x = odometry.get_x();
    odometry_msg.pose.pose.position.y = odometry.get_y();
    odometry_msg.pose.pose.orientation = tf2::toMsg(orientation);
    odometry_msg.twist.twist.linear.x = odometry.get_linear();
    odometry_msg.twist.twist.angular.z = odometry.get_angular();

    odometry_publisher->publish(odometry_msg);
  }

} // namespace eureka_odometry