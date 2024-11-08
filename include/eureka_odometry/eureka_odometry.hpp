#pragma once

#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_ros/transform_broadcaster.h>

#include <eureka_odometry/steering_odometry.hpp>

#include <rcppmath/rolling_mean_accumulator.hpp>

namespace eureka_odometry
{
  using namespace std::literals::chrono_literals;

  class EurekaOdometry : public rclcpp::Node
  {
  public:
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

  public:
    EurekaOdometry();

  private:
    void joint_state_subscriber_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imu_subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  private:
    // Odometry publisher's
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_tf_publisher;

    // Subscriber's
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;

    // Subscriber's parameter's
    TimePoint last_time_point;

    double wheel_radius;
    double wheel_base;
    double wheel_track;
    double measure_error;

    nav_msgs::msg::Odometry odometry_msg;
    geometry_msgs::msg::TransformStamped transform_msg;

    tf2::Quaternion orientation;
    tf2::Matrix3x3 orientation_matrix;

    double current_roll;
    double current_pitch;
    double current_yaw;

    steering_odometry::SteeringOdometry odometry;

    rcppmath::RollingMeanAccumulator<double> linear_acc_;
    rcppmath::RollingMeanAccumulator<double> angular_acc_;
  };

}  // namespace eureka_odometry