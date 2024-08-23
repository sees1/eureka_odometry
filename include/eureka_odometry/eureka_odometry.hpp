#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcppmath/rolling_mean_accumulator.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <eureka_odometry/steering_odometry.hpp>

namespace eureka_odometry
{

  class EurekaOdometry : public rclcpp::Node
  {
  public:
    EurekaOdometry();

  private:
    void joint_state_subscriber_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  private:
    // Odometry publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odometry_transform_publisher;

    // JointState subscriber
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

    double wheel_radius;
    double wheel_base;
    double wheel_track;
    double measure_error;

    nav_msgs::msg::Odometry odometry_msg;
    tf2::Quaternion orientation;

    steering_odometry::SteeringOdometry odometry;
  };

}  // namespace eureka_odometry