#include <eureka_odometry/eureka_odometry.hpp>
#include <eureka_odometry/steering_odometry.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);


    std::shared_ptr<eureka_odometry::EurekaOdometry> node = std::make_shared<eureka_odometry::EurekaOdometry>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
}