cmake_minimum_required(VERSION 3.16)
project(eureka_odometry LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  sensor_msgs
  geometry_msgs
  rclcpp
  tf2_msgs
  tf2_geometry_msgs
  tf2
  realtime_tools
  rcpputils
  nav_msgs
  pluginlib
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include/)

add_executable(${PROJECT_NAME}
  src/steering_odometry.cpp
  src/eureka_odometry.cpp
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  geometry_msgs
  rclcpp
  tf2_msgs
  tf2_geometry_msgs
  tf2
  rcpputils
  realtime_tools
  nav_msgs
  pluginlib
)

# INSTALL
install(TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


ament_package()
