cmake_minimum_required(VERSION 3.8)
project(starling_telecom_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include "${CMAKE_CURRENT_BINARY_DIR}")

add_executable(transmitter src/transmitter.cpp)
ament_target_dependencies(transmitter
  rclcpp
  rclcpp_components
  std_msgs)

add_executable(receiver src/receiver.cpp)
ament_target_dependencies(receiver
  rclcpp
  rclcpp_components
  std_msgs)

add_executable(publisher src/basic_publisher.cpp)
ament_target_dependencies(publisher
  rclcpp
  rclcpp_components
  std_msgs)

add_executable(subscriber src/basic_subscriber.cpp)
ament_target_dependencies(subscriber
  rclcpp
  rclcpp_components
  std_msgs)

install(TARGETS
  transmitter
  receiver
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
