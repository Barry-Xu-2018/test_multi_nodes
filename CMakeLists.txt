cmake_minimum_required(VERSION 3.8)
project(test_multi_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publishers src/publishers.cpp)
ament_target_dependencies(publishers
  rclcpp
  std_msgs
)

add_executable(subscribers src/subscribers.cpp)
ament_target_dependencies(subscribers
  rclcpp
  std_msgs
)

install(TARGETS
  publishers
  subscribers
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
