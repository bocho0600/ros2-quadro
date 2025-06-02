cmake_minimum_required(VERSION 3.8)
project(calobration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For message generation

# === Message generation ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMsg.msg"
  "msg/MotorSpeed.msg"
)

# === Executables ===
add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs) # Ensure dependencies are linked
rosidl_target_interfaces(publisher_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node  rclcpp std_msgs)
rosidl_target_interfaces(subscriber_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# === Install targets ===
install(TARGETS
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Export dependencies ===
ament_export_dependencies(rosidl_default_runtime) # For message typesupport

# === Linting ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
