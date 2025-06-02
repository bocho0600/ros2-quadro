cmake_minimum_required(VERSION 3.8)
project(quadros_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorSpeed.msg"
  DEPENDENCIES std_msgs
)
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
# Export runtime dependency for generated messages
ament_export_dependencies(rosidl_default_runtime)

# Add executable after message generation and export dependencies
add_executable(calibration_gui src/calibration_gui.cpp)
ament_target_dependencies(calibration_gui rclcpp std_msgs Qt5Widgets )
rosidl_target_interfaces(calibration_gui ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(calibration_gui ${rosidl_typesupport_target})

install(TARGETS
  calibration_gui
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
