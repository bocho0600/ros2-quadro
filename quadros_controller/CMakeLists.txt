cmake_minimum_required(VERSION 3.8)
project(quadros_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include
)

add_executable(controller
  src/main.cpp
  src/controller_window.cpp
  include/quadros_controller/controller_window.hpp
)

target_link_libraries(controller Qt5::Widgets)
ament_target_dependencies(controller rclcpp)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
