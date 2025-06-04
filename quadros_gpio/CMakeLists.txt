cmake_minimum_required(VERSION 3.5)
project(quadros_gpio)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadros_calibration REQUIRED)

include_directories(
  /usr/local/include  # Path to wiringPi headers, adjust if needed
)

add_executable(quadros_gpio_node src/quadros_gpio.cpp)

ament_target_dependencies(quadros_gpio_node
  rclcpp
  std_msgs
  quadros_calibration
)

target_link_libraries(quadros_gpio_node pigpio)

install(TARGETS
  quadros_gpio_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
