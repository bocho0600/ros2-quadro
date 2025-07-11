cmake_minimum_required(VERSION 3.8)
project(quadros_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui PrintSupport)
find_package(rosidl_default_generators REQUIRED)
find_package(quadros REQUIRED)

set(CMAKE_AUTOMOC ON)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
# Export runtime dependency for generated messages
ament_export_dependencies(rosidl_default_runtime)

# Add executable after message generation and export dependencies
add_executable(calibration_gui 
      src/main.cpp
      src/sensors_calibration_widget.cpp
      src/motor_calibration_widget.cpp
      src/calibration_dialog.cpp
      src/qcustomplot.cpp      
      include/quadros_calibration/sensors_calibration_widget.hpp  # <- ADD THIS
      include/quadros_calibration/motor_calibration_widget.hpp
      include/quadros_calibration/calibration_dialog.hpp
      include/quadros_calibration/qcustomplot.hpp
      )
ament_target_dependencies(calibration_gui rclcpp std_msgs Qt5Widgets quadros)
# rosidl_target_interfaces is not needed for this executable and causes an error
# rosidl_target_interfaces(calibration_gui ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(calibration_gui ${rosidl_typesupport_target})
target_link_libraries(calibration_gui
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::PrintSupport
)
target_include_directories(calibration_gui PRIVATE # Include directories for the target
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(calibration_gui PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON
)

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
