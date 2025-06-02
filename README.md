# ros2-quadro

ros2 run quadros_calibration calibration_gui
ros2 topic echo /MotorSpeed

sudo bash -c "source /opt/ros/humble/setup.bash && source /home/quadro/ros2_ws/install/setup.bash && ros2 run quadros quadros_gpio_node"