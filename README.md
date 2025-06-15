# ros2-quadro

ros2 run quadros_calibration calibration_gui
ros2 topic echo /MotorSpeed

sudo bash -c "source /opt/ros/humble/setup.bash && source /home/quadro/ros2_ws/install/setup.bash && ros2 run quadros_gpio quadros_gpio_node"

sudo bash -c "source /opt/ros/humble/setup.bash && source /home/quadro/ros2_ws/install/setup.bash && ros2 run quadros uart_reader"
sudo bash -c "source /opt/ros/humble/setup.bash && source /home/quadro/ros2_ws/install/setup.bash && ros2 topic echo /quadros/state/telemetry"


source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && ros2 topic echo /quadros/set/motors


source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash