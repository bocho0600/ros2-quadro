#include <QApplication>
#include <QPushButton>
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "quadros_calibration/msg/MotorSpeed.hpp"

int main(int argc, char **argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create ROS2 node (shared ptr for use in lambda)
  auto node = std::make_shared<rclcpp::Node>("calibration_gui_node");

  // Create publisher
  auto publisher = node->create_publisher<quadros_calibration::msg::MotorSpeed>("MotorSpeed", 10);

  // Qt app init
  QApplication app(argc, argv);

  // Basic widget setup
  QWidget window;
  window.setWindowTitle("Quadros Calibration");

  auto layout = new QVBoxLayout(&window);

  auto label = new QLabel("Press button to publish motor speeds", &window);
  layout->addWidget(label);

  auto button = new QPushButton("Publish Motor Speeds", &window);
  layout->addWidget(button);

  // Connect button click to publishing a MotorSpeed message
  QObject::connect(button, &QPushButton::clicked, [&]() {
    quadros_calibration::msg::MotorSpeed msg;
    msg.motor_speed_1 = 100.0f;
    msg.motor_speed_2 = 110.0f;
    msg.motor_speed_3 = 120.0f;
    msg.motor_speed_4 = 130.0f;

    publisher->publish(msg);

    label->setText(QString("Published speeds: %1, %2, %3, %4")
                   .arg(msg.motor_speed_1)
                   .arg(msg.motor_speed_2)
                   .arg(msg.motor_speed_3)
                   .arg(msg.motor_speed_4));
  });

  window.show();

  // Spin ROS in background thread
  std::thread ros_spin_thread([&]() {
    rclcpp::spin(node);
  });

  // Run Qt event loop
  int ret = app.exec();

  // Shutdown ROS and join thread
  rclcpp::shutdown();
  ros_spin_thread.join();

  return ret;
}
