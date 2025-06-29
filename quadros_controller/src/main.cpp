#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "quadros_controller/controller_window.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = rclcpp::Node::make_shared("quadros_controller");
    ControllerWindow window(node);
    window.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
