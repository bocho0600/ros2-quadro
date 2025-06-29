#include "quadros_controller/controller_window.hpp"

ControllerWindow::ControllerWindow(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QWidget(parent), node_(node) {

    auto *layout = new QVBoxLayout(this);

    throttle_slider_ = new QSlider(Qt::Vertical);
    yaw_slider_ = new QSlider(Qt::Horizontal);
    pitch_slider_ = new QSlider(Qt::Vertical);
    roll_slider_ = new QSlider(Qt::Horizontal);

    throttle_slider_->setRange(0, 100);
    yaw_slider_->setRange(-100, 100);
    pitch_slider_->setRange(-100, 100);
    roll_slider_->setRange(-100, 100);

    layout->addWidget(new QLabel("Throttle"));
    layout->addWidget(throttle_slider_);
    layout->addWidget(new QLabel("Yaw"));
    layout->addWidget(yaw_slider_);
    layout->addWidget(new QLabel("Pitch"));
    layout->addWidget(pitch_slider_);
    layout->addWidget(new QLabel("Roll"));
    layout->addWidget(roll_slider_);

    setLayout(layout);
    setWindowTitle("Quadros UAV Controller");
    resize(300, 400);
}
