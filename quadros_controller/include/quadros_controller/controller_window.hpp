#pragma once

#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <rclcpp/rclcpp.hpp>

class ControllerWindow : public QWidget {
    Q_OBJECT
public:
    explicit ControllerWindow(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);

private:
    QSlider *throttle_slider_;
    QSlider *yaw_slider_;
    QSlider *pitch_slider_;
    QSlider *roll_slider_;
    std::shared_ptr<rclcpp::Node> node_;
};
