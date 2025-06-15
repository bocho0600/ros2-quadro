#pragma once

#include <QWidget>
#include <QSlider>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFrame>
#include <array>
#include "quadros_calibration/sensors_calibration_widget.hpp"
#include "quadros_calibration/motor_calibration_widget.hpp"
#include "quadros_calibration/msg/motor_speed.hpp"
#include "rclcpp/rclcpp.hpp"


class CalibrationDialog : public QDialog {
    Q_OBJECT
public:
    explicit CalibrationDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);


private slots:
    void switchMode(int index);

    void publishMotorSpeeds();

private:
    QListWidget *menu_;
    QStackedLayout *stack_;
    MotorCalibrationWidget *motor_widget_;
    SensorsCalibrationWidget *sensor_widget_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<quadros_calibration::msg::MotorSpeed>::SharedPtr publisher_;
    QTimer *timer_;
};