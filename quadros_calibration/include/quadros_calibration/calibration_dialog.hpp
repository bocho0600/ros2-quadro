#pragma once
#include <QDialog>
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
#include "rclcpp/rclcpp.hpp"
#include "quadros/msg/telemetry.hpp" // Add this include
#include "quadros/msg/motor_speed.hpp" // Add this include
#include "quadros/msg/calibration.hpp" // Add this include

class CalibrationDialog : public QDialog {
    Q_OBJECT
public:
    explicit CalibrationDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);


private slots:
    void switchMode(int index);

    void publishMotorSpeeds();
    void publishCalibration();

private:
    QListWidget *menu_;
    QStackedLayout *stack_;
    MotorCalibrationWidget *motor_widget_; // instance of MotorCalibrationWidget
    SensorsCalibrationWidget *sensor_widget_;   // instance of SensorsCalibrationWidget
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<quadros::msg::MotorSpeed>::SharedPtr publisher_;
    rclcpp::Publisher<quadros::msg::Calibration>::SharedPtr calibration_publisher_;
    rclcpp::Subscription<quadros::msg::Telemetry>::SharedPtr telemetry_sub_; // Add this member
    QTimer *timer_;

    void onTelemetryReceived(const quadros::msg::Telemetry::SharedPtr msg);
};

// #include "calibration_dialog.moc"