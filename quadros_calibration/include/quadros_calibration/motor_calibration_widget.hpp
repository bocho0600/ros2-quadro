#pragma once

#include <QWidget>
#include <QSlider>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFrame>
#include <array>
#include "rclcpp/rclcpp.hpp"


class MotorCalibrationWidget : public QWidget {
    Q_OBJECT
public:
    explicit MotorCalibrationWidget(QWidget *parent = nullptr);

    std::array<int, 4> getMotorSpeeds() const; // a function to get array of motor speeds
    bool isArmed() const;

private slots: // private slots for handling events
    void toggleArm(bool checked);

private:
    QSlider* sliders_[4];
    QSlider* all_slider_;
    QCheckBox* arm_switch_;
    bool armed_;
};

//#include "motor_calibration_widget.moc"