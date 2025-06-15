#pragma once
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSlider>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QStackedLayout>
#include <QTimer>
#include <QCheckBox>
#include <array>
#include <memory>
#include "rclcpp/rclcpp.hpp"
//#include "sensor_calibration_widget.moc"
class SensorsCalibrationWidget : public QWidget {
    Q_OBJECT
public:
    explicit SensorsCalibrationWidget(QWidget *parent = nullptr);
};

#include "sensors_calibration_widget.moc"