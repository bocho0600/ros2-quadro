#pragma once

#include <QWidget>
#include <QTimer>
#include <QCheckBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include "quadros_calibration/qcustomplot.hpp"
#include <array>

struct CalibrationValues {
    std::array<float, 3> roll_pid;   // P, I, D
    std::array<float, 3> pitch_pid;  // P, I, D
    std::array<float, 3> yaw_pid;    // P, I, D
    std::array<float, 2> kalman_qr;  // Q, R
};

class SensorsCalibrationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SensorsCalibrationWidget(QWidget *parent = nullptr);
    void setLiveAngles(double pitch, double roll);
    CalibrationValues getCalibration() const;
private slots:
    void updatePlot();

private:
    QCustomPlot *plotPitch_;
    QCustomPlot *plotRoll_;
    QTimer *timer_;
    double phase_;

    QCheckBox *liveDataCheckBox_;
    QPushButton *calibrateButton_;
    // ...existing members...
    double latestPitch_ = 0.0;
    double latestRoll_ = 0.0;
    QLabel *modeLabel_;

    QVector<double> pitchBuffer_;
    QVector<double> rollBuffer_;
    QVector<double> timeBuffer_;
    double time_;

    QDoubleSpinBox *kalmanQSpin_ = nullptr;
    QDoubleSpinBox *kalmanRSpin_ = nullptr;

signals:
    void calibrationRequested();
};
