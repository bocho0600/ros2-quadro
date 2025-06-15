#pragma once
#include <QWidget>
#include <QTimer>
#include "quadros_calibration/qcustomplot.hpp"

class SensorsCalibrationWidget : public QWidget {
    Q_OBJECT
public:
    explicit SensorsCalibrationWidget(QWidget *parent = nullptr);

private slots:
    void updatePlot();  // Slot for timer updates

private:
    QCustomPlot *plot_;
    QTimer *timer_;    // Timer for dynamic updates
    double phase_ = 0; // Tracks sine wave phase
};
