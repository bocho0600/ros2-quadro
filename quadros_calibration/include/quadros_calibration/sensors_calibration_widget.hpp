#pragma once

#include <QWidget>
#include <QTimer>
#include <QCheckBox>
#include <QLabel>
#include "quadros_calibration/qcustomplot.hpp"

class SensorsCalibrationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SensorsCalibrationWidget(QWidget *parent = nullptr);
    void setLiveAngles(double pitch, double roll);

private slots:
    void updatePlot();
    void onLiveDataToggled(bool checked);

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
};
