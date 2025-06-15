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
    
    QLabel *modeLabel_;
};
