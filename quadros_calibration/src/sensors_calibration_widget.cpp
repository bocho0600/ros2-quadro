#include "quadros_calibration/sensors_calibration_widget.hpp"
#include <QTimer>
#include <cmath>  // for sin()

SensorsCalibrationWidget::SensorsCalibrationWidget(QWidget *parent)
    : QWidget(parent), phase_(0.0)
{
    QVBoxLayout *layout = new QVBoxLayout();

    // Plot widget
    plot_ = new QCustomPlot(this);
    plot_->addGraph();
    plot_->graph(0)->setPen(QPen(Qt::blue));
    plot_->xAxis->setLabel("Time");
    plot_->yAxis->setLabel("Pitch");
    plot_->xAxis->setRange(0, 10);    // Fixed x-axis range (adjust as needed)
    plot_->yAxis->setRange(-180, 180);  // Y-axis range for sine wave

    QLabel *label = new QLabel("Sensor Calibration");
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("font-size: 16pt; font-weight: bold;");

    // layout->addWidget(label);
    layout->addWidget(plot_);
    setLayout(layout);

    // Timer to update plot
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &SensorsCalibrationWidget::updatePlot);
    timer_->start(10);  // Update every 50 ms (20 FPS)
}

void SensorsCalibrationWidget::updatePlot()
{
    const int nPoints = 100;          // Number of data points
    const double dt = 10.0 / nPoints; // Total width = 10, so spacing is 0.1
    double frequency = 1.0;           // 1 Hz sine wave

    QVector<double> x(nPoints), y(nPoints);

    for (int i = 0; i < nPoints; ++i) {
        x[i] = i * dt;
        y[i] = 10 * sin(2 * M_PI * frequency * x[i] + phase_);
    }

    plot_->graph(0)->setData(x, y);
    plot_->replot();

    // Increment phase to make the wave scroll
    phase_ += 0.1;
    if (phase_ > 2 * M_PI)
        phase_ -= 2 * M_PI;
}