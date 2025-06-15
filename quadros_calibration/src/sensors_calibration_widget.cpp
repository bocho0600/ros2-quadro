#include "quadros_calibration/sensors_calibration_widget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>
#include <QTimer>
#include <cmath>  // For sin(), cos()

SensorsCalibrationWidget::SensorsCalibrationWidget(QWidget *parent)
    : QWidget(parent), phase_(0.0)
{
    // === Main Horizontal Layout ===
    QHBoxLayout *mainLayout = new QHBoxLayout();

    // === Left: Pitch and Roll Plots ===
    QVBoxLayout *plotLayout = new QVBoxLayout();

    // Pitch Plot
    plotPitch_ = new QCustomPlot(this);
    plotPitch_->addGraph();
    plotPitch_->graph(0)->setPen(QPen(Qt::blue));
    plotPitch_->xAxis->setLabel("Time");
    plotPitch_->yAxis->setLabel("Pitch");
    plotPitch_->xAxis->setRange(0, 10);
    plotPitch_->yAxis->setRange(-180, 180);

    // Roll Plot
    plotRoll_ = new QCustomPlot(this);
    plotRoll_->addGraph();
    plotRoll_->graph(0)->setPen(QPen(Qt::red));
    plotRoll_->xAxis->setLabel("Time");
    plotRoll_->yAxis->setLabel("Roll");
    plotRoll_->xAxis->setRange(0, 10);
    plotRoll_->yAxis->setRange(-180, 180);

    plotLayout->addWidget(plotPitch_);
    plotLayout->addWidget(plotRoll_);

    // === Middle: Vertical Line Separator ===
    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::VLine);
    line->setFrameShadow(QFrame::Sunken);
    line->setLineWidth(2);

    // === Right: Placeholder Layout (Future Use) ===
    QVBoxLayout *rightLayout = new QVBoxLayout();
    rightLayout->addSpacing(40); // Optional content space
    rightLayout->addStretch();   // Push contents to top

    // === Assemble Layout ===
    mainLayout->addLayout(plotLayout);
    mainLayout->addWidget(line);
    mainLayout->addLayout(rightLayout);
    setLayout(mainLayout);

    // === Timer Setup ===
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &SensorsCalibrationWidget::updatePlot);
    timer_->start(10);  // 100 FPS (10ms interval)
}

void SensorsCalibrationWidget::updatePlot()
{
    const int nPoints = 100;
    const double dt = 10.0 / nPoints;
    const double frequency = 1.0;

    QVector<double> x(nPoints), yPitch(nPoints), yRoll(nPoints);
    for (int i = 0; i < nPoints; ++i) {
        x[i] = i * dt;
        yPitch[i] = 45 * sin(2 * M_PI * frequency * x[i] + phase_);  // Simulated pitch
        yRoll[i] = 30 * cos(2 * M_PI * frequency * x[i] + phase_);   // Simulated roll
    }

    plotPitch_->graph(0)->setData(x, yPitch);
    plotPitch_->replot();

    plotRoll_->graph(0)->setData(x, yRoll);
    plotRoll_->replot();

    phase_ += 0.1;
    if (phase_ > 2 * M_PI)
        phase_ -= 2 * M_PI;
}
