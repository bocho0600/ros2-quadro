#include "quadros_calibration/sensors_calibration_widget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>
#include <QTimer>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <cmath>  // For sin(), cos()

SensorsCalibrationWidget::SensorsCalibrationWidget(QWidget *parent)
    : QWidget(parent), phase_(0.0), time_(0.0)
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

    // === Right: Control Panel ===
    QVBoxLayout *rightLayout = new QVBoxLayout();

    // Mode Label
    modeLabel_ = new QLabel("Mode: Simulation", this);
    modeLabel_->setAlignment(Qt::AlignCenter);
    QFont labelFont = modeLabel_->font();
    labelFont.setPointSize(12);
    labelFont.setBold(true);
    modeLabel_->setFont(labelFont);

    // Live Sensor Data Switch
    liveDataCheckBox_ = new QCheckBox("Live Sensor Data", this);
    liveDataCheckBox_->setToolTip("Toggle between simulated and live IMU data");
    connect(liveDataCheckBox_, &QCheckBox::toggled, this, &SensorsCalibrationWidget::onLiveDataToggled);

    // Calibrate Button
    QPushButton *calibrateButton = new QPushButton("Calibrate", this);
    calibrateButton->setFixedSize(120, 40);

    // Center the button horizontally
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    buttonLayout->addWidget(calibrateButton);
    buttonLayout->addStretch();

    // Add widgets to the right layout
    rightLayout->addWidget(modeLabel_);
    rightLayout->addWidget(liveDataCheckBox_);
    rightLayout->addSpacing(40);
    rightLayout->addLayout(buttonLayout);  // Centered button
    rightLayout->addStretch();  // Push remaining content to the top

    // === Assemble Layout ===
    mainLayout->addLayout(plotLayout);
    mainLayout->addWidget(line);
    QWidget *rightWidget = new QWidget();
    rightWidget->setLayout(rightLayout);
    rightWidget->setMaximumWidth(200);  // Adjust width as needed (e.g., 200 pixels)
    mainLayout->addWidget(rightWidget);
    setLayout(mainLayout);

    // === Timer Setup ===
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &SensorsCalibrationWidget::updatePlot);
    timer_->start(10);  // 100 FPS (10ms interval)

    const int nPoints = 100;
    const double dt = 10.0 / (nPoints - 1); // 0.10101... so last point is exactly 10.0
    pitchBuffer_ = QVector<double>(nPoints, 0.0);
    rollBuffer_ = QVector<double>(nPoints, 0.0);
    timeBuffer_.resize(nPoints);
    for (int i = 0; i < nPoints; ++i) timeBuffer_[i] = i * dt;

    plotPitch_->xAxis->setRange(0, 10);
    plotRoll_->xAxis->setRange(0, 10);
}

void SensorsCalibrationWidget::updatePlot()
{
    const int nPoints = 100;

    if (liveDataCheckBox_->isChecked()) {
        // Shift buffer left
        for (int i = 0; i < nPoints - 1; ++i) {
            pitchBuffer_[i] = pitchBuffer_[i + 1];
            rollBuffer_[i] = rollBuffer_[i + 1];
        }
        // Add new value at the end
        pitchBuffer_[nPoints - 1] = latestPitch_;
        rollBuffer_[nPoints - 1] = latestRoll_;

        // X axis is fixed: timeBuffer_ = [0, ..., 10]
        plotPitch_->graph(0)->setData(timeBuffer_, pitchBuffer_);
        plotRoll_->graph(0)->setData(timeBuffer_, rollBuffer_);
    } else {
        QVector<double> x(nPoints), yPitch(nPoints), yRoll(nPoints);
        const double dt = 10.0 / (nPoints - 1);
        for (int i = 0; i < nPoints; ++i) {
            x[i] = i * dt;
            yPitch[i] = 45 * sin(2 * M_PI * 1.0 * x[i] + phase_);
            yRoll[i] = 30 * cos(2 * M_PI * 1.0 * x[i] + phase_);
        }
        plotPitch_->graph(0)->setData(x, yPitch);
        plotRoll_->graph(0)->setData(x, yRoll);
        phase_ += 0.1;
        if (phase_ > 2 * M_PI)
            phase_ -= 2 * M_PI;
    }

    plotPitch_->replot();
    plotRoll_->replot();
}

void SensorsCalibrationWidget::onLiveDataToggled(bool checked)
{
    modeLabel_->setText(checked ? "Mode: Live Sensor" : "Mode: Simulation");
}


void SensorsCalibrationWidget::setLiveAngles(double pitch, double roll)
{
    latestPitch_ = pitch;
    latestRoll_ = roll;
}