#include "quadros_calibration/sensors_calibration_widget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>
#include <QTimer>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <cmath>  // For sin(), cos()
#include <array>

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

    // Helper lambda to create a PID row for a given label and default values
    auto createPidSection = [this](const QString &title, double pDefault, double iDefault, double dDefault,
                                   QDoubleSpinBox **pSpinOut, QDoubleSpinBox **iSpinOut, QDoubleSpinBox **dSpinOut) {
        QVBoxLayout *sectionLayout = new QVBoxLayout();
        QLabel *sectionLabel = new QLabel(title, this);
        sectionLabel->setAlignment(Qt::AlignCenter);
        QFont sectionFont = sectionLabel->font();
        sectionFont.setBold(true);
        sectionLabel->setFont(sectionFont);
        sectionLayout->addWidget(sectionLabel);
        QHBoxLayout *pidLayout = new QHBoxLayout();
        QVBoxLayout *pLayout = new QVBoxLayout();
        QLabel *pLabel = new QLabel("P", this);
        QDoubleSpinBox *pSpin = new QDoubleSpinBox(this);
        pSpin->setRange(-10000.0, 10000.0);
        pSpin->setDecimals(2);
        pSpin->setSingleStep(0.01);
        pSpin->setFixedWidth(60);
        pSpin->setValue(pDefault);
        pLayout->addWidget(pLabel);
        pLayout->addWidget(pSpin);
        QVBoxLayout *iLayout = new QVBoxLayout();
        QLabel *iLabel = new QLabel("I", this);
        QDoubleSpinBox *iSpin = new QDoubleSpinBox(this);
        iSpin->setRange(-10000.0, 10000.0);
        iSpin->setDecimals(2);
        iSpin->setSingleStep(0.01);
        iSpin->setFixedWidth(60);
        iSpin->setValue(iDefault);
        iLayout->addWidget(iLabel);
        iLayout->addWidget(iSpin);
        QVBoxLayout *dLayout = new QVBoxLayout();
        QLabel *dLabel = new QLabel("D", this);
        QDoubleSpinBox *dSpin = new QDoubleSpinBox(this);
        dSpin->setRange(-10000.0, 10000.0);
        dSpin->setDecimals(2);
        dSpin->setSingleStep(0.01);
        dSpin->setFixedWidth(60);
        dSpin->setValue(dDefault);
        dLayout->addWidget(dLabel);
        dLayout->addWidget(dSpin);
        pidLayout->addLayout(pLayout);
        pidLayout->addLayout(iLayout);
        pidLayout->addLayout(dLayout);
        pidLayout->addStretch();
        sectionLayout->addLayout(pidLayout);
        // if 
        if (pSpinOut) *pSpinOut = pSpin; // assign the spin box to the output pointer
        if (iSpinOut) *iSpinOut = iSpin;
        if (dSpinOut) *dSpinOut = dSpin;
        return sectionLayout;
    };

    // Add Roll, Pitch, Yaw PID sections with lines between them and default values
    rightLayout->addSpacing(10);
    rightLayout->addLayout(createPidSection("Roll", 0.6, 3.5, 0.03, &rollPSpin_, &rollISpin_, &rollDSpin_));
    QFrame *line1 = new QFrame();
    line1->setFrameShape(QFrame::HLine);
    line1->setFrameShadow(QFrame::Sunken);
    rightLayout->addWidget(line1);
    rightLayout->addLayout(createPidSection("Pitch", 0.6, 3.5, 0.03, &pitchPSpin_, &pitchISpin_, &pitchDSpin_));
    QFrame *line2 = new QFrame();
    line2->setFrameShape(QFrame::HLine);
    line2->setFrameShadow(QFrame::Sunken);
    rightLayout->addWidget(line2);
    rightLayout->addLayout(createPidSection("Yaw", 2.0, 12.0, 0.0, &yawPSpin_, &yawISpin_, &yawDSpin_));
    rightLayout->addSpacing(20);
    QFrame *line3 = new QFrame();
    line3->setFrameShape(QFrame::HLine);
    line3->setFrameShadow(QFrame::Sunken);
    rightLayout->addWidget(line3);

    // Kalman Filter Section
    QLabel *kalmanLabel = new QLabel("Kalman Filter", this);
    kalmanLabel->setAlignment(Qt::AlignCenter);
    QFont kalmanFont = kalmanLabel->font();
    kalmanFont.setBold(true);
    kalmanLabel->setFont(kalmanFont);
    rightLayout->addWidget(kalmanLabel);
    QHBoxLayout *kalmanLayout = new QHBoxLayout();
    QVBoxLayout *qLayout = new QVBoxLayout();
    QLabel *qLabel = new QLabel("Q", this);
    kalmanQSpin_ = new QDoubleSpinBox(this);
    kalmanQSpin_->setRange(0.0, 10000.0);
    kalmanQSpin_->setDecimals(2); //
    kalmanQSpin_->setSingleStep(0.01);
    kalmanQSpin_->setFixedWidth(60);
    kalmanQSpin_->setValue(1.0); // Default value for Q
    qLayout->addWidget(qLabel);
    qLayout->addWidget(kalmanQSpin_);
    QVBoxLayout *rLayout = new QVBoxLayout();
    QLabel *rLabel = new QLabel("R", this);
    kalmanRSpin_ = new QDoubleSpinBox(this);
    kalmanRSpin_->setRange(0.0, 10000.0);
    kalmanRSpin_->setDecimals(2);
    kalmanRSpin_->setSingleStep(0.01);
    kalmanRSpin_->setFixedWidth(60);
    kalmanRSpin_->setValue(1.0); // Default value for R
    rLayout->addWidget(rLabel);
    rLayout->addWidget(kalmanRSpin_);
    kalmanLayout->addLayout(qLayout);
    kalmanLayout->addLayout(rLayout);
    kalmanLayout->addStretch();
    rightLayout->addLayout(kalmanLayout);
    rightLayout->addSpacing(20);

    // Calibrate Button
    QPushButton *calibrateButton = new QPushButton("Calibrate", this);
    calibrateButton->setFixedSize(120, 40);
    // set font
    QFont buttonFont = calibrateButton->font();
    buttonFont.setPointSize(10);
    buttonFont.setBold(true);
    calibrateButton->setFont(buttonFont);
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    buttonLayout->addWidget(calibrateButton);
    buttonLayout->addStretch();
    rightLayout->addLayout(buttonLayout);
    // Connect calibrate button to emit calibrationRequested signal
    connect(calibrateButton, &QPushButton::clicked, this, [this]() {
        emit calibrationRequested();
    });
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
    // Only live data mode
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
    plotPitch_->replot();
    plotRoll_->replot();
}


void SensorsCalibrationWidget::setLiveAngles(double pitch, double roll)
{
    latestPitch_ = pitch;
    latestRoll_ = roll;
}

CalibrationValues SensorsCalibrationWidget::getCalibration() const {
    CalibrationValues values;
    values.roll_pid[0] = rollPSpin_->value();
    values.roll_pid[1] = rollISpin_->value();
    values.roll_pid[2] = rollDSpin_->value();
    values.pitch_pid[0] = pitchPSpin_->value();
    values.pitch_pid[1] = pitchISpin_->value();
    values.pitch_pid[2] = pitchDSpin_->value();
    values.yaw_pid[0] = yawPSpin_->value();
    values.yaw_pid[1] = yawISpin_->value();
    values.yaw_pid[2] = yawDSpin_->value();
    values.kalman_qr[0] = kalmanQSpin_->value();
    values.kalman_qr[1] = kalmanRSpin_->value();
    return values;
}