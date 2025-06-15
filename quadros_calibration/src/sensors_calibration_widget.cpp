
#include "quadros_calibration/sensors_calibration_widget.hpp"
//#include "sensor_calibration_widget.moc"

SensorsCalibrationWidget::SensorsCalibrationWidget(QWidget *parent)
    : QWidget(parent)
{
    QVBoxLayout *layout = new QVBoxLayout();
    QLabel *label = new QLabel("Sensor Calibration Placeholder");
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("font-size: 16pt; font-style: italic;");
    layout->addWidget(label);
    setLayout(layout);
};
