#include "quadros_calibration/motor_calibration_widget.hpp"
#include <QFrame>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QCheckBox>

MotorCalibrationWidget::MotorCalibrationWidget(QWidget *parent)
    : QWidget(parent), armed_(false)
{
    QVBoxLayout *main_vbox = new QVBoxLayout();
    QHBoxLayout *layout = new QHBoxLayout();
    QStringList slider_titles = {"Motor 1", "Motor 2", "Motor 3", "Motor 4"};
    for (int i = 0; i < 4; ++i) {
        QVBoxLayout *vbox = new QVBoxLayout();

        QLabel *title_label = new QLabel(slider_titles[i]);
        title_label->setAlignment(Qt::AlignCenter);
        title_label->setStyleSheet("font-size: 12pt; font-weight: bold;");

        QLabel *value_label = new QLabel("0%");
        value_label->setAlignment(Qt::AlignCenter);
        value_label->setStyleSheet("font-size: 10pt;");

        QSlider *slider = new QSlider(Qt::Vertical);
        slider->setMinimum(0);
        slider->setMaximum(100);
        slider->setValue(0);
        slider->setFixedWidth(60);

        slider->setStyleSheet(
            "QSlider::groove:vertical {"
            "    background: #bbb;"
            "    width: 30px;"
            "    border-radius: 8px;"
            "}"
            "QSlider::handle:vertical {"
            "    background: #3880ff;"
            "    height: 30px;"
            "    margin: 0 -10px;"
            "    border-radius: 8px;"
            "}"
            "QSlider::sub-page:vertical {"
            "    background: #bbb;"
            "    border-radius: 8px;"
            "}"
            "QSlider::add-page:vertical {"
            "    background:rgba(56, 129, 255, 0.87);" 
            "    border-radius: 8px;"
            "}"
        );

        connect(slider, &QSlider::valueChanged, [value_label](int val) {
            value_label->setText(QString("%1%").arg(val));
        });

        vbox->addWidget(title_label);
        vbox->addWidget(value_label);
        vbox->addWidget(slider);

        layout->addLayout(vbox);

        sliders_[i] = slider;
    }

    // --- Add a vertical line separator ---
    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::VLine);
    line->setFrameShadow(QFrame::Sunken);
    line->setLineWidth(2);
    layout->addWidget(line);

    // --- Add spacing before All Motors slider ---
    layout->addSpacing(40); // Adjust this value for more/less space

    // --- Add All Motors slider on the right ---
    QVBoxLayout *all_vbox = new QVBoxLayout();
    QLabel *all_label = new QLabel("All Motors");
    all_label->setAlignment(Qt::AlignCenter);
    all_label->setStyleSheet("font-size: 14pt; font-weight: bold;");

    QLabel *all_value_label = new QLabel("0%");
    all_value_label->setAlignment(Qt::AlignCenter);
    all_value_label->setStyleSheet("font-size: 12pt;");

    all_slider_ = new QSlider(Qt::Vertical);
    all_slider_->setMinimum(0);
    all_slider_->setMaximum(100);
    all_slider_->setValue(0);
    all_slider_->setFixedWidth(80);

    all_slider_->setStyleSheet(
        "QSlider::groove:vertical {"
        "    background: #bbb;"
        "    width: 30px;"
        "    border-radius: 8px;"
        "}"
        "QSlider::handle:vertical {"
        "    background:rgb(221, 0, 0);"
        "    height: 30px;"
        "    margin: 0 -10px;"
        "    border-radius: 8px;"
        "}"
        "QSlider::sub-page:vertical {"
        "    background: #bbb;"
        "    border-radius: 8px;"
        "}"
        "QSlider::add-page:vertical {"
        "    background:rgb(255, 0, 0);"
        "    border-radius: 8px;"
        "}"
    );

    connect(all_slider_, &QSlider::valueChanged, [=](int val) {
        all_value_label->setText(QString("%1%").arg(val));
        if (armed_) {
            for (int i = 0; i < 4; ++i) {
                sliders_[i]->setValue(val);
            }
        }
    });

    all_vbox->addWidget(all_label);
    all_vbox->addWidget(all_value_label);
    all_vbox->addWidget(all_slider_);

    layout->addLayout(all_vbox); // Add the all-motors slider to the right

    // --- End All Motors slider ---

    // Slide switch for Arm/Disarm
    QHBoxLayout *switch_layout = new QHBoxLayout();
    QLabel *arm_label = new QLabel("Arm");
    arm_label->setStyleSheet("font-size: 12pt;");
    QLabel *disarm_label = new QLabel("Disarm");
    disarm_label->setStyleSheet("font-size: 12pt;");
    arm_switch_ = new QCheckBox();
    arm_switch_->setChecked(false);
    arm_switch_->setStyleSheet(
        "QCheckBox::indicator { width: 40px; height: 20px; }"
        "QCheckBox::indicator:unchecked {"
        "    image: url();"
        "    border-radius: 10px;"
        "    background: #bbb;"
        "}"
        "QCheckBox::indicator:checked {"
        "    image: url();"
        "    border-radius: 10px;"
        "    background: #3880ff;"
        "}"
    );
    switch_layout->addWidget(arm_label);
    switch_layout->addWidget(arm_switch_);
    switch_layout->addWidget(disarm_label);
    switch_layout->setAlignment(Qt::AlignCenter);

    connect(arm_switch_, &QCheckBox::toggled, this, &MotorCalibrationWidget::toggleArm);

    main_vbox->addLayout(switch_layout);
    main_vbox->addLayout(layout);
    setLayout(main_vbox);

    // Initialize sliders as disabled
    for (int i = 0; i < 4; ++i) {
        sliders_[i]->setEnabled(false);
    }
    all_slider_->setEnabled(false);
}

std::array<int, 4> MotorCalibrationWidget::getMotorSpeeds() const {
    std::array<int, 4> speeds;
    for (int i = 0; i < 4; ++i)
        speeds[i] = sliders_[i]->isEnabled() ? sliders_[i]->value() : 0;
    return speeds;
}

bool MotorCalibrationWidget::isArmed() const {
    return armed_;
}

void MotorCalibrationWidget::toggleArm(bool checked) {
    armed_ = checked;
    if (!armed_) {
        for (int i = 0; i < 4; ++i) {
            sliders_[i]->setValue(0); // Reset sliders to 0 when disarmed
            sliders_[i]->setEnabled(false); // Disable sliders when disarmed
        }
        all_slider_->setValue(0);
        all_slider_->setEnabled(false);
    } else {
        for (int i = 0; i < 4; ++i) {
            sliders_[i]->setEnabled(true);
        }
        all_slider_->setEnabled(true);
    }
}