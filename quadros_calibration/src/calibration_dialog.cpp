#include "quadros_calibration/calibration_dialog.hpp"

CalibrationDialog::CalibrationDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QDialog(parent), node_(node)
{
    setWindowTitle("Quadro UAV Calibration Tools");
    resize(800, 500);

    QHBoxLayout *main_layout = new QHBoxLayout();

    // Sidebar menu
    menu_ = new QListWidget();
    menu_->setFixedWidth(180);
    menu_->addItem(new QListWidgetItem("Motor Calibration"));
    menu_->addItem(new QListWidgetItem("Sensors Calibration"));
    menu_->addItem(new QListWidgetItem("Baterry"));
    connect(menu_, &QListWidget::currentRowChanged, this, &CalibrationDialog::switchMode);

    // Stacked layout for content
    stack_ = new QStackedLayout();
    motor_widget_ = new MotorCalibrationWidget();
    sensor_widget_ = new SensorsCalibrationWidget();
    stack_->addWidget(motor_widget_);
    stack_->addWidget(sensor_widget_);

    QWidget *content_widget = new QWidget();
    content_widget->setLayout(stack_);

    main_layout->addWidget(menu_);
    main_layout->addWidget(content_widget);

    setLayout(main_layout);

    // ROS2 publisher
    publisher_ = node_->create_publisher<quadros_calibration::msg::MotorSpeed>("/quadros/set/motors", 10);

    // Timer for periodic publishing
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CalibrationDialog::publishMotorSpeeds);
    timer_->start(100); // 100 ms
}

void CalibrationDialog::switchMode(int index) {
    stack_->setCurrentIndex(index);
}

void CalibrationDialog::publishMotorSpeeds() {
    if (stack_->currentIndex() != 0) return; // Only publish in Motor Calibration tab

    auto speeds = motor_widget_->getMotorSpeeds();
    quadros_calibration::msg::MotorSpeed msg; // Create a new message instance
    msg.armed = motor_widget_->isArmed();  // <-- Set the armed field
    msg.motor_speed_1 = speeds[0];
    msg.motor_speed_2 = speeds[1];
    msg.motor_speed_3 = speeds[2];
    msg.motor_speed_4 = speeds[3];
    publisher_->publish(msg);
}


