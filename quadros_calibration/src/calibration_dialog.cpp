#include "quadros_calibration/calibration_dialog.hpp"

CalibrationDialog::CalibrationDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QDialog(parent), node_(node)
{
    setWindowTitle("Quadro UAV Calibration Tools");
    resize(800, 500);

    QHBoxLayout *main_layout = new QHBoxLayout();

    // Sidebar menu
    menu_ = new QListWidget();
    menu_->setFixedWidth(150);
    menu_->addItem(new QListWidgetItem("Motor Control"));
    menu_->addItem(new QListWidgetItem("Sensors Calibration"));
    menu_->addItem(new QListWidgetItem("Baterry Status"));
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
    publisher_ = node_->create_publisher<quadros::msg::MotorSpeed>("/quadros/set/motors", 10);
    calibration_publisher_ = node_->create_publisher<quadros::msg::Calibration>("/quadros/set/calibration", 10);
    telemetry_sub_ = node_->create_subscription<quadros::msg::Telemetry>(
        "/quadros/state/telemetry", 10,
        std::bind(&CalibrationDialog::onTelemetryReceived, this, std::placeholders::_1)
    );
    // Timer for periodic publishing
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CalibrationDialog::publishMotorSpeeds);
    timer_->start(100); // 100 ms

    connect(sensor_widget_, &SensorsCalibrationWidget::calibrationRequested, this, &CalibrationDialog::publishCalibration);
}

void CalibrationDialog::onTelemetryReceived(const quadros::msg::Telemetry::SharedPtr msg)
{
    float roll = msg->roll_angle;
    float pitch = msg->pitch_angle;
    sensor_widget_->setLiveAngles(pitch, roll);
}

void CalibrationDialog::switchMode(int index) {
    stack_->setCurrentIndex(index);
}

void CalibrationDialog::publishMotorSpeeds() {
    if (stack_->currentIndex() != 0) return; // Only publish in Motor Calibration tab

    auto speeds = motor_widget_->getMotorSpeeds();
    quadros::msg::MotorSpeed msg; // Create a new message instance
    msg.armed = motor_widget_->isArmed();  // <-- Set the armed field
    msg.motor_speed_1 = speeds[0];
    msg.motor_speed_2 = speeds[1];
    msg.motor_speed_3 = speeds[2];
    msg.motor_speed_4 = speeds[3];
    publisher_->publish(msg);
}

void CalibrationDialog::publishCalibration() {
    if (stack_->currentIndex() != 1) return; // Only publish in Sensors Calibration tab

    auto calibration = sensor_widget_->getCalibration();
    quadros::msg::Calibration msg;
    msg.roll_pid = std::vector<float>(calibration.roll_pid.begin(), calibration.roll_pid.end());
    msg.pitch_pid = std::vector<float>(calibration.pitch_pid.begin(), calibration.pitch_pid.end());
    msg.yaw_pid = std::vector<float>(calibration.yaw_pid.begin(), calibration.yaw_pid.end());
    msg.kalman_filter_qr = std::vector<float>(calibration.kalman_qr.begin(), calibration.kalman_qr.end());

    // Publish the message to the calibration topic
    calibration_publisher_->publish(msg);
}
