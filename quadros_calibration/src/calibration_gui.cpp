#include <QApplication>
#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSlider>
#include <QListWidget>
#include <QListWidgetItem>
#include <QWidget>
#include <QStackedLayout>
#include <QTimer>
#include <QThread>
#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "quadros_calibration/msg/motor_speed.hpp"

class MotorCalibrationWidget : public QWidget {
    Q_OBJECT
public:
    explicit MotorCalibrationWidget(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        QHBoxLayout *layout = new QHBoxLayout();
        QStringList slider_titles = {"Motor 1", "Motor 2", "Motor 3", "Motor 4"};
        for (int i = 0; i < 4; ++i) {
            QVBoxLayout *vbox = new QVBoxLayout();

            QLabel *title_label = new QLabel(slider_titles[i]);
            title_label->setAlignment(Qt::AlignCenter);
            title_label->setStyleSheet("font-size: 14pt; font-weight: bold;");

            QLabel *value_label = new QLabel("0%");
            value_label->setAlignment(Qt::AlignCenter);
            value_label->setStyleSheet("font-size: 12pt;");

            QSlider *slider = new QSlider(Qt::Vertical);
            slider->setMinimum(0);
            slider->setMaximum(100);
            slider->setValue(0);
            slider->setFixedWidth(80); // Increased width for a thicker slider

            // Make the slider handle thicker using stylesheet
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
                "    background: #bbb;"   // The trace color
                "    border-radius: 8px;"
                "}"
                "QSlider::add-page:vertical {"
                "    background: #3880ff;"      // The unfilled part
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
        setLayout(layout);
    }

    std::array<int, 4> getMotorSpeeds() const {
        std::array<int, 4> speeds;
        for (int i = 0; i < 4; ++i)
            speeds[i] = sliders_[i]->value();
        return speeds;
    }

private:
    QSlider* sliders_[4];
};

class SensorsCalibrationWidget : public QWidget {
    Q_OBJECT
public:
    explicit SensorsCalibrationWidget(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        QVBoxLayout *layout = new QVBoxLayout();
        QLabel *label = new QLabel("Sensor Calibration Placeholder");
        label->setAlignment(Qt::AlignCenter);
        label->setStyleSheet("font-size: 16pt; font-style: italic;");
        layout->addWidget(label);
        setLayout(layout);
    }
};

class CalibrationDialog : public QDialog {
    Q_OBJECT
public:
    explicit CalibrationDialog(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr)
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
        publisher_ = node_->create_publisher<quadros_calibration::msg::MotorSpeed>("MotorSpeed", 10);

        // Timer for periodic publishing
        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, &CalibrationDialog::publishMotorSpeeds);
        timer_->start(100); // 100 ms
    }

private slots:
    void switchMode(int index) {
        stack_->setCurrentIndex(index);
    }

    void publishMotorSpeeds() {
        if (stack_->currentIndex() != 0) return; // Only publish in Motor Calibration tab

        auto speeds = motor_widget_->getMotorSpeeds();
        quadros_calibration::msg::MotorSpeed msg;
        msg.motor_speed_1 = speeds[0];
        msg.motor_speed_2 = speeds[1];
        msg.motor_speed_3 = speeds[2];
        msg.motor_speed_4 = speeds[3];
        publisher_->publish(msg);
    }

private:
    QListWidget *menu_;
    QStackedLayout *stack_;
    MotorCalibrationWidget *motor_widget_;
    SensorsCalibrationWidget *sensor_widget_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<quadros_calibration::msg::MotorSpeed>::SharedPtr publisher_;
    QTimer *timer_;
};

class RosSpinThread : public QThread {
    Q_OBJECT
public:
    RosSpinThread(std::shared_ptr<rclcpp::Node> node) : node_(node) {}
    void run() override {
        rclcpp::spin(node_);
    }
private:
    std::shared_ptr<rclcpp::Node> node_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("motor_calibration_gui");

    // Start ROS2 spinning in a background thread
    RosSpinThread ros_thread(node);
    ros_thread.start();

    CalibrationDialog dialog(node);
    dialog.show();

    int ret = app.exec();

    rclcpp::shutdown();
    ros_thread.quit();
    ros_thread.wait();

    return ret;
}

#include "calibration_gui.moc"
