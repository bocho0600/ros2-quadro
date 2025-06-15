#include <QApplication>
#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSlider>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QWidget>
#include <QStackedLayout>
#include <QTimer>
#include <QThread>
#include <array>
#include <memory>
#include <QCheckBox>

#include "quadros_calibration/sensors_calibration_widget.hpp"
#include "rclcpp/rclcpp.hpp"



#include "quadros_calibration/msg/motor_speed.hpp"

class MotorCalibrationWidget : public QWidget {
    Q_OBJECT
public:
    explicit MotorCalibrationWidget(QWidget *parent = nullptr)
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

    std::array<int, 4> getMotorSpeeds() const {
        std::array<int, 4> speeds;
        for (int i = 0; i < 4; ++i)
            speeds[i] = sliders_[i]->isEnabled() ? sliders_[i]->value() : 0;
        return speeds;
    }

    bool isArmed() const {
        return armed_;
    }

private slots:
    void toggleArm(bool checked) {
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

private: 
    QSlider* sliders_[4];
    QSlider* all_slider_;
    QCheckBox* arm_switch_;
    bool armed_;
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
        publisher_ = node_->create_publisher<quadros_calibration::msg::MotorSpeed>("/quadros/set/motors", 10);

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
        quadros_calibration::msg::MotorSpeed msg; // Create a new message instance
        msg.armed = motor_widget_->isArmed();  // <-- Set the armed field
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

    auto node = std::make_shared<rclcpp::Node>("motor_calibration_gui"); // Create a ROS2 node

    std::cout << "Node created and GUI starting..." << std::endl;

    // Start ROS2 spinning in a background thread
    RosSpinThread ros_thread(node); 
    ros_thread.start(); // Start the thread

    CalibrationDialog dialog(node); // Pass the node to the dialog
    dialog.show(); // Show the dialog

    int ret = app.exec();

    rclcpp::shutdown();
    ros_thread.quit();
    ros_thread.wait();

    return ret;
}

#include "calibration_gui.moc"