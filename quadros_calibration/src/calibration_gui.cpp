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
#include "quadros_calibration/motor_calibration_widget.hpp"
#include "quadros_calibration/calibration_dialog.hpp"
#include "quadros_calibration/msg/motor_speed.hpp"
#include "rclcpp/rclcpp.hpp"


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