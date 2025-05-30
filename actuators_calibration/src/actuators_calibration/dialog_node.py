import sys
from PyQt5.QtWidgets import (
    QApplication, QDialog, QLabel, QVBoxLayout, QHBoxLayout,
    QSlider, QListWidget, QListWidgetItem, QWidget, QStackedLayout
)
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
#from your_package_name.msg import MotorSpeeds  # Import your custom msg

class MotorCalibrationWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout()
        slider_titles = ["Motor 1", "Motor 2", "Motor 3", "Motor 4"]

        for title in slider_titles:
            vbox = QVBoxLayout()

            # Title label
            title_label = QLabel(title)
            title_label.setAlignment(Qt.AlignCenter)
            title_label.setStyleSheet("font-size: 14pt; font-weight: bold;")

            # Value label
            value_label = QLabel("0%")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setStyleSheet("font-size: 12pt;")

            # Slider
            slider = QSlider(Qt.Vertical)
            slider.setMinimum(0)
            slider.setMaximum(100)
            slider.setValue(0)
            slider.setFixedWidth(60)
            slider.valueChanged.connect(lambda val, label=value_label: label.setText(f"{val}%"))

            vbox.addWidget(title_label)
            vbox.addWidget(value_label)
            vbox.addWidget(slider)
            layout.addLayout(vbox)

        self.setLayout(layout)

class SensorsCalibrationWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()
        label = QLabel("Sensor Calibration Placeholder")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 16pt; font-style: italic;")
        layout.addWidget(label)
        self.setLayout(layout)

class MyDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quadro UAV Calibration Tools")
        self.resize(800, 500)

        main_layout = QHBoxLayout()

        # Sidebar menu
        self.menu = QListWidget()
        self.menu.setFixedWidth(180)
        self.menu.addItem(QListWidgetItem("Motor Calibration"))
        self.menu.addItem(QListWidgetItem("Sensors Calibration"))
        self.menu.addItem(QListWidgetItem("Baterry"))
        self.menu.currentRowChanged.connect(self.switch_mode)

        # Stacked layout for content
        self.stack = QStackedLayout()
        self.motor_widget = MotorCalibrationWidget()
        self.sensor_widget = SensorsCalibrationWidget()
        self.stack.addWidget(self.motor_widget)
        self.stack.addWidget(self.sensor_widget)

        # Combine layouts
        main_layout.addWidget(self.menu)
        content_widget = QWidget()
        content_widget.setLayout(self.stack)
        main_layout.addWidget(content_widget)

        self.setLayout(main_layout)

    def switch_mode(self, index):
        self.stack.setCurrentIndex(index)

class DummyNode(Node):
    def __init__(self):
        super().__init__('motor_calibration_gui')
        self.get_logger().info("ROS 2 Node Started (with Qt Dialog)")

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setApplicationName("ROS 2 + QDialog Calibration GUI")
    ros_node = DummyNode()

    dialog = MyDialog()
    dialog.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.05))
    timer.start(100)

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
