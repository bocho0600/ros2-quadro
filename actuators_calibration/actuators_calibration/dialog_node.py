import sys
from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QVBoxLayout, QHBoxLayout, QSlider
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node

class MyDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 + QDialog Example")
        self.resize(600, 400)  # Set initial window size

        main_layout = QVBoxLayout()
        sliders_layout = QHBoxLayout()

        slider_titles = ["Motor 1", "Motor 2", "Motor 3", "Motor 4"]

        for title in slider_titles:
            vbox = QVBoxLayout()

            # Label showing the slider title
            title_label = QLabel(title)
            title_label.setAlignment(Qt.AlignCenter)
            title_label.setStyleSheet("font-size: 14pt; font-weight: bold;")

            # Label showing the slider value with percentage
            value_label = QLabel("0%")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setStyleSheet("font-size: 12pt;")

            # Slider
            slider = QSlider(Qt.Vertical)
            slider.setMinimum(0)
            slider.setMaximum(100)
            slider.setValue(0)
            slider.setFixedWidth(60)

            # Update value label on slider change
            slider.valueChanged.connect(lambda val, label=value_label: label.setText(f"{val}%"))

            # Add widgets to layout
            vbox.addWidget(title_label)
            vbox.addWidget(value_label)
            vbox.addWidget(slider)

            sliders_layout.addLayout(vbox)

        main_layout.addLayout(sliders_layout)
        self.setLayout(main_layout)

class DummyNode(Node):
    def __init__(self):
        super().__init__('motor_calibration_gui')
        self.get_logger().info("ROS 2 Node Started (with Qt Dialog)")

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setApplicationName("ROS 2 + QDialog Example")
    ros_node = DummyNode()

    dialog = MyDialog()
    dialog.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.1))
    timer.start(100)

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
