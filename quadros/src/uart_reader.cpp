#include <rclcpp/rclcpp.hpp>

#include "quadros/msg/telemetry.hpp"
#include "quadros_calibration/msg/motor_speed.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <sstream>
#include <vector>
#include <chrono> // <-- Add this
using namespace std::chrono_literals; // <-- And this for 5ms of delay
using std::placeholders::_1; 

class UARTReader : public rclcpp::Node
{
public:
    UARTReader() : Node("uart_reader")
    {
        publisher_ = this->create_publisher<quadros::msg::Telemetry>("quadros/state/telemetry", 10);
        motor_subscriber_ = this->create_subscription<quadros_calibration::msg::MotorSpeed>(
            "quadros/set/motors", 10,
            std::bind(&UARTReader::motor_callback, this, _1)
        );



        // Open serial port
        serial_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            rclcpp::shutdown();
            return;
        }

        struct termios tty{};
        tcgetattr(serial_fd_, &tty);
        cfsetospeed(&tty, B230400);
        cfsetispeed(&tty, B230400);
        tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;              // 8-bit chars
        tty.c_cflag &= ~PARENB;          // No parity
        tty.c_cflag &= ~CSTOPB;          // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;         // No flow control
        tty.c_lflag = 0;                 // No canonical mode
        tty.c_oflag = 0;                 // No remapping
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;
        tcsetattr(serial_fd_, TCSANOW, &tty);

        timer_ = this->create_wall_ticmer(5ms, std::bind(&UARTReader::read_serial, this)); // Set timer to read serial data every 5 milliseconds
    }

    ~UARTReader() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    void write_serial(const std::string &data){
        if (serial_fd_ >= 0) {
            write(serial_fd_, data.c_str(), data.size());
        }
    }

    void motor_callback(const quadros_calibration::msg::MotorSpeed::SharedPtr msg)
    {
        if (!msg->armed) {
            RCLCPP_WARN(this->get_logger(), "Motors not armed. Command ignored.");
            return;
        }

        std::ostringstream oss;
        oss << "<MOT," 
            << static_cast<int>(msg->motor_speed_1) << ","
            << static_cast<int>(msg->motor_speed_2) << ","
            << static_cast<int>(msg->motor_speed_3) << ","
            << static_cast<int>(msg->motor_speed_4) << ">";

        write_serial(oss.str());
        // RCLCPP_INFO(this->get_logger(), "Sent motor command: %s", oss.str().c_str());
    }

    void read_serial()
    {   
        char buf[128];
        int n = read(serial_fd_, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = 0;
            std::string line(buf);

            // Look for message format: <TEL,-0.47,-1.23>
            if (line.find("<TEL") != std::string::npos && line.find('>') != std::string::npos) {
                size_t start = line.find('<') + 1;
                size_t end = line.find('>');
                std::string content = line.substr(start, end - start);

                std::stringstream ss(content);
                std::string item;
                std::vector<std::string> parts;

                while (std::getline(ss, item, ',')) {
                    parts.push_back(item);
                }

                if (parts.size() == 3 && parts[0] == "TEL") {
                    try {
                        float roll = std::stof(parts[1]);
                        float pitch = std::stof(parts[2]);

                        auto msg_telemetry = quadros::msg::Telemetry();
                        msg_telemetry.roll_angle = roll;
                        msg_telemetry.pitch_angle = pitch;
                        publisher_->publish(msg_telemetry);
                       // RCLCPP_INFO(this->get_logger(), "Published Roll=%.2f, Pitch=%.2f", roll, pitch);
                    } catch (...) {
                       // RCLCPP_WARN(this->get_logger(), "Failed to parse floats from: %s", content.c_str());
                    }
                }
            }
        }
    }

    rclcpp::Publisher<quadros::msg::Telemetry>::SharedPtr publisher_;
    rclcpp::Subscription<quadros_calibration::msg::MotorSpeed>::SharedPtr motor_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_fd_;
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UARTReader>()); // Run the node
    rclcpp::shutdown(); // Clean up and exit after spinning
    return 0;
}
