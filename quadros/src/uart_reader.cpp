#include <rclcpp/rclcpp.hpp>
#include "quadros/msg/telemetry.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <sstream>
#include <vector>
#include <chrono> // <-- Add this
using namespace std::chrono_literals; // <-- And this
using std::placeholders::_1;

class UARTReader : public rclcpp::Node
{
public:
    UARTReader() : Node("uart_reader")
    {
        publisher_ = this->create_publisher<quadros::msg::Telemetry>("quadros/state/telemetry", 10);

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

        timer_ = this->create_wall_timer(5ms, std::bind(&UARTReader::read_serial, this)); // Set timer to read serial data every 5 milliseconds
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

                        auto msg = quadros::msg::Telemetry();
                        msg.roll_angle = roll;
                        msg.pitch_angle = pitch;
                        publisher_->publish(msg);
                       // RCLCPP_INFO(this->get_logger(), "Published Roll=%.2f, Pitch=%.2f", roll, pitch);
                    } catch (...) {
                       // RCLCPP_WARN(this->get_logger(), "Failed to parse floats from: %s", content.c_str());
                    }
                }
            }
        }
    }

    rclcpp::Publisher<quadros::msg::Telemetry>::SharedPtr publisher_;
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
