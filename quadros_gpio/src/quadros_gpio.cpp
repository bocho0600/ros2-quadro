#include "rclcpp/rclcpp.hpp"
#include "quadros_gpio/msg/motor_speed.hpp"

#include <array>
#include <pigpio.h>
#include <algorithm> // for std::clamp

class MotorPwmNode : public rclcpp::Node
{
public:
    MotorPwmNode()
    : Node("motor_pwm_node")
    {
        motor_pins_ = {17, 18, 27, 22}; // BCM GPIO pins

        if (gpioInitialise() < 0) {
            RCLCPP_FATAL(this->get_logger(), "pigpio initialization failed!");
            throw std::runtime_error("pigpio init failed");
        }

        for (int pin : motor_pins_) {
            gpioSetMode(pin, PI_OUTPUT);
        }

        sub_ = this->create_subscription<quadros_gpio::msg::MotorSpeed>(
            "/quadros/set/motors", 10,
            std::bind(&MotorPwmNode::motor_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Motor PWM Node (servo-style) started.");
    }

    ~MotorPwmNode() override
    {
        for (int pin : motor_pins_) {
            gpioServo(pin, 0); // Stop signal
        }
        gpioTerminate();
    }

private:
    void motor_callback(const quadros_gpio::msg::MotorSpeed::SharedPtr msg) // This function tracks the motor speeds
    {
        std::array<float, 4> percentages = {
            msg->motor_speed_1,
            msg->motor_speed_2,
            msg->motor_speed_3,
            msg->motor_speed_4
        };
        bool armed = msg->armed;
        if (armed) {
            for (size_t i = 0; i < percentages.size(); ++i) {
                float percent = std::clamp(percentages[i], 0.0f, 100.0f);
                int microseconds = static_cast<int>(1000 + (percent / 100.0f) * 1000); // 1000–2000 µs
                gpioServo(motor_pins_[i], microseconds);
                RCLCPP_INFO(this->get_logger(), "ESC[%ld] set to %.1f%% => %d µs", i+1, percent, microseconds);
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Motors NOT armed, setting to 0%");
            for (int pin : motor_pins_) {
                gpioServo(pin, 0); // Stop signal
            }
        }

    }

    std::array<int, 4> motor_pins_;
    rclcpp::Subscription<quadros_gpio::msg::MotorSpeed>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPwmNode>());
    rclcpp::shutdown();
    return 0;
}
