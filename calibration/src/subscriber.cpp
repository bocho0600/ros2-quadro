#include "rclcpp/rclcpp.hpp"
#include "calibration/msg/my_msg.hpp"

class MySubscriber : public rclcpp::Node
{
public:
  MySubscriber() : Node("my_subscriber")
  {
    sub_ = this->create_subscription<calibration::msg::MyMsg>(
      "my_topic", 10,
      [this](const calibration::msg::MyMsg::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<calibration::msg::MyMsg>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}
