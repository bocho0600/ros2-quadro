#include "rclcpp/rclcpp.hpp"
#include "calobration/msg/my_msg.hpp"

class MySubscriber : public rclcpp::Node
{
public:
  MySubscriber() : Node("my_subscriber")
  {
    sub_ = this->create_subscription<calobration::msg::MyMsg>(
      "my_topic", 10,
      [this](const calobration::msg::MyMsg::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<calobration::msg::MyMsg>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}
