#include "rclcpp/rclcpp.hpp"
#include "calobration/msg/my_msg.hpp"

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher() : Node("my_publisher")
  {
    pub_ = this->create_publisher<calobration::msg::MyMsg>("my_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        auto msg = calobration::msg::MyMsg();
        msg.data = "Hello from publisher!";
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
      });
  }

private:
  rclcpp::Publisher<calobration::msg::MyMsg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}
