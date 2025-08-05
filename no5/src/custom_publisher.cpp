
#include "rclcpp/rclcpp.hpp"
#include "no5/msg/hss.hpp"  // Include your custom msg header

using std::placeholders::_1;

class CustomPublisher : public rclcpp::Node
{
public:
  CustomPublisher()
  : Node("custom_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<no5::msg::Hss>("custom_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CustomPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = no5::msg::Hss();
    message.number = count_;
    message.decimal = count_ * 0.5;
    message.text = "Message number " + std::to_string(count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: number=%d, decimal=%.2f, text='%s'",
                message.number, message.decimal, message.text.c_str());
    publisher_->publish(message);
    count_++;
  }

  rclcpp::Publisher<no5::msg::Hss>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomPublisher>());
  rclcpp::shutdown();
  return 0;
}