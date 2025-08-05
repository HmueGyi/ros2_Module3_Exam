
#include "rclcpp/rclcpp.hpp"
#include "no5/msg/hss.hpp"

class CustomSubscriber : public rclcpp::Node
{
public:
  CustomSubscriber()
  : Node("custom_subscriber")
  {
    subscription_ = this->create_subscription<no5::msg::Hss>(
      "custom_topic",
      10,
      std::bind(&CustomSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const no5::msg::Hss::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: number=%d, decimal=%.2f, text='%s'",
                msg->number, msg->decimal, msg->text.c_str());
  }

  rclcpp::Subscription<no5::msg::Hss>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomSubscriber>());
  rclcpp::shutdown();
  return 0;
}