#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class InteractivePublisher : public rclcpp::Node
{
public:
  InteractivePublisher()
  : Node("interactive_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("custom_topic", 10);
    RCLCPP_INFO(this->get_logger(), "Interactive publisher started.");
  }

  void run()
  {
    std::string input;
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());

      std::cout << "[Input] Enter message to publish : ";
      if (!std::getline(std::cin, input)) {
        break;
      }

      if (input.empty()) {
        continue;
      }

      std_msgs::msg::String msg;
      msg.data = input;
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InteractivePublisher>();
  node->run();

  rclcpp::shutdown();
  return 0;
}
