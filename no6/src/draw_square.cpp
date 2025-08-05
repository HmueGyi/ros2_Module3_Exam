#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DrawSquare : public rclcpp::Node
{
public:
  DrawSquare() : Node("draw_square_node"), step_(0), phase_(0)
  // step_ → တစ်ချောင်းစီမှာလှုပ်ရှားတာကို count လုပ်တယ်။

  // phase_ → 0 ဆိုရင် တည့်တည့်သွား phase, 1 ဆိုရင် လှည့်ဖို့ phase ဖြစ်တယ်။
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&DrawSquare::publish_cmd, this));

    forward_steps_ = 20;  // ~2 seconds forward
    turn_steps_ = 16;     // ~1.6 seconds to rotate 90 degrees
  }

private:
  void publish_cmd()
  {
    geometry_msgs::msg::Twist msg;

    if (phase_ == 0 && step_ < forward_steps_) {
      msg.linear.x = 2.0;
      msg.angular.z = 0.0;
    }
    else if (phase_ == 1 && step_ < turn_steps_) {
      msg.linear.x = 0.0;
      msg.angular.z = 1.57 / (turn_steps_ * 0.1);  // 90 deg in 1.6s
    }

    step_++;

    if ((phase_ == 0 && step_ >= forward_steps_) ||
        (phase_ == 1 && step_ >= turn_steps_)) {
      step_ = 0;
      phase_ = (phase_ + 1) % 2;  // Alternate between forward and turn
    }

    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int step_;
  int phase_;
  int forward_steps_;
  int turn_steps_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrawSquare>());
  return 0;
}
