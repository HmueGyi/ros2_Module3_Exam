#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("turtle_tf2_frame_listener"),
    turtle_spawning_service_ready_(false),
    turtle_spawned_(false)
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");
    turtle_name_ = this->declare_parameter<std::string>("turtle_name", "turtle2");
    spawn_x_ = this->declare_parameter<double>("spawn_x", 5.0);
    spawn_y_ = this->declare_parameter<double>("spawn_y", 5.0);
    spawn_theta_ = this->declare_parameter<double>("spawn_theta", 0.0);

    // Set buffer cache duration to at least 10 seconds for storing TF history
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), std::chrono::seconds(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");

    std::string cmd_topic = "/" + turtle_name_ + "/cmd_vel";
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 1);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    std::string fromFrameRel = target_frame_;  // e.g. "turtle1"
    std::string toFrameRel = turtle_name_;    // e.g. "turtle2"

    if (turtle_spawning_service_ready_) {
      if (turtle_spawned_) {
        geometry_msgs::msg::TransformStamped t;

        try {
          rclcpp::Time now = this->get_clock()->now();
          rclcpp::Time when = now - rclcpp::Duration(5, 0); // 5 seconds ago

          if (!tf_buffer_->canTransform(
                toFrameRel, now,
                fromFrameRel, when,
                "world",
                50ms))
          {
    
            return;
          }

          // Safe to lookup transform now
          t = tf_buffer_->lookupTransform(
            toFrameRel, now,
            fromFrameRel, when,
            "world",
            50ms);

        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN(
            this->get_logger(),
            "Could not transform %s to %s from 5 seconds ago: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

        geometry_msgs::msg::Twist msg;

        double angle_to_target = atan2(
          t.transform.translation.y,
          t.transform.translation.x);
        double distance_to_target = sqrt(
          t.transform.translation.x * t.transform.translation.x +
          t.transform.translation.y * t.transform.translation.y);

        const double scaleRotationRate = 2.0;
        const double scaleForwardSpeed = 1.0;

        msg.angular.z = scaleRotationRate * angle_to_target;
        msg.linear.x = scaleForwardSpeed * distance_to_target;

        publisher_->publish(msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Turtle spawned successfully");
        turtle_spawned_ = true;
      }
    } else {
      if (spawner_->service_is_ready()) {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = spawn_x_;
        request->y = spawn_y_;
        request->theta = spawn_theta_;
        request->name = turtle_name_;

        using ServiceResponseFuture =
          rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (result->name == turtle_name_) {
              turtle_spawning_service_ready_ = true;
              RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", result->name.c_str());
            } else {
              RCLCPP_ERROR(this->get_logger(), "Spawn service result mismatch");
            }
          };
        spawner_->async_send_request(request, response_received_callback);
      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for spawn service...");
      }
    }
  }

  bool turtle_spawning_service_ready_;
  bool turtle_spawned_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  std::string turtle_name_;
  double spawn_x_;
  double spawn_y_;
  double spawn_theta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
