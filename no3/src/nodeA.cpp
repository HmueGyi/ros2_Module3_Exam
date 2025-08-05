#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

using Add = example_interfaces::srv::AddTwoInts;

class NodeA : public rclcpp::Node
{
public:
  NodeA(int64_t a = 0, int64_t b = 0, bool call_client = false)
  : Node("node_a")
  {
    // Provide service
    service_ = this->create_service<Add>(
      "add_two_ints",
      [this](const std::shared_ptr<Add::Request> request,
             std::shared_ptr<Add::Response> response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Service A: %ld + %ld = %ld", request->a, request->b, response->sum);
      });

    // Connect client to nodeB's service
    client_ = this->create_client<Add>("add_two_ints_b");

    if (call_client)
    {
      call_other_service(a, b);
    }
  }

  void call_other_service(int64_t a, int64_t b)
  {
    // Print warning immediately if not available
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service node_B not available.");
    }

    // Wait until service becomes available, logging info every second
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for service node_B to become available...");
    }

    // Service is now available, send request
    auto request = std::make_shared<Add::Request>();
    request->a = a;
    request->b = b;

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Received result: %ld", result.get()->sum);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call add_two_ints_b");
    }
  }

private:
  rclcpp::Service<Add>::SharedPtr service_;
  rclcpp::Client<Add>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  bool call_client = (argc == 3);
  int64_t a = 0, b = 0;

  if (call_client)
  {
    a = std::stoll(argv[1]);
    b = std::stoll(argv[2]);
  }

  auto node = std::make_shared<NodeA>(a, b, call_client);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
