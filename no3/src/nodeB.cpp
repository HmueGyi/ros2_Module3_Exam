#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

using Add = example_interfaces::srv::AddTwoInts;

class NodeB : public rclcpp::Node
{
public:
  NodeB(int64_t a = 0, int64_t b = 0, bool call_client = false)
  : Node("node_b")
  {
    // Provide service
    service_ = this->create_service<Add>(
      "add_two_ints_b",
      [this](const std::shared_ptr<Add::Request> request,
             std::shared_ptr<Add::Response> response) {
        response->sum = request->a + request->b;  // ✅ Addition only
        RCLCPP_INFO(this->get_logger(), "Service B: %ld + %ld = %ld", request->a, request->b, response->sum);
      });

    // Connect client to nodeA's service
    client_ = this->create_client<Add>("add_two_ints");

    if (call_client)
    {
      call_other_service(a, b);
    }
  }

  void call_other_service(int64_t a, int64_t b)
  {
    // Immediate warning if service unavailable
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service node_A not available.");
    }

    // Wait until the service is available, logging info every second
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting node_A to become available...");
    }

    // Service is available, send request
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
      RCLCPP_ERROR(this->get_logger(), "Failed to call add_two_ints");
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

  auto node = std::make_shared<NodeB>(a, b, call_client);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
