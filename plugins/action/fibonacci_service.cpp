#include "std_srvs/srv/trigger.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"


using namespace BT;
using FibonacciService = std_srvs::srv::Trigger;

class Fibonacci : public BtService<FibonacciService>
{
public:
  Fibonacci(const std::string & name, const BT::NodeConfiguration & config)
  : BtService<FibonacciService>(name, config) {}

  FibonacciService::Request::SharedPtr populate_request() override
  {
    FibonacciService::Request::SharedPtr request;
    return request;
  }

  BT::NodeStatus handle_response(FibonacciService::Response::SharedPtr response) override
  {
    RCLCPP_INFO(_node->get_logger(), "%s %s",  "Service call complete: ", response->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<Fibonacci>("FibonacciService");
}
