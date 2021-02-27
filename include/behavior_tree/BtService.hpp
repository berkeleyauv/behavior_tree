//
// Created by michael on 1/11/21.
//

#ifndef BEHAVIOR_TREE_BTSERVICE_H
#define BEHAVIOR_TREE_BTSERVICE_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

/**
 * Template specialization to converts a string to std::chrono timeout.
 */
namespace BT
{
  template<>
  inline std::chrono::milliseconds convertFromString(StringView str)
  {
    return static_cast<std::chrono::milliseconds>(std::stoi(str.data()));
  }
} // end namespace BT

/**
 * BtService class for running ROS2 services from within BTCPP behavior trees
 * All ROS2 services should inherit from this class and at minimum must implement the
 * populate_goal method
 */

template<class ServiceT>
class BtService : public BT::ActionNodeBase
{
protected:
  /// ROS2
  rclcpp::Node::SharedPtr _node;
  typename rclcpp::Client<ServiceT>::SharedPtr _client;

  /// Port inputs
  std::string _server_name{};
  std::chrono::milliseconds _server_timeout{};

public:
  BtService(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config)
  {
    _server_name = getInput<std::string>("server_name").value();
    _server_timeout = getInput<std::chrono::milliseconds>("server_timeout").value();

    config.blackboard->template get<rclcpp::Node::SharedPtr>("node", _node);

    _client = _node->create_client<ServiceT>(_server_name);

    RCLCPP_INFO_STREAM(
      _node->get_logger(), "Service client created for " << _server_name <<
        " with timeout " << _server_timeout.count() << "ms");
  }

  /**
   * Any actions that requires additional custom ports must create a new provided ports function
   * and call providedBasicPorts within it.
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  virtual typename ServiceT::Request populate_request() = 0;

  virtual BT::NodeStatus handle_response(typename ServiceT::Response response) = 0;

  BT::NodeStatus tick() override {
    while (!_client->wait_for_service(_server_timeout)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(_node->get_logger(), "Client interrupted while waiting for service to appear.");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(_node->get_logger(), "waiting for service to appear...");
    }
    auto request = populate_request();
    auto result_future = _client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(_node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(_node->get_logger(), "service call failed :(");
      return BT::NodeStatus::FAILURE;
    }
    return handle_response(result_future.get());
  }
}; // class BtService

#endif //BEHAVIOR_TREE_BTSERVICE_H
