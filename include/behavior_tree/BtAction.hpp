//
// Created by michael on 1/11/21.
//

#ifndef BEHAVIOR_TREE_BTACTION_H
#define BEHAVIOR_TREE_BTACTION_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "example_interfaces/action/fibonacci.hpp"

using namespace std::chrono_literals;

// Template specialization to converts a string to Position2D.
namespace BT
{
  template <> inline std::chrono::milliseconds convertFromString(StringView str)
  {
    return static_cast<std::chrono::milliseconds>(std::stoi(str.data()));
  }
} // end namespace BT

template<class ActionT>
class BtAction: public BT::AsyncActionNode {
protected:
  rclcpp::Node::SharedPtr _node;
  std::atomic<bool> _halt_requested{false};
  typename rclcpp_action::Client<ActionT>::SharedPtr _action_client;

  std::string _server_name{};
  std::chrono::milliseconds _server_timeout{};

public:
  BtAction(const std::string& name, const BT::NodeConfiguration& config):
           BT::AsyncActionNode(name, config)
  {
//    config.blackboard->template get<rclcpp::Node::SharedPtr>("node", _node);

    _server_name = getInput<std::string>("server_name").value();
    _server_timeout = getInput<std::chrono::milliseconds>("server_timeout").value();

    _node = rclcpp::Node::make_shared("bt_node_min");
    _action_client = rclcpp_action::create_client<ActionT>(_node, _server_name);
  }

  // This overloaded method is used to stop the execution of this node.
  void halt() override
  {
    _halt_requested.store(true);
  }

  // Any actions that requires ports must create a new
  // provided ports function and call providedBasicPorts in it.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

//  virtual typename ActionT::Goal populate_goal();

  BT::NodeStatus tick() override {
    typename ActionT::Goal goal;
    goal.order = 5;

    // Send goal to the server
    if (!_action_client->wait_for_action_server(_server_timeout)) {
      RCLCPP_ERROR(_node->get_logger(), "Action server not available after waiting");
      return BT::NodeStatus::FAILURE;
    }
    auto goal_handle_future = _action_client->async_send_goal(goal);

    while(goal_handle_future.wait_for(0ms) != std::future_status::ready){
      if(_halt_requested){return BT::NodeStatus::FAILURE;}
      rclcpp::spin_some(_node);
    }

    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(_node->get_logger(), "Goal was rejected by server");
      return BT::NodeStatus::FAILURE;
    }

    // Wait for the server to complete the goal
    std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult>
      result_future = _action_client->async_get_result(goal_handle);

    while(result_future.wait_for(0ms) != std::future_status::ready){
      if(_halt_requested){return BT::NodeStatus::FAILURE;}
      rclcpp::spin_some(_node);
    }

    switch (result_future.get().code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return BT::NodeStatus::SUCCESS;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(_node->get_logger(), "Goal was aborted");
        return BT::NodeStatus::FAILURE;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(_node->get_logger(), "Goal was canceled");
        return BT::NodeStatus::FAILURE;
      default:
        RCLCPP_ERROR(_node->get_logger(), "Unknown result code");
        return BT::NodeStatus::FAILURE;
    }
  }
};

#endif //BEHAVIOR_TREE_BTACTION_H
