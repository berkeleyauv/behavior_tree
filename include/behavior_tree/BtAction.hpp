//
// Created by michael on 1/11/21.
//

#ifndef BEHAVIOR_TREE_BTACTION_H
#define BEHAVIOR_TREE_BTACTION_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "example_interfaces/action/fibonacci.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

/**
 * Template specialization to converts a string to Position2D.
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
 * BtAction class for running ROS2 actions from within BTCPP behavior trees
 * All ROS2 actions should inherit from this class and at minimum must implement the
 * populate_goal method
 */

template<class ActionT>
class BtAction : public BT::AsyncActionNode
{
protected:
  /// ROS2
  rclcpp::Node::SharedPtr _node;
  typename rclcpp_action::Client<ActionT>::SharedPtr _action_client;

  /// State variables
  std::atomic<bool> _halt_requested{false};
  bool _goal_done{false}, _goal_rejected{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult _result{};

  /// Port inputs
  std::string _server_name{};
  std::chrono::milliseconds _server_timeout{};

public:
  BtAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::AsyncActionNode(name, config)
  {
    std::string node_namespace;

    _server_name = getInput<std::string>("server_name").value();
    _server_timeout = getInput<std::chrono::milliseconds>("server_timeout").value();

    /* Create a new node with the specified namespace, since this node might be copied elsewhere in the tree
     * we force the name to be unique using the current system time
     */
    config.blackboard->template get<std::string>("node_namespace", node_namespace);
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count(); // make sure name is unique
    _node = rclcpp::Node::make_shared(name + "_" + std::to_string(stamp), node_namespace);

    _action_client = rclcpp_action::create_client<ActionT>(_node, _server_name);

    RCLCPP_INFO_STREAM(
      _node->get_logger(), "Action client created for " << _server_name <<
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
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /**
   * This overloaded method is used to stop the execution of this node.
   */
  void halt() override
  {
    _halt_requested.store(true);
  }

  /**
   * This method should populate the goal message based on the state of the system. Please implement
   * this in the subclass.
   */
  virtual typename ActionT::Goal populate_goal() = 0;

  /**
   * This is the main method that dictates the flow of the action once this node is called. The node return success
   * if and only if (1) the action completes successfully or (2) halt is called and the actions
   * successfully cancels. The reasons for this node to return failure are as follows:
   *    1. The server cannot be found
   *    2. The goal is not accepted by the server
   *    3. The goal is aborted
   *    4. The goal is canceled
   *    5. An unknown result code is received
   *    6. rclcpp::ok() returns false and the while loop ends before returning success
   */
  BT::NodeStatus tick() override
  {
    /// Reset any state if needed
    _halt_requested.store(false);
    _goal_done = false;
    _goal_rejected = false;

    /// Make sure the action server is available
    if (!_action_client->wait_for_action_server(_server_timeout)) {
      RCLCPP_ERROR(_node->get_logger(), "Action server not available after waiting");
      return BT::NodeStatus::FAILURE;
    }

    /// Create goal message and send to actions server
    auto goal = populate_goal();
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
      &BtAction<ActionT>::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(
      &BtAction<ActionT>::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&BtAction<ActionT>::result_callback, this, _1);
    auto goal_handle_future = _action_client->async_send_goal(goal, send_goal_options);

    /// Set the BT node status to running
    setStatus(BT::NodeStatus::RUNNING);

    /// Enter the operational loop
    while (rclcpp::ok()) {
      // Run the callbacks
      rclcpp::spin_some(_node);
      // Process any flags that might be set after spinning the node
      if (_halt_requested) {
        auto cancel_result_future = _action_client->async_cancel_goal(goal_handle_future.get());
        if (rclcpp::spin_until_future_complete(
            _node,
            cancel_result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(_node->get_logger(), "Failed to cancel goal");
          return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(_node->get_logger(), "Goal is being canceled");
        _halt_requested.store(false); // We dont want to end up back here on the next loop
      }
      // If the goal was finished or rejected
      if (_goal_done || _goal_rejected) {
        // Rejected goal is an instant failure
        if (_goal_rejected) {
          return BT::NodeStatus::FAILURE;
        } else { // handle the result after the actions has completed
          switch (_result.code) {
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
      }
    }
    /// If something goes wrong with the while loop we should return a failure
    return BT::NodeStatus::FAILURE;
  }

  void result_callback(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
  {
    _goal_done = true;
    _result = result;
  }

  void feedback_callback(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
    const std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    RCLCPP_DEBUG(_node->get_logger(), "Feedback unhandled");
  }


  void goal_response_callback(std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      _goal_rejected = true;
      RCLCPP_ERROR(_node->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(_node->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

}; // class BtAction

#endif //BEHAVIOR_TREE_BTACTION_H
