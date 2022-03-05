//
// Created by michael on 1/11/21.
//

#ifndef BEHAVIOR_TREE_BTACTION_H
#define BEHAVIOR_TREE_BTACTION_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
 * BtAction class for running ROS2 actions from within BTCPP behavior trees
 * All ROS2 actions should inherit from this class and at minimum must implement the
 * populate_request method
 */

template<class ActionT>
class BtAction : public BT::ActionNodeBase
{
protected:
  /// ROS2
  rclcpp::Node::SharedPtr _node;
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr _goal_handle;
  typename rclcpp_action::Client<ActionT>::SharedPtr _action_client;

  /// State variables
  bool _goal_done{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult _result{};

  /// Port inputs
  std::string _server_name{};
  std::chrono::milliseconds _server_timeout{}, _cancel_timeout{};

  void wait_for_goal_to_cancel()
  {
    while (rclcpp::ok()) {
      rclcpp::spin_some(_node);
      if (_goal_done) {
        if (_result.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_INFO(_node->get_logger(), "%s", "Goal canceled successfully");
        }
        return;
      }
    }
  }

  BT::NodeStatus send_goal()
  {
    /// Reset any state if needed
    _goal_done = false;

    /// Make sure the action server is available
    if (!_action_client->wait_for_action_server(_server_timeout)) {
      RCLCPP_ERROR(_node->get_logger(), "%s", "Action server not available after waiting");
      return BT::NodeStatus::FAILURE;
    }

    /// Create goal message and send to actions server
    auto goal = populate_goal();
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(
      &BtAction<ActionT>::feedback_callback, this, _1, _2);

    send_goal_options.result_callback = std::bind(&BtAction<ActionT>::result_callback, this, _1);
    RCLCPP_INFO(_node->get_logger(), "%s %s", "Sending goal to ", _server_name.c_str());

    auto goal_handle_future = _action_client->async_send_goal(goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(_node, goal_handle_future, _server_timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(_node->get_logger(), "Send goal failed");
      return BT::NodeStatus::FAILURE;
    }
    _goal_handle = goal_handle_future.get();
    if (!_goal_handle) {
      RCLCPP_ERROR(_node->get_logger(), "Goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus process_goal_status()
  {
    // Process any flags that might be set after spinning the node
    if (_goal_done) {
      // handle the result after the actions has completed
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
    return BT::NodeStatus::RUNNING;
  }

  /**
   * Checks to see if the goal handle is currently attached to a goal that is in process or in a transitions.
   * A new goal should not be sent if the goal_handle is currently attending to a goal in progress.
   */
  bool goal_in_progress()
  {
    rclcpp::spin_some(_node);
    return _goal_handle != nullptr &&
           (_goal_handle->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
           _goal_handle->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           _goal_handle->get_status() == action_msgs::msg::GoalStatus::STATUS_CANCELING);
  }

public:
  BtAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config)
  {
    _server_name = getInput<std::string>("server_name").value();
    _server_timeout = getInput<std::chrono::milliseconds>("server_timeout").value();
    _cancel_timeout = getInput<std::chrono::milliseconds>("cancel_timeout").value();

    config.blackboard->template get<rclcpp::Node::SharedPtr>("node", _node);

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
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
      BT::InputPort<std::chrono::milliseconds>("cancel_timeout")
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
    RCLCPP_ERROR(_node->get_logger(), "Halting");
    if (goal_in_progress()) {
      auto cancel_result_future = _action_client->async_cancel_goal(_goal_handle);
      if (rclcpp::spin_until_future_complete(_node, cancel_result_future, _cancel_timeout) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(_node->get_logger(), "Failed to cancel goal");
      }
      RCLCPP_INFO(_node->get_logger(), "Goal is being canceled");
      wait_for_goal_to_cancel();
    }
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
   *    2. Send goal fails or the goal is not accepted by the server
   *    3. The goal is aborted
   *    4. The goal is canceled
   *    5. An unknown result code is received
   */
  BT::NodeStatus tick() override
  {
    if (this->status() == BT::NodeStatus::IDLE && !goal_in_progress()) {
      /// Send a new goal
      return send_goal();
    } else {
      /// Attend to an existing goal
      rclcpp::spin_some(_node);
      return process_goal_status();
    }
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
    // Silence unused variable warning
    (void)feedback;
    RCLCPP_DEBUG(_node->get_logger(), "Feedback unhandled");
  }

}; // class BtAction

#endif //BEHAVIOR_TREE_BTACTION_H
