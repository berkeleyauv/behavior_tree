//
// Created by michael on 1/10/21.
//

#ifndef BEHAVIOR_TREE_BT_ENGINE_H
#define BEHAVIOR_TREE_BT_ENGINE_H

#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using namespace BT;

class BtEngine: public rclcpp::Node {
  BehaviorTreeFactory factory_;
  std::shared_ptr<Tree> tree_;
  std::unique_ptr<BT::PublisherZMQ> groot_monitor_;

  /// ROS Parameters
  std::string bt_file_path_;
  std::chrono::milliseconds loop_timeout_{};
  std::vector<std::string> plugins_;
  // Groot
  bool run_groot_monitoring_{};
  uint16_t publisher_port_{}, server_port_{}, max_msg_per_second_{};

  void configure_parameters();
  void load_tree();
  void run();
  void add_groot_monitoring();
  void load_plugins();


public:
  BtEngine();
};


#endif //BEHAVIOR_TREE_BT_ENGINE_H
