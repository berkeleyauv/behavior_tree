//
// Created by michael on 1/10/21.
//

#include "behavior_tree/BtEngine.hpp"

BtEngine::BtEngine(): Node("bt_engine") {
  configure_parameters();
  load_plugins();
  load_tree();
  if(run_groot_monitoring_){
    add_groot_monitoring();
  }
  run();
}

void BtEngine::configure_parameters() {
  bt_file_path_ = this->declare_parameter("bt_file_path", "tree.xml");
  loop_timeout_ = std::chrono::milliseconds(this->declare_parameter("loop_timeout", 100));
  plugins_ = this->declare_parameter("plugins", std::vector<std::string>());
  // Groot
  run_groot_monitoring_ = this->declare_parameter("run_groot_monitoring", true);
  publisher_port_ = this->declare_parameter("publisher_port", 1666);
  server_port_ = this->declare_parameter("server_port", 1667);
  max_msg_per_second_ = this->declare_parameter("max_msg_per_second", 25);
}

void BtEngine::load_tree() {
  // Make the ROS node instance accessible to nodes in the tree
  auto blackboard  = Blackboard::create();
  auto node = this->create_sub_node("bt");
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  tree_ = std::make_shared<Tree>(factory_.createTreeFromFile(bt_file_path_, blackboard));
}

void BtEngine::run() {
  rclcpp::WallRate loop_rate(loop_timeout_);
  while(rclcpp::ok()){
    tree_->tickRoot();
    loop_rate.sleep();
  }
}

void BtEngine::add_groot_monitoring() {
  groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
    *tree_, max_msg_per_second_, publisher_port_, server_port_);
}

void BtEngine::load_plugins() {
  for (const auto & p : plugins_) {
    factory_.registerFromPlugin(SharedLibrary::getOSName(p));
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BtEngine>());
  rclcpp::shutdown();
  return 0;
}