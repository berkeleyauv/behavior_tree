#include "example_interfaces/action/fibonacci.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtAction.hpp"


using namespace BT;
using FibonacciAction = example_interfaces::action::Fibonacci;

class Fibonacci: public  BtAction<FibonacciAction> {
public:
  Fibonacci(const std::string& name, const BT::NodeConfiguration& config):
    BtAction<FibonacciAction>(name, config){}

//  FibonacciAction::Goal populate_goal() override {
//    FibonacciAction::Goal goal;
//    goal.order = 5;
//    return goal;
//  }
};

BT_REGISTER_NODES(factory){
  factory.registerNodeType<Fibonacci>("Fibonacci");
}