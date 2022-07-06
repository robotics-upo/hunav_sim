#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

#include "hunav_agent_manager/time_expired_condition.hpp"

namespace hunav {

TimeExpiredCondition::TimeExpiredCondition(const std::string &condition_name,
                                           const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf), period_(1.0) {
  getInput("seconds", period_);
  // node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // start_ = node_->now();
  getInput("ts", dt_);
  getInput("only_once", only_once_);
  accum_ = 0.0;
  print_ = true;
  printf(
      "TimeExpiredCondition initiated!\n seconds:%.2f, dt:%.3f, only_once:%i\n",
      period_, dt_, only_once_);
  //   std::cout << std::endl
  //             << "TimeExpiredCondition initiated!" << std::endl
  //             << " seconds:" << period_ << " dt:" << dt_
  //             << " only_once:" << only_once_ << std::endl;
}

BT::NodeStatus TimeExpiredCondition::tick() {
  //   if (status() == BT::NodeStatus::IDLE) {
  //     // start_ = node_->now();
  //     accum_ = 0.0;
  //     return BT::NodeStatus::FAILURE;
  //   }
  getInput("seconds", period_);
  getInput("only_once", only_once_);
  getInput("ts", dt_);

  // Determine how long its been since we've started this iteration
  // auto elapsed = node_->now() - start_;
  accum_ += dt_;

  // Now, get that in seconds
  // auto seconds = elapsed.seconds();

  // if (seconds < period_) {
  if (accum_ < period_) {
    // std::cout << std::endl
    //           << "TimeExpiredCondition dt: " << dt_ << " accum: " << accum_
    //           << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  if (only_once_ && print_) {
    std::cout << std::endl
              << "Only-once Timer of " << period_ << " seconds has expired!!"
              << std::endl;
    print_ = false;
  }
  if (!only_once_) {
    accum_ = 0.0;
    std::cout << std::endl
              << "Timer of " << period_ << " seconds has expired!!"
              << std::endl;
  }
  // start_ = node_->now(); // Reset the timer
  return BT::NodeStatus::SUCCESS;
}

} // namespace hunav

// #include "behaviortree_cpp_v3/bt_factory.h"
// BT_REGISTER_NODES(factory) {
//   factory.registerNodeType<hunav_agent_manager::TimeExpiredCondition>(
//       "TimeExpired");
// }
