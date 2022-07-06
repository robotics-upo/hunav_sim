
#ifndef AGENT_MANAGER__TIME_EXPIRED_CONDITION_HPP_
#define AGENT_MANAGER__TIME_EXPIRED_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace hunav {

/**
 * @brief A BT::ConditionNode that returns SUCCESS every time a specified
 * time period passes and FAILURE otherwise
 */
class TimeExpiredCondition : public BT::ConditionNode {
public:
  /**
   * @brief A constructor for nav2_behavior_tree::TimeExpiredCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TimeExpiredCondition(const std::string &condition_name,
                       const BT::NodeConfiguration &conf);

  TimeExpiredCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("seconds", 1.0, "Seconds"),
            BT::InputPort<double>("ts", 0.01, "time step to be added"),
            BT::InputPort<bool>("only_once", false,
                                "if true, the timer is not reset at the end, "
                                "so it is executed only once")};
  }

private:
  // rclcpp::Node::SharedPtr node_;
  // rclcpp::Time start_;
  double dt_;
  double period_;
  double accum_;
  bool only_once_;
  bool print_;
};

} // namespace hunav

#endif // BEHAVIOR_MANAGER__TIME_EXPIRED_CONDITION_HPP_
