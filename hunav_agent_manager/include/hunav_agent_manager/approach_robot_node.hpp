#ifndef APPROACH_ROBOT_NODE_HPP_
#define APPROACH_ROBOT_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include <chrono>

namespace hunav
{

  class ApproachRobotNode : public BT::StatefulActionNode
  {
  public:
    ApproachRobotNode(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config),
          duration_(0.0),
          agent_manager_(nullptr)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int>("agent_id"),
          BT::InputPort<double>("time_step"),
          BT::InputPort<double>("closest_dist"),
          BT::InputPort<double>("max_vel"),
          BT::InputPort<double>("duration", 0.0, "Duration (in seconds) for approaching the robot (0 = run forever)")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    int agent_id_;
    double dt_;
    double closest_dist_;
    double max_vel_;
    double duration_; // if > 0, run for that many seconds; if 0, run indefinitely.
    std::chrono::steady_clock::time_point start_time_;
    AgentManager *agent_manager_;
  };

} // namespace hunav

#endif // APPROACH_ROBOT_NODE_HPP_
