#ifndef FOLLOW_AGENT_NODE_HPP_
#define FOLLOW_AGENT_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include <chrono>

namespace hunav
{

  class FollowAgentNode : public BT::StatefulActionNode
  {
  public:
    FollowAgentNode(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config),
          duration_(0.0),
          agent_manager_(nullptr)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int>("agent_id"),
          BT::InputPort<int>("target_agent_id"),
          BT::InputPort<double>("time_step"),
          BT::InputPort<double>("closest_dist"),
          BT::InputPort<double>("max_vel"),
          BT::InputPort<double>("duration")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    int agent_id_;
    int target_agent_id_;
    double dt_;
    double closest_dist_;
    double max_vel_;
    double duration_; // if > 0, run for that many seconds; if 0, run indefinitely
    std::chrono::steady_clock::time_point start_time_;
    AgentManager *agent_manager_;

    // Default parameters - kept for backward compatibility
    const double default_closest_dist_ = 1.5;
    const double default_max_vel_ = 1.5;
  };

} // namespace hunav

#endif // FOLLOW_AGENT_NODE_HPP_
