#ifndef GROUP_WALK_NODE_HPP_
#define GROUP_WALK_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <unordered_map>

namespace hunav
{

  class GroupWalkNode : public BT::StatefulActionNode
  {
  public:
    GroupWalkNode(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config),
          duration_(0.0),
          agent_manager_(nullptr)
    {
    }

    GroupWalkNode() = delete;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int>("main_agent_id"),
          BT::InputPort<double>("time_step"),
          BT::InputPort<double>("duration", 0.0, "Duration (in seconds) for group walk; 0 means indefinite"),
          BT::InputPort<BT::AnyTypeAllowed>("non_main_agent_ids", "Comma-separated list of non-main agent IDs")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    int main_agent_id_;
    double dt_;
    double duration_; // Duration (in seconds) for which the grouping remains active
    std::chrono::steady_clock::time_point start_time_;
    AgentManager *agent_manager_;

    // non-main agent IDs parsed from input
    std::vector<int> non_main_ids_;
    // Combined list: main agent + non-main agents
    std::vector<int> all_ids_;
    // Save original goals for each non-main agent
    std::unordered_map<int, std::list<sfm::Goal>> original_goals_;
  };

} // namespace hunav

#endif // GROUP_WALK_NODE_HPP_
