#ifndef HUNAV_CONVERSATION_FORMATION_NODE_HPP_
#define HUNAV_CONVERSATION_FORMATION_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <unordered_map>
#include <list> // for std::list<sfm::Goal>

namespace hunav
{

  class ConversationFormationNode : public BT::StatefulActionNode
  {
  public:
    ConversationFormationNode(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config),
          agent_manager_(nullptr)
    {
    }

    ConversationFormationNode() = delete;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int>("main_agent_id"),
          BT::InputPort<int>("goal_id", "Global goal ID to use as conversation center"),
          BT::InputPort<double>("conversation_duration", 10.0, "Duration of the conversation (seconds)"),
          BT::InputPort<double>("time_step", 0.1, "Time step (seconds) for navigation updates"),
          BT::InputPort<BT::AnyTypeAllowed>("non_main_agent_ids", "Comma-separated IDs of non-main agents or single ID")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    int main_agent_id_;
    double conversation_duration_;

    // Conversation center (set in onStart())
    utils::Vector2d conversation_center_;
    int center_goal_id_;

    // Time step (read each tick in onRunning())
    double dt_;

    // When all agents first become “ready,” we start this timer
    std::chrono::steady_clock::time_point conversation_start_time_;

    AgentManager *agent_manager_;

    // IDs of participants (main + non-main)
    std::vector<int> non_main_ids_;
    std::vector<int> all_ids_;

    // To restore each agent’s original goals after the conversation
    std::unordered_map<int, std::list<sfm::Goal>> original_goals_;

    // For each agent (in the same order as all_ids_), track if it's “ready” (at-spot + oriented)
    std::vector<bool> readiness_;
  };

} // namespace hunav

#endif // HUNAV_CONVERSATION_FORMATION_NODE_HPP_
