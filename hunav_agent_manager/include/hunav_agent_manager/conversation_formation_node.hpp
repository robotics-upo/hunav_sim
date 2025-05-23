#ifndef HUNAV_CONVERSATION_FORMATION_NODE_HPP_
#define HUNAV_CONVERSATION_FORMATION_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <unordered_map>

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
          BT::InputPort<double>("conversation_duration", 10.0, "Duration of the conversation (seconds)"),
          BT::InputPort<double>("center_x", 0.0, "X coordinate of the conversation center"),
          BT::InputPort<double>("center_y", 0.0, "Y coordinate of the conversation center"),
          BT::InputPort<double>("time_step", 0.1, "Time step (seconds) for navigation updates"),
          BT::InputPort<std::string>("non_main_agent_ids", "1,3", "Comma-separated IDs of non-main agents")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    int main_agent_id_;
    double conversation_duration_;
    // Conversation center provided by inputs
    utils::Vector2d conversation_center_;
    double center_x_, center_y_;
    // Time step will be read on each tick
    double dt_;
    std::chrono::steady_clock::time_point conversation_start_time_;
    AgentManager *agent_manager_;

    // For the input version, store the non-main agent IDs
    std::vector<int> non_main_ids_;
    // List of all agent IDs
    std::vector<int> all_ids_;
    // Store original goals for non-main agents, so they can be restored
    std::unordered_map<int, std::list<sfm::Goal>> original_goals_;
    // Vector for storing (agent_id, current_angle) pairs
    std::vector<std::pair<int, double>> agent_angles_;
  };

} // namespace hunav

#endif // HUNAV_CONVERSATION_FORMATION_NODE_HPP_
