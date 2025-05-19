#ifndef IS_ANYONE_SPEAKING_NODE_HPP_
#define IS_ANYONE_SPEAKING_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <mutex>
#include <string>

namespace hunav
{
  class IsAnyoneSpeakingNode : public BT::StatefulActionNode
  {
  public:
    IsAnyoneSpeakingNode(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config),
          agent_manager_(nullptr)
    {
    }

    IsAnyoneSpeakingNode() = delete;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int>("agent_id"),
          BT::InputPort<double>("time_step"),
          BT::InputPort<double>("distance_threshold", 5.0, "Max distance for speech to be considered"),
          BT::InputPort<double>("duration", 5.0, "Time (in seconds) to listen"),
          BT::OutputPort<int>("speaker_id")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    int agent_id_;
    double dt_;
    double distance_threshold_;
    double duration_;
    std::chrono::steady_clock::time_point start_time_;

    AgentManager *agent_manager_;

    // Store last message retrieved from AgentSayNode.
    std::string last_msg_;
    std::mutex msg_mutex_;
  };

} // namespace hunav

#endif // IS_ANYONE_SPEAKING_NODE_HPP_
