#ifndef BLOCK_AGENT_NODE_HPP_
#define BLOCK_AGENT_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include <chrono>

namespace hunav
{

    class BlockAgentNode : public BT::StatefulActionNode
    {
    public:
        BlockAgentNode(const std::string &name, const BT::NodeConfig &config)
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
                BT::InputPort<double>("front_dist"),
                BT::InputPort<double>("duration", 0.0, "Duration (in seconds) for blocking the target agent; 0 means permanent")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int agent_id_;
        int target_agent_id_;
        double dt_;
        double front_dist_;
        double duration_; // if > 0, run for that many seconds; if 0, run indefinitely.
        std::chrono::steady_clock::time_point start_time_;
        AgentManager *agent_manager_;
    };

} // namespace hunav

#endif // BLOCK_AGENT_NODE_HPP_
