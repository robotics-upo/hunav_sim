#ifndef IS_ANYONE_LOOKING_AT_ME_NODE_HPP_
#define IS_ANYONE_LOOKING_AT_ME_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include <chrono>
#include <mutex>

namespace hunav
{

    class IsAnyoneLookingAtMeNode : public BT::StatefulActionNode
    {
    public:
        IsAnyoneLookingAtMeNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config),
              duration_(0.0),
              agent_manager_(nullptr)
        {
        }

        IsAnyoneLookingAtMeNode() = delete;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("agent_id"),
                BT::InputPort<double>("time_step"),
                BT::InputPort<double>("distance_threshold", 5.0, "Maximum distance for being considered looked at"),
                BT::InputPort<double>("angle_threshold", 0.2, "Angular tolerance in radians"),
                BT::InputPort<double>("duration", 5.0, "Time (in seconds) to check for an observer"),
                BT::OutputPort<int>("observer_id")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int agent_id_;
        double dt_;
        double distance_threshold_;
        double angle_threshold_;
        double duration_;
        std::chrono::steady_clock::time_point start_time_;

        AgentManager *agent_manager_;
    };

} // namespace hunav

#endif // IS_ANYONE_LOOKING_AT_ME_NODE_HPP_
