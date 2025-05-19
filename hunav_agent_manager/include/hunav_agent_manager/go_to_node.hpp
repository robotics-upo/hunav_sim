#ifndef HUNAV_GOTO_NODE_HPP_
#define HUNAV_GOTO_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include <chrono>

namespace hunav
{

    class GoToNode : public BT::StatefulActionNode
    {
    public:
        GoToNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config),
              agent_manager_(nullptr)
        {
        }

        GoToNode() = delete;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("agent_id"),
                BT::InputPort<double>("target_x"),
                BT::InputPort<double>("target_y"),
                BT::InputPort<double>("time_step"),
                BT::InputPort<bool>("temporary", false, "Set true for a temporary goal"),
                BT::InputPort<double>("stop_duration", 5.0, "Duration to freeze the agent for a temporary goal")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int agent_id_;
        double target_x_;
        double target_y_;
        double dt_;
        bool temporary_;
        double stop_duration_;
        std::chrono::steady_clock::time_point freeze_start_time_;
        AgentManager *agent_manager_;
        std::list<sfm::Goal> original_goals_;
    };

} // namespace hunav

#endif // HUNAV_GOTO_NODE_HPP_
