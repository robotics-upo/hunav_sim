#ifndef HUNAV_LOOK_AT_POINT_NODE_HPP_
#define HUNAV_LOOK_AT_POINT_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <chrono>

namespace hunav
{

    class LookAtPointNode : public BT::StatefulActionNode
    {
    public:
        LookAtPointNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config),
              agent_manager_(nullptr)
        {
        }

        LookAtPointNode() = delete;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("agent_id"),
                BT::InputPort<int>("goal_id", "Goal ID from agent's ROS goals to look at"),
                BT::InputPort<double>("yaw_tolerance", 0.01, "Angle tolerance [rad] to consider 'aligned'")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int agent_id_;
        double target_x_;
        double target_y_;
        double yaw_tolerance_;
        AgentManager *agent_manager_;

        // Store the “absolute” desired yaw so we can compare each tick
        double desired_yaw_rad_;
    };

} // namespace hunav

#endif // HUNAV_LOOK_AT_POINT_NODE_HPP_
