#include "hunav_agent_manager/avoid_robot_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <cmath>
#include <iostream>

namespace hunav
{

    BT::NodeStatus AvoidRobotNode::onStart()
    {
        if (!getInput<int>("agent_id", agent_id_))
            throw BT::RuntimeError("AvoidRobotNode: missing input [agent_id]");
        if (!getInput<double>("time_step", dt_))
            throw BT::RuntimeError("AvoidRobotNode: missing input [time_step]");
        if (!getInput<double>("safe_distance", safe_distance_))
            throw BT::RuntimeError("AvoidRobotNode: missing input [safe_distance]");
        if (!getInput<double>("runaway_vel", runaway_vel_))
            throw BT::RuntimeError("AvoidRobotNode: missing input [runaway_vel]");
        if (!getInput<double>("duration", duration_))
            duration_ = 0.0; // Default: permanent

        // Retrieve AgentManager
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("AvoidRobotNode: global AgentManager pointer not set");
        }

        if (duration_ > 0.0)
            start_time_ = std::chrono::steady_clock::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus AvoidRobotNode::onRunning()
    {
        double dt;
        auto dt_msg = getInput<double>("time_step");
        if (!dt_msg)
            throw BT::RuntimeError("AvoidRobotNode: missing input [time_step] during onRunning", dt_msg.error());
        dt = dt_msg.value();

        agent_manager_->avoidRobot(agent_id_, dt, safe_distance_, runaway_vel_);

        if (duration_ > 0.0)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
            if (elapsed >= duration_)
                return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void AvoidRobotNode::onHalted()
    {
    }

} // namespace hunav
