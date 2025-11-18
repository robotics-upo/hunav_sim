#include "hunav_agent_manager/approach_robot_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <cmath>
#include <iostream>

namespace hunav
{

    BT::NodeStatus ApproachRobotNode::onStart()
    {
        // Retrieve required inputs.
        if (!getInput<int>("agent_id", agent_id_))
            throw BT::RuntimeError("ApproachRobotNode: missing input [agent_id]");
        if (!getInput<double>("time_step", dt_))
            throw BT::RuntimeError("ApproachRobotNode: missing input [time_step]");
        if (!getInput<double>("closest_dist", closest_dist_))
            throw BT::RuntimeError("ApproachRobotNode: missing input [closest_dist]");
        if (!getInput<double>("max_vel", max_vel_))
            throw BT::RuntimeError("ApproachRobotNode: missing input [max_vel]");
        if (!getInput<double>("duration", duration_))
            duration_ = 0.0; // Default to permanent (run forever)
        // Retrieve AgentManager
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("ApproachRobotNode: global AgentManager pointer not set");
        }
        // If a duration is provided (>0), record the start time
        if (duration_ > 0.0)
            start_time_ = std::chrono::steady_clock::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ApproachRobotNode::onRunning()
    {

        double current_dt;
        auto dt_msg = getInput<double>("time_step");
        if (!dt_msg)
            throw BT::RuntimeError("ApproachRobotNode: missing input [time_step] in onRunning", dt_msg.error());
        current_dt = dt_msg.value();

        agent_manager_->approximateRobot(agent_id_, current_dt, closest_dist_, max_vel_);

        if (duration_ > 0.0)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
            if (elapsed >= duration_)
            {
                // std::cout << "[ApproachRobotNode] Duration (" << duration_
                //           << " s) elapsed for agent " << agent_id_ << ". Returning SUCCESS.\n";
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void ApproachRobotNode::onHalted()
    {
    }

} // namespace hunav
