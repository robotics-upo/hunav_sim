#include "hunav_agent_manager/is_looking_at_me_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <sstream>
#include <iostream>
#include <cmath>

namespace hunav
{
    BT::NodeStatus IsLookingAtMeNode::onStart()
    {
        if (!getInput<int>("agent_id", agent_id_))
            throw BT::RuntimeError("IsLookingAtMeNode: missing input [agent_id]");
        if (!getInput<double>("time_step", dt_))
            throw BT::RuntimeError("IsLookingAtMeNode: missing input [time_step]");
        if (!getInput<int>("target_id", target_id_))
            throw BT::RuntimeError("IsLookingAtMeNode: missing input [target_id]");
        if (!getInput<double>("distance_threshold", distance_threshold_))
            distance_threshold_ = 5.0;
        if (!getInput<double>("angle_threshold", angle_threshold_))
            angle_threshold_ = 0.3;
        if (!getInput<double>("duration", duration_))
            duration_ = 5.0;

        // Retrieve AgentManager pointer
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("IsLookingAtMeNode: global AgentManager pointer not set");
        }

        start_time_ = std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus IsLookingAtMeNode::onRunning()
    {
        // Get positions.
        auto myPos = agent_manager_->getAgentPosition(agent_id_);
        auto targetPos = agent_manager_->getAgentPosition(target_id_);
        double dx = myPos.getX() - targetPos.getX();
        double dy = myPos.getY() - targetPos.getY();
        double dist = std::hypot(dx, dy);

        if (dist > distance_threshold_)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
            if (elapsed >= duration_)
                return BT::NodeStatus::FAILURE;
            return BT::NodeStatus::RUNNING;
        }

        // Compute the angle from the target to the listener.
        double desired_angle = std::atan2(myPos.getY() - targetPos.getY(), myPos.getX() - targetPos.getX());
        double target_yaw = agent_manager_->getAgentYaw(target_id_);
        double angle_diff = std::remainder(target_yaw - desired_angle, 2 * M_PI);

        if (std::fabs(angle_diff) <= angle_threshold_)
        {
            return BT::NodeStatus::SUCCESS;
        }

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
        if (elapsed >= duration_)
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::RUNNING;
    }

    void IsLookingAtMeNode::onHalted()
    {
    }

} // namespace hunav
