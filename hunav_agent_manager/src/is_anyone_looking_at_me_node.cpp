#include "hunav_agent_manager/is_anyone_looking_at_me_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <sstream>
#include <iostream>
#include <cmath>

namespace hunav
{
    BT::NodeStatus IsAnyoneLookingAtMeNode::onStart()
    {
        if (!getInput<int>("agent_id", agent_id_))
            throw BT::RuntimeError("IsAnyoneLookingAtMeNode: missing input [agent_id]");
        if (!getInput<double>("time_step", dt_))
            throw BT::RuntimeError("IsAnyoneLookingAtMeNode: missing input [time_step]");
        if (!getInput<double>("distance_threshold", distance_threshold_))
            distance_threshold_ = 5.0;
        if (!getInput<double>("angle_threshold", angle_threshold_))
            angle_threshold_ = 0.3; // 17º approx
        if (!getInput<double>("duration", duration_))
            duration_ = 5.0;

        // Retrieve AgentManager pointer
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("IsAnyoneLookingAtMeNode: global AgentManager pointer not set");
        }

        start_time_ = std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus IsAnyoneLookingAtMeNode::onRunning()
    {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();

        auto dt_msg = getInput<double>("time_step");
        if (!dt_msg)
            throw BT::RuntimeError("IsAnyoneLookingAtMeNode: missing input [time_step] during onRunning", dt_msg.error());
        //double dt = dt_msg.value();

        // Note: Position updates should be handled by RegularNav in the BT structure
        // agent_manager_->updatePosition(agent_id_, dt);
        // if (agent_manager_->goalReached(agent_id_))
        // {
        //     agent_manager_->updateGoal(agent_id_);
        // }

        // Get the position of the main agent
        auto myPos = agent_manager_->getAgentPosition(agent_id_);
        int foundObserver = -1;

        // Iterate over all agents
        for (const auto &kv : agent_manager_->getAgents())
        {
            int obs_id = kv.first;
            if (obs_id == agent_id_)
                continue;

            // Compute distance from observer to the main agent
            auto obsPos = agent_manager_->getAgentPosition(obs_id);
            double dx = myPos.getX() - obsPos.getX();
            double dy = myPos.getY() - obsPos.getY();
            double dist = std::hypot(dx, dy);
            if (dist > distance_threshold_)
                continue;

            // Compute angle from observer to main agent
            double desired_angle = std::atan2(myPos.getY() - obsPos.getY(), myPos.getX() - obsPos.getX());
            double observer_yaw = agent_manager_->getAgentYaw(obs_id);
            double angle_diff = std::remainder(observer_yaw - desired_angle, 2 * M_PI);
            if (std::fabs(angle_diff) <= angle_threshold_)
            {
                foundObserver = obs_id;
                break;
            }
        }

        if (foundObserver != -1)
        {
            setOutput("observer_id", foundObserver);
            return BT::NodeStatus::SUCCESS;
        }

        if (elapsed >= duration_)
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::RUNNING;
    }

    void IsAnyoneLookingAtMeNode::onHalted()
    {
    }

} // namespace hunav
