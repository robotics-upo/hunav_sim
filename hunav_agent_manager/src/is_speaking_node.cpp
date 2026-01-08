#include "hunav_agent_manager/is_speaking_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/agent_say_node.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <sstream>
#include <iostream>
#include <cmath>

namespace hunav
{
    BT::NodeStatus IsSpeakingNode::onStart()
    {
        if (!getInput<int>("agent_id", listener_id_))
            throw BT::RuntimeError("IsSpeakingNode: missing input [listener_id]");
        if (!getInput<double>("time_step", dt_))
            throw BT::RuntimeError("IsSpeakingNode: missing input [time_step]");
        if (!getInput<int>("target_id", target_id_))
            throw BT::RuntimeError("IsSpeakingNode: missing input [target_id]");
        if (!getInput<double>("distance_threshold", distance_threshold_))
            distance_threshold_ = 5.0;
        if (!getInput<double>("duration", duration_))
            duration_ = 10.0;

        // Retrieve AgentManager pointer
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("IsSpeakingNode: global AgentManager pointer not set");
        }

        start_time_ = std::chrono::steady_clock::now();

        // Ensure AgentSayNode is available
        if (!hunav::AgentSayNode::getInstance())
            throw BT::RuntimeError("IsSpeakingNode: AgentSayNode instance is not set");

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus IsSpeakingNode::onRunning()
    {
        // Compute elapsed time
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();

        auto dt_msg = getInput<double>("time_step");
        if (!dt_msg)
            throw BT::RuntimeError("IsSpeakingNode: missing input [time_step] during onRunning", dt_msg.error());
        //double dt = dt_msg.value();

        // Ensure normal navigation
        // agent_manager_->updatePosition(listener_id_, dt);

        // if (agent_manager_->goalReached(listener_id_))
        // {
        //     agent_manager_->updateGoal(listener_id_);
        // }

        // Get the last published message
        std::string msg = hunav::AgentSayNode::getInstance()->getLastMessage();

        if (!msg.empty())
        {
            // Expected format: "Agent X says: <message>"
            std::istringstream iss(msg);
            std::string word;
            int speakerID = -1;
            iss >> word >> speakerID;

            // If the speaker is not the target, ignore it
            if (speakerID != target_id_)
            {
            }
            else
            {
                // Check distance between listener and target
                utils::Vector2d listenerPos = agent_manager_->getAgentPosition(listener_id_);
                utils::Vector2d targetPos = agent_manager_->getAgentPosition(target_id_);
                double dx = targetPos.getX() - listenerPos.getX();
                double dy = targetPos.getY() - listenerPos.getY();
                double dist = std::hypot(dx, dy);

                if (dist <= distance_threshold_)
                {
                    return BT::NodeStatus::SUCCESS;
                }
            }
        }

        if (elapsed >= duration_)
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::RUNNING;
    }

    void IsSpeakingNode::onHalted()
    {
    }

} // namespace hunav
