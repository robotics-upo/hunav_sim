#include "hunav_agent_manager/is_anyone_speaking_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/agent_say_node.hpp" 
#include "hunav_agent_manager/bt_functions.hpp"
#include <sstream>
#include <iostream>
#include <cmath>

namespace hunav
{

    BT::NodeStatus IsAnyoneSpeakingNode::onStart()
    {
        if (!getInput<int>("agent_id", agent_id_))
            throw BT::RuntimeError("IsAnyoneSpeakingNode: missing input [agent_id]");
        if (!getInput<double>("time_step", dt_))
            throw BT::RuntimeError("IsAnyoneSpeakingNode: missing input [time_step]");
        if (!getInput<double>("distance_threshold", distance_threshold_))
            distance_threshold_ = 5.0;
        if (!getInput<double>("duration", duration_))
            duration_ = 5.0;

        if (!hunav::AgentSayNode::getInstance())
            throw BT::RuntimeError("IsAnyoneSpeakingNode: AgentSayNode instance is not set");

        // Retrieve AgentManager pointer
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("IsAnyoneSpeakingNode: global AgentManager pointer not set");
        }

        // Clear any previously stored message
        {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            last_msg_.clear();
        }

        start_time_ = std::chrono::steady_clock::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus IsAnyoneSpeakingNode::onRunning()
    {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();

        auto dt_msg = getInput<double>("time_step");
        if (!dt_msg)
            throw BT::RuntimeError("IsAnyoneSpeakingNode: missing input [time_step] during onRunning", dt_msg.error());
        //double dt = dt_msg.value();

        // Update position/orientation of the main agent - COMMENTED OUT to avoid navigation issues
        // agent_manager_->updatePosition(agent_id_, dt);

        // if (agent_manager_->goalReached(agent_id_))
        // {
        //     agent_manager_->updateGoal(agent_id_);
        // }

        // Get the last message from agent_say topic
        std::string msg = hunav::AgentSayNode::getInstance()->getLastMessage();

        {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            last_msg_ = msg;
        }

        if (!last_msg_.empty())
        {
            // Expected format: "Agent X says: <message>"
            std::istringstream iss(last_msg_);
            std::string word;
            int speaker_id = -1;
            iss >> word >> speaker_id;

            if (speaker_id == agent_id_)
            {
                // Ignore messages from self
                {
                    std::lock_guard<std::mutex> lock(msg_mutex_);
                    last_msg_.clear();
                }
            }
            else
            {
                utils::Vector2d listenerPos = agent_manager_->getAgentPosition(agent_id_);
                utils::Vector2d speakerPos = agent_manager_->getAgentPosition(speaker_id);
                double dx = speakerPos.getX() - listenerPos.getX();
                double dy = speakerPos.getY() - listenerPos.getY();
                double dist = std::hypot(dx, dy);

                if (dist <= distance_threshold_)
                {
                    setOutput("speaker_id", speaker_id);
                    RCLCPP_INFO(rclcpp::get_logger("IsAnyoneSpeakingNode"), "Agent %d heard Agent %d speaking within %.2f meters.", agent_id_, speaker_id, distance_threshold_);
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    // Clear message if not within threshold
                    {
                        std::lock_guard<std::mutex> lock(msg_mutex_);
                        last_msg_.clear();
                    }
                }
            }
        }

        if (elapsed >= duration_)
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::RUNNING;
    }

    void IsAnyoneSpeakingNode::onHalted()
    {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        last_msg_.clear();
    }

} // namespace hunav
