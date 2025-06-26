#include "hunav_agent_manager/block_agent_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <cmath>
#include <iostream>

namespace hunav
{

    BT::NodeStatus BlockAgentNode::onStart()
    {
        if (!getInput<int>("agent_id", agent_id_))
            throw BT::RuntimeError("BlockAgentNode: missing input [agent_id]");
        if (!getInput<int>("target_agent_id", target_agent_id_))
            throw BT::RuntimeError("BlockAgentNode: missing input [target_agent_id]");
        if (!getInput<double>("time_step", dt_))
            throw BT::RuntimeError("BlockAgentNode: missing input [time_step]");
        if (!getInput<double>("front_dist", front_dist_))
            throw BT::RuntimeError("BlockAgentNode: missing input [front_dist]");
        if (!getInput<double>("duration", duration_))
            duration_ = 0.0;

        // Retrieve AgentManager
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("BlockAgentNode: global AgentManager pointer not set");
        }

        if (duration_ > 0.0)
            start_time_ = std::chrono::steady_clock::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus BlockAgentNode::onRunning()
    {
        double dt;
        auto dt_msg = getInput<double>("time_step");
        if (!dt_msg)
            throw BT::RuntimeError("BlockAgentNode: missing input [time_step] during onRunning", dt_msg.error());
        dt = dt_msg.value();

        agent_manager_->blockAgent(agent_id_, target_agent_id_, dt, front_dist_);

        if (duration_ > 0.0)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
            if (elapsed >= duration_)
                return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void BlockAgentNode::onHalted()
    {
    }

} // namespace hunav
