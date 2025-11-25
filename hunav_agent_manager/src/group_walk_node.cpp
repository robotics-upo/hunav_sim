#include "hunav_agent_manager/group_walk_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <sstream>
#include <iostream>

namespace hunav
{

    BT::NodeStatus GroupWalkNode::onStart()
    {
        if (!getInput<int>("main_agent_id", main_agent_id_))
            throw BT::RuntimeError("GroupWalkNode: missing input [main_agent_id]");
        if (!getInput<double>("time_step", dt_))
            throw BT::RuntimeError("GroupWalkNode: missing input [time_step]");
        if (!getInput<double>("duration", duration_))
            duration_ = 0.0;

        // Retrieve AgentManager pointer
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("GroupWalkNode: global AgentManager pointer not set");
        }

        // Flexibly read non_main_agent_ids - handles both string "2,3" and int values
        std::string ids_str;
        if (!getFlexibleString(*this, "non_main_agent_ids", ids_str))
            throw BT::RuntimeError("GroupWalkNode: missing input [non_main_agent_ids]");

        // Parse the comma-separated string into a vector of ints
        if (!parseAgentIdList(ids_str, non_main_ids_))
        {
            throw BT::RuntimeError("GroupWalkNode: failed to parse [non_main_agent_ids]: '" + ids_str + "'");
        }

        // Build the complete list of participants (main agent + non-main agents)
        all_ids_.clear();
        all_ids_.push_back(main_agent_id_);
        for (int id : non_main_ids_)
            all_ids_.push_back(id);

        // Save original goals for each non-main agent
        original_goals_.clear();
        for (int id : non_main_ids_)
        {
            original_goals_[id] = agent_manager_->getAgentGoals(id);
        }

        // Set a common group ID for all participants
        // The main agent's ID is chosen as the common group ID
        int common_group_id = main_agent_id_;
        for (int id : all_ids_)
        {
            agent_manager_->setAgentGroupId(id, common_group_id);
        }

        // Override the goals of each non-main agent so they match the main agent’s goals
        for (int id : non_main_ids_)
        {
            agent_manager_->overrideAgentGoals(main_agent_id_, id);
            // std::cout << "[GroupWalkNode] Agent " << id
            //           << " goals overridden to match main agent " << main_agent_id_ << ".\n";
        }

        // If a positive duration is provided, record the start time
        if (duration_ > 0.0)
            start_time_ = std::chrono::steady_clock::now();

        // std::cout << "[GroupWalkNode] Group walk initiated for agents: ";
        // for (int id : all_ids_)
        //     std::cout << id << " ";
        // std::cout << "with common group ID " << common_group_id
        //           << " for duration " << duration_ << " seconds.\n";

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus GroupWalkNode::onRunning()
    {

        double dt;
        auto dt_msg = getInput<double>("time_step");
        if (!dt_msg)
            throw BT::RuntimeError("GroupWalkNode: missing input [time_step] during onRunning", dt_msg.error());
        dt = dt_msg.value();

        // Update position/orientation of the main agent
        agent_manager_->updatePosition(main_agent_id_, dt);
        if (agent_manager_->goalReached(main_agent_id_))
        {
            agent_manager_->updateGoal(main_agent_id_);
        }

        // If a duration is specified, check if it has elapsed
        if (duration_ > 0.0)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
            if (elapsed >= duration_)
            {
                // Restore original goals for each non-main agent
                for (int id : non_main_ids_)
                {
                    agent_manager_->restoreAgentGoals(id, original_goals_[id]);
                }
                // Reset group ID for all participants 
                for (int id : all_ids_)
                {
                    agent_manager_->setAgentGroupId(id, -1);
                }
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void GroupWalkNode::onHalted()
    {
        // Restore original goals for each non-main agent
        for (int id : non_main_ids_)
        {
            agent_manager_->restoreAgentGoals(id, original_goals_[id]);
        }
        // Reset group IDs for all participants
        for (int id : all_ids_)
        {
            agent_manager_->setAgentGroupId(id, -1);
        }
    }

} // namespace hunav
