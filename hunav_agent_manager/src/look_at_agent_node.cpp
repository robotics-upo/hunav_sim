#include "hunav_agent_manager/look_at_agent_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <cmath>

namespace hunav
{

    BT::NodeStatus LookAtAgentNode::onStart()
    {
        // 1) Read required inputs
        if (!getInput<int>("observer_id", observer_id_))
            throw BT::RuntimeError("LookAtAgentNode: missing [observer_id]");
        if (!getInput<int>("target_id", target_id_))
            throw BT::RuntimeError("LookAtAgentNode: missing [target_id]");
        if (!getInput<double>("yaw_tolerance", yaw_tolerance_))
            yaw_tolerance_ = 0.01;

        // 2) Fetch the global AgentManager pointer
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("LookAtAgentNode: global AgentManager pointer not set");
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus LookAtAgentNode::onRunning()
    {
        // 1) Each tick, nudge the observer toward the target
        agent_manager_->lookAtAgent(observer_id_, target_id_);

        // 2) Immediately read back the observer’s current yaw (in radians):
        double current_yaw = agent_manager_->getAgentYaw(observer_id_);

        // 3) Recompute the “world‐space” desired angle from observer→target:
        auto obs_pos = agent_manager_->getAgentPosition(observer_id_);
        auto tgt_pos = agent_manager_->getAgentPosition(target_id_);

        double dx = tgt_pos.getX() - obs_pos.getX();
        double dy = tgt_pos.getY() - obs_pos.getY();
        double desired_yaw = std::atan2(dy, dx);

        // 4) Compute normalized error in [–π, π]:
        double error = std::remainder(desired_yaw - current_yaw, 2 * M_PI);

        // 5) If within tolerance, succeed. Otherwise keep running.
        if (std::fabs(error) <= yaw_tolerance_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void LookAtAgentNode::onHalted()
    {
    }

} // namespace hunav
