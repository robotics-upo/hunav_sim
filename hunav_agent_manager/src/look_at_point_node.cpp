#include "hunav_agent_manager/look_at_point_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <iostream>
#include <cmath>

namespace hunav
{

    BT::NodeStatus LookAtPointNode::onStart()
    {
    // 1) Read all required inputs
    if (!getInput<int>("agent_id", agent_id_))
        throw BT::RuntimeError("LookAtPointNode: missing [agent_id]");

    auto gid_msg = getInput<int>("goal_id");
    if (!gid_msg)
        throw BT::RuntimeError("LookAtPointNode: missing [goal_id]");
    int goal_id = gid_msg.value();  

    if (g_btfunctions == nullptr) {
        throw BT::RuntimeError("LookAtPointNode: BTfunctions singleton not initialized");
    }
    auto pt = g_btfunctions->getGlobalGoal(goal_id);
    target_x_ = pt.x;
    target_y_ = pt.y;

    // 2) optional yaw_tolerance
    if (!getInput<double>("yaw_tolerance", yaw_tolerance_))
        yaw_tolerance_ = 0.05;

    // 3) grab AgentManager 
    if (agent_manager_ == nullptr) {
        agent_manager_ = g_agent_manager;
        if (!agent_manager_)
        throw BT::RuntimeError("LookAtPointNode: no AgentManager");
    }

    // 4) compute the desired yaw
    auto pos = agent_manager_->getAgentPosition(agent_id_);
    desired_yaw_rad_ = std::atan2(target_y_ - pos.getY(),
                                    target_x_ - pos.getX());

    return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus LookAtPointNode::onRunning()
    {

        // Call the incremental lookAtPoint in AgentManager
        utils::Vector2d target_vec;
        target_vec.set(target_x_, target_y_);
        agent_manager_->lookAtPoint(agent_id_, target_vec);

        // After rotating a bit, check the agent’s current yaw
        double current_yaw = agent_manager_->getAgentYaw(agent_id_);

        // Compute the shortest‐path difference ∈ [–π, π]
        double error = std::remainder(desired_yaw_rad_ - current_yaw, 2 * M_PI);

        // If the absolute difference is within tolerance, finish successfully
        if (std::fabs(error) <= yaw_tolerance_)
        {
            return BT::NodeStatus::SUCCESS;
        }

        // 6) Otherwise, keep spinning
        return BT::NodeStatus::RUNNING;
    }

    void LookAtPointNode::onHalted()
    {
    }

} // namespace hunav
