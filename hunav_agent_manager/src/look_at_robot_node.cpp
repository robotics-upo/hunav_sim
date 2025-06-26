#include "hunav_agent_manager/look_at_robot_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <cmath>

namespace hunav
{

    BT::NodeStatus LookAtRobotNode::onStart()
    {
        // 1) Read required inputs
        if (!getInput<int>("agent_id", agent_id_))
            throw BT::RuntimeError("LookAtRobotNode: missing [agent_id]");
        if (!getInput<double>("yaw_tolerance", yaw_tolerance_))
            yaw_tolerance_ = 0.01;

        // 2) Fetch the global AgentManager pointer
        if (agent_manager_ == nullptr)
        {
            agent_manager_ = hunav::g_agent_manager;
            if (agent_manager_ == nullptr)
                throw BT::RuntimeError("LookAtRobotNode: global AgentManager pointer not set");
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus LookAtRobotNode::onRunning()
    {
        // 1) Each tick, nudge the agent’s yaw toward the robot
        agent_manager_->lookAtTheRobot(agent_id_);

        // 2) Read back the agent’s current yaw (in radians)
        double current_yaw = agent_manager_->getAgentYaw(agent_id_);

        // 3) Compute the “world‐space” desired angle from agent → robot
        auto agent_pos = agent_manager_->getAgentPosition(agent_id_);
        auto robot_pos = agent_manager_->getRobotPosition();
        double dx = robot_pos.getX() - agent_pos.getX();
        double dy = robot_pos.getY() - agent_pos.getY();
        double desired_yaw = std::atan2(dy, dx);

        // 4) Normalize error into [–π, π]
        double error = std::remainder(desired_yaw - current_yaw, 2 * M_PI);

        // 5) If within tolerance, return SUCCESS
        if (std::fabs(error) <= yaw_tolerance_)
        {
            return BT::NodeStatus::SUCCESS;
        }

        // 6) Otherwise keep spinning (RUNNING)
        return BT::NodeStatus::RUNNING;
    }

    void LookAtRobotNode::onHalted()
    {
    }

} // namespace hunav
