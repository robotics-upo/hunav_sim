#include "hunav_agent_manager/go_to_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <iostream>
#include <cmath>

namespace hunav
{

  BT::NodeStatus GoToNode::onStart()
  {
    // 1) Read required inputs
    if (!getInput<int>("agent_id", agent_id_))
      throw BT::RuntimeError("GoToNode: missing [agent_id]");

    // 2) Read required goal_id 
    auto goal_id_msg = getInput<int>("goal_id");
    if (!goal_id_msg)
      throw BT::RuntimeError("GoToNode: missing required input [goal_id]");

    int goal_id = goal_id_msg.value(); 

    if (g_btfunctions == nullptr) {
      throw BT::RuntimeError("GoToNode: BTfunctions singleton not initialized");
    }
    auto pt = g_btfunctions->getGlobalGoal(goal_id);
    target_x_ = pt.x;
    target_y_ = pt.y;

    if (!getInput<double>("time_step", dt_))
      throw BT::RuntimeError("GoToNode: missing [time_step]");
    if (!getInput<double>("tolerance", tolerance_))
      throw BT::RuntimeError("GoToNode: missing [tolerance]");

    // 3) Fetch the global AgentManager pointer
    if (agent_manager_ == nullptr)
    {
      agent_manager_ = hunav::g_agent_manager;
      if (agent_manager_ == nullptr)
        throw BT::RuntimeError("GoToNode: global AgentManager pointer not set");
    }

    // 4) Snapshot any existing goals, so we can restore later
    original_goals_ = agent_manager_->getAgentGoals(agent_id_);

    // 5) Create exactly one “temporary” goal at (target_x_, target_y_)
    sfm::Goal goal;
    goal.center.set(target_x_, target_y_);
    goal.radius = 0.1;
    agent_manager_->clearAndSetAgentGoal(agent_id_, goal);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus GoToNode::onRunning()
  {
    // 1) Re‐read dt in case it’s dynamic
    auto dt_msg = getInput<double>("time_step");
    if (!dt_msg)
      throw BT::RuntimeError("GoToNode: missing input [time_step] during onRunning",
                             dt_msg.error());
    dt_ = dt_msg.value();

    // 2) Compute current distance to (target_x_, target_y_)
    // utils::Vector2d pos = agent_manager_->getAgentPosition(agent_id_);
    // double dx = pos.getX() - target_x_;
    // double dy = pos.getY() - target_y_;
    // double distance = std::sqrt(dx * dx + dy * dy);

    // 3) If within tolerance_, remove the temporary goal and succeed
    if (agent_manager_->goalReached(agent_id_)) //(distance <= tolerance_)
    {
      // Restore the original goals exactly as they were
      agent_manager_->restoreAgentGoals(agent_id_, original_goals_);
      return BT::NodeStatus::SUCCESS;
    }
    // Check if the agent has reached the goal
    // if (!agent_manager_->goalReached(agent_id_))
    // {
    //   agent_manager_->updatePosition(agent_id_, dt);
    //   return BT::NodeStatus::RUNNING;
    // }

    // 4) Otherwise, we haven’t reached it yet → keep moving
    agent_manager_->updatePosition(agent_id_, dt_);
    return BT::NodeStatus::RUNNING;
  }

  void GoToNode::onHalted()
  {
    // If the tree is halted mid‐goal, restore original goals so we don’t leave the
    // agent stuck on a half‐completed temporary goal.
    agent_manager_->clearAndSetAgentGoals(agent_id_, original_goals_);
  }

} // namespace hunav
