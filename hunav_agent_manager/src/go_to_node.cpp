#include "hunav_agent_manager/go_to_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <iostream>

namespace hunav
{

  BT::NodeStatus GoToNode::onStart()
  {
    if (!getInput<int>("agent_id", agent_id_))
      throw BT::RuntimeError("GoToNode: missing [agent_id]");
    if (!getInput<double>("target_x", target_x_))
      throw BT::RuntimeError("GoToNode: missing [target_x]");
    if (!getInput<double>("target_y", target_y_))
      throw BT::RuntimeError("GoToNode: missing [target_y]");
    if (!getInput<double>("time_step", dt_))
      throw BT::RuntimeError("GoToNode: missing [time_step]");
    if (!getInput<bool>("temporary", temporary_))
      throw BT::RuntimeError("GoToNode: missing [temporary]");
    if (!getInput<double>("stop_duration", stop_duration_))
      throw BT::RuntimeError("GoToNode: missing [stop_duration]");

    // Retrieve AgentManager pointer
    if (agent_manager_ == nullptr)
    {
      agent_manager_ = hunav::g_agent_manager;
      if (agent_manager_ == nullptr)
        throw BT::RuntimeError("GoToNode: global AgentManager pointer not set");
    }

    // Create a new goal
    sfm::Goal goal;
    goal.center.set(target_x_, target_y_);
    goal.radius = 0.05;

    if (temporary_)
    {
      // Save the original goals to restore later
      original_goals_ = agent_manager_->getAgentGoals(agent_id_);
    }

    // Set the new goal
    agent_manager_->clearAndSetAgentGoal(agent_id_, goal);
    // std::cout << "[GoToNode] Agent " << agent_id_
    //           << " goal set to (" << target_x_ << ", " << target_y_ << ")."
    //           << (temporary_ ? " [Temporary]" : " [Permanent]") << "\n";

    // Reset freeze_start_time_ to indicate timer hasn't started yet
    freeze_start_time_ = std::chrono::steady_clock::time_point();

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus GoToNode::onRunning()
  {
    double dt;
    auto dt_msg = getInput<double>("time_step");
    if (!dt_msg)
      throw BT::RuntimeError("GoToNode: missing input [time_step] during onRunning", dt_msg.error());
    dt = dt_msg.value();

    // Check if the agent has reached the goal
    if (!agent_manager_->goalReached(agent_id_))
    {
      agent_manager_->updatePosition(agent_id_, dt);
      return BT::NodeStatus::RUNNING;
    }

    // At this point, the goal has been reached
    if (!temporary_)
    {
      // std::cout << "[GoToNode] Agent " << agent_id_
      //           << " reached the permanent goal (" << target_x_ << ", " << target_y_ << ").\n";
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      // For temporary goal: start freeze timer if not already started
      if (freeze_start_time_ == std::chrono::steady_clock::time_point())
      {
        freeze_start_time_ = std::chrono::steady_clock::now();
        // agent_manager_->freezeAgent(agent_id_);
        std::cout << "[GoToNode] Agent " << agent_id_
                  << " reached temporary goal. Freeze timer started.\n";
      }
      // Check elapsed freeze time
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - freeze_start_time_).count();
      if (elapsed >= stop_duration_)
      {
        // Restore the original goals
        agent_manager_->restoreAgentGoals(agent_id_, original_goals_);
        // agent_manager_->resumeAgent(agent_id_);
        // std::cout << "[GoToNode] Temporary goal duration elapsed for agent " << agent_id_
        //           << ". Original goals restored.\n";
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  void GoToNode::onHalted()
  {
    if (temporary_)
    {
      // Restore original goals if the node is halted
      agent_manager_->restoreAgentGoals(agent_id_, original_goals_);
      // std::cout << "[GoToNode] Halted: Temporary goal aborted for agent " << agent_id_
      //           << ". Original goals restored.\n";
    }
    else
    {
      // std::cout << "[GoToNode] Halted: Agent " << agent_id_ << " GoTo action aborted.\n";
    }
  }

} // namespace hunav
