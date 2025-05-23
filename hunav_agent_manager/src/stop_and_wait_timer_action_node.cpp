#include "hunav_agent_manager/stop_and_wait_timer_action_node.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <chrono>
#include <iostream>

namespace hunav
{

  BT::NodeStatus StopAndWaitTimerActionNode::onStart()
  {
    // Retrieve required inputs.
    if (!getInput<int>("agent_id", agent_id_))
    {
      throw BT::RuntimeError("StopAndWaitTimerActionNode: missing required input [agent_id]");
    }
    if (!getInput<double>("wait_duration", wait_duration_))
    {
      throw BT::RuntimeError("StopAndWaitTimerActionNode: missing required input [wait_duration]");
    }

    // Retrieve AgentManager
    if (agent_manager_ == nullptr)
    {
      agent_manager_ = hunav::g_agent_manager;
      if (agent_manager_ == nullptr)
        throw BT::RuntimeError("StopAndWaitTimerActionNode: global AgentManager pointer not set");
    }

    // Freeze the agent so that it stops moving
    agent_manager_->freezeAgent(agent_id_);

    // Record the starting time
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus StopAndWaitTimerActionNode::onRunning()
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();

    // If the wait duration has elapsed, resume the agent
    if (elapsed >= wait_duration_)
    {
      agent_manager_->resumeAgent(agent_id_);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void StopAndWaitTimerActionNode::onHalted()
  {
  }

} // namespace hunav
