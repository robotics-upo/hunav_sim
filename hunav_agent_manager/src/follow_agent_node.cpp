#include "hunav_agent_manager/follow_agent_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

namespace hunav
{

  BT::NodeStatus FollowAgentNode::onStart()
  {
    if (!getInput<int>("agent_id", agent_id_))
      throw BT::RuntimeError("FollowAgentNode: missing input [agent_id]");
    if (!getInput<int>("target_agent_id", target_agent_id_))
      throw BT::RuntimeError("FollowAgentNode: missing input [target_agent_id]");
    if (!getInput<double>("time_step", dt_))
      throw BT::RuntimeError("FollowAgentNode: missing input [time_step]");
    if (!getInput<double>("closest_dist", closest_dist_))
      closest_dist_ = 1.5; // Use default if not provided
    if (!getInput<double>("max_vel", max_vel_))
      max_vel_ = 1.5; // Use default if not provided
    if (!getInput<double>("duration", duration_))
      duration_ = 0.0;

    // Retrieve AgentManager
    if (agent_manager_ == nullptr)
    {
      agent_manager_ = hunav::g_agent_manager;
      if (agent_manager_ == nullptr)
        throw BT::RuntimeError("FollowAgentNode: global AgentManager pointer not set");
    }

    // If duration > 0, record the start time
    if (duration_ > 0.0)
      start_time_ = std::chrono::steady_clock::now();

    //   std::cout << "[FollowAgentNode] Starting approach for agent " << agent_id_
    //             << " towards target " << target_agent_id_
    //             << "  (duration=" << duration_ << ").\n";

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus FollowAgentNode::onRunning()
  {
    double current_dt;
    auto dt_msg = getInput<double>("time_step");
    if (!dt_msg)
      throw BT::RuntimeError("FollowAgentNode: missing input [time_step] in onRunning", dt_msg.error());
    current_dt = dt_msg.value();

    agent_manager_->followAgent(agent_id_, target_agent_id_, current_dt, closest_dist_, max_vel_);

    if (duration_ > 0.0)
    {
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
      if (elapsed >= duration_)
      {
        //   std::cout << "[FollowAgentNode] Duration elapsed (" << duration_
        //             << " s) for agent " << agent_id_ << ".\n";
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  void FollowAgentNode::onHalted()
  {
  }

} // namespace hunav
