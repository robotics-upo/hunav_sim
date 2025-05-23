#include "hunav_agent_manager/conversation_formation_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <chrono>
#include <cmath>
#include <sstream>
#include <iostream>

namespace hunav
{

  BT::NodeStatus ConversationFormationNode::onStart()
  {
    if (!getInput<int>("main_agent_id", main_agent_id_))
      throw BT::RuntimeError("ConversationFormationNode: missing [main_agent_id]");
    if (!getInput<double>("conversation_duration", conversation_duration_))
      throw BT::RuntimeError("ConversationFormationNode: missing [conversation_duration]");
    if (!getInput<double>("center_x", center_x_))
      throw BT::RuntimeError("ConversationFormationNode: missing [center_x]");
    if (!getInput<double>("center_y", center_y_))
      throw BT::RuntimeError("ConversationFormationNode: missing [center_y]");
    if (!getInput<double>("time_step", dt_))
      throw BT::RuntimeError("ConversationFormationNode: missing [time_step]");

    std::string ids_str;
    if (!getInput<std::string>("non_main_agent_ids", ids_str))
      throw BT::RuntimeError("ConversationFormationNode: missing [non_main_agent_ids]");

    std::vector<int> parsed_ids;
    std::istringstream iss(ids_str);
    std::string token;
    while (std::getline(iss, token, ','))
    {
      try
      {
        parsed_ids.push_back(std::stoi(token));
      }
      catch (const std::exception &e)
      {
        throw BT::RuntimeError("ConversationFormationNode: failed to convert token '" + token + "' to int");
      }
    }

    non_main_ids_ = parsed_ids;

    // Retrieve AgentManager
    if (agent_manager_ == nullptr)
    {
      agent_manager_ = hunav::g_agent_manager;
      if (agent_manager_ == nullptr)
        throw BT::RuntimeError("ConversationFormationNode: global AgentManager pointer not set");
    }

    // Set conversation center from provided inputs
    conversation_center_.set(center_x_, center_y_);

    // Vector of all participants IDs
    all_ids_.clear();
    all_ids_.push_back(main_agent_id_);
    for (int id : non_main_ids_)
      all_ids_.push_back(id);

    // Save original goals for all agents
    original_goals_.clear();
    for (int id : all_ids_)
    {
      original_goals_[id] = agent_manager_->getAgentGoals(id);
    }

    // Compute formation radius (scaling with number of non-main agents)
    double computed_radius = 0.5 + 0.2 * non_main_ids_.size();

    // Get the main agent's yaw to serve as the reference angle
    double starting_angle = agent_manager_->getAgentYaw(main_agent_id_);

    size_t N = all_ids_.size();
    double angle_step = 2 * M_PI / static_cast<double>(N);

    // For each agent, compute its current angle relative to the conversation center,
    // then assign the candidate position (starting_angle + i*angle_step) that minimizes the angular difference.
    for (int id : all_ids_)
    {
      // Get the agent’s current position
      utils::Vector2d pos = agent_manager_->getAgentPosition(id);

      // Compute the Euclidean distance to the target
      double ax = pos.getX();
      double ay = pos.getY();

      double current_angle = std::atan2(ay - center_y_, ax - center_x_);
      agent_angles_.push_back(std::make_pair(id, current_angle));
    }

    std::sort(agent_angles_.begin(), agent_angles_.end(),
              [](const std::pair<int, double> &a, const std::pair<int, double> &b)
              {
                return a.second < b.second;
              });

    for (size_t i = 0; i < agent_angles_.size(); i++)
    {
      double candidate_angle = starting_angle + i * angle_step;
      // Normalize candidate_angle within [-π, π]
      candidate_angle = std::remainder(candidate_angle, 2 * M_PI);

      double target_x = center_x_ + computed_radius * std::cos(candidate_angle);
      double target_y = center_y_ + computed_radius * std::sin(candidate_angle);

      int assigned_agent = agent_angles_[i].first;
      sfm::Goal goal;
      goal.center.set(target_x, target_y);
      goal.radius = 0.1;

      // Set the goal for this agent
      agent_manager_->clearAndSetAgentGoal(assigned_agent, goal);

      // std::cout << "[ConversationFormationNode] Assigned candidate position ("
      //           << target_x << ", " << target_y << ") at angle " << candidate_angle
      //           << " to agent " << assigned_agent << ".\n";
    }

    // std::cout << "[ConversationFormationNode] Waiting for all agents to reach targets.\n";

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus ConversationFormationNode::onRunning()
  {
    double current_dt;
    auto dt_msg = getInput<double>("time_step");
    if (!dt_msg)
      throw BT::RuntimeError("ConversationFormationNode: missing input [time_step] in onRunning", dt_msg.error());
    current_dt = dt_msg.value();

    bool mainReached = agent_manager_->goalReached(main_agent_id_);

    // Update main agent
    if (!mainReached)
      agent_manager_->updatePosition(main_agent_id_, current_dt);
    else
      agent_manager_->lookAtPoint(main_agent_id_, conversation_center_);

    // Update non-main agents
    bool othersReached = true;
    for (int id : non_main_ids_)
    {
      if (!agent_manager_->goalReached(id))
      {
        othersReached = false;
      }
      else
        agent_manager_->lookAtPoint(id, conversation_center_);
    }

    if (mainReached && othersReached)
    {
      // Start conversation timer if not already started
      if (conversation_start_time_ == std::chrono::steady_clock::time_point())
      {
        conversation_start_time_ = std::chrono::steady_clock::now();
        // std::cout << "[ConversationFormationNode] All agents reached targets. Conversation started.\n";
      }

      // Check if conversation duration has elapsed
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - conversation_start_time_).count();
      if (elapsed >= conversation_duration_)
      {
        // Restore original goals for all agents
        for (int id : all_ids_)
        {
          agent_manager_->restoreAgentGoals(id, original_goals_[id]);
        }
        // std::cout << "[ConversationFormationNode] Conversation ended after "
        //           << conversation_duration_ << " seconds. Original goals restored.\n";
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  void ConversationFormationNode::onHalted()
  {
  }

} // namespace hunav
