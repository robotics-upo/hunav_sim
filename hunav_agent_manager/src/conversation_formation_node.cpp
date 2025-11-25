#include "hunav_agent_manager/conversation_formation_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <iostream>
#include <tuple>
#include <algorithm>

namespace hunav
{

  BT::NodeStatus ConversationFormationNode::onStart()
  {
    // 1) Read all required inputs
    if (!getInput<int>("main_agent_id", main_agent_id_))
      throw BT::RuntimeError("ConversationFormationNode: missing [main_agent_id]");
    if (!getInput<double>("conversation_duration", conversation_duration_))
      throw BT::RuntimeError("ConversationFormationNode: missing [conversation_duration]");
    if (!getInput<int>("goal_id", center_goal_id_))
      throw BT::RuntimeError("ConversationFormationNode: missing [goal_id]");
    if (!getInput<double>("time_step", dt_))
      throw BT::RuntimeError("ConversationFormationNode: missing [time_step]");

    // Flexibly read non_main_agent_ids - handles both string "2,3" and int values
    std::string ids_str;
    if (!getFlexibleString(*this, "non_main_agent_ids", ids_str))
      throw BT::RuntimeError("ConversationFormationNode: missing [non_main_agent_ids]");

    // Parse the comma‐separated list of non‐main IDs
    if (!parseAgentIdList(ids_str, non_main_ids_))
    {
      throw BT::RuntimeError("ConversationFormationNode: failed to parse [non_main_agent_ids]: '" + ids_str + "'");
    }

    // 2) Retrieve AgentManager
    if (agent_manager_ == nullptr)
    {
      agent_manager_ = hunav::g_agent_manager;
      if (agent_manager_ == nullptr)
        throw BT::RuntimeError("ConversationFormationNode: global AgentManager pointer not set");
    }

    // 3) Build the full list of participant IDs
    all_ids_.clear();
    all_ids_.push_back(main_agent_id_);
    for (int id : non_main_ids_)
      all_ids_.push_back(id);

    // 4) Snapshot original goals so we can restore later
    original_goals_.clear();
    for (int id : all_ids_)
    {
      original_goals_[id] = agent_manager_->getAgentGoals(id);
    }

    // 5) Compute a “conversation circle” radius
    double computed_radius = 0.5 + 0.2 * static_cast<double>(non_main_ids_.size());

    // 6) Use the main agent’s current yaw as the reference angle
    double main_yaw_rad = agent_manager_->getAgentYaw(main_agent_id_);

    // 7) Generate N candidate positions (evenly spaced around the circle)
    size_t N = all_ids_.size();
    double angle_step = (2.0 * M_PI) / static_cast<double>(N);

    // Conversation center
    auto pt = g_btfunctions->getGlobalGoal(center_goal_id_);
    // Pre‐compute all N circle coordinates (x,y)
    std::vector<std::pair<double, double>> candidate_positions(N);
    for (size_t k = 0; k < N; ++k)
    {
      double candidate_angle = main_yaw_rad + static_cast<double>(k) * angle_step;
      // Normalize to [–π, π]
      candidate_angle = std::remainder(candidate_angle, 2.0 * M_PI);

      double px = pt.x + (computed_radius * std::cos(candidate_angle));
      double py = pt.y + (computed_radius * std::sin(candidate_angle));
      candidate_positions[k] = std::make_pair(px, py);
    }

    // 8) Build a list of all (agent_index, position_index, distance) tuples
    struct Triple
    {
      size_t agent_idx;
      size_t pos_idx;
      double dist;
    };
    std::vector<Triple> all_pairs;
    all_pairs.reserve(N * N);

    for (size_t i = 0; i < N; ++i)
    {
      int agent_id = all_ids_[i];
      // Fetch the agent’s current (x,y)
      utils::Vector2d pos = agent_manager_->getAgentPosition(agent_id);
      double ax = pos.getX();
      double ay = pos.getY();

      for (size_t j = 0; j < N; ++j)
      {
        double px = candidate_positions[j].first;
        double py = candidate_positions[j].second;
        double dx = ax - px;
        double dy = ay - py;
        double d = std::sqrt(dx * dx + dy * dy);
        all_pairs.push_back({i, j, d});
      }
    }

    // 9) Sort all pairs by increasing distance
    std::sort(all_pairs.begin(), all_pairs.end(),
              [](const Triple &a, const Triple &b)
              {
                return a.dist < b.dist;
              });

    // 10) Greedy assignment: each agent → closest free spot
    std::vector<bool> agent_assigned(N, false), pos_assigned(N, false);
    std::vector<size_t> assignment(N, SIZE_MAX); // assignment[i] = pos_idx for agent i

    size_t assigned_count = 0;
    for (const auto &t : all_pairs)
    {
      if (assigned_count >= N)
        break;

      if (!agent_assigned[t.agent_idx] && !pos_assigned[t.pos_idx])
      {
        assignment[t.agent_idx] = t.pos_idx;
        agent_assigned[t.agent_idx] = true;
        pos_assigned[t.pos_idx] = true;
        ++assigned_count;
      }
    }

    // 11) Push one temporary goal onto each agent’s goal list
    for (size_t i = 0; i < N; ++i)
    {
      int agent_id = all_ids_[i];
      size_t pos_idx = assignment[i];
      double tx = candidate_positions[pos_idx].first;
      double ty = candidate_positions[pos_idx].second;

      sfm::Goal goal;
      goal.center.set(tx, ty);
      goal.radius = 0.1; // conversational “personal space”

      // We use setAgentGoal (push front) so existing goals remain behind
      agent_manager_->setAgentGoal(agent_id, goal);

      // std::cout << "[ConversationFormationNode] Assigned agent " << agent_id
      //           << " → circle spot (" << tx << ", " << ty << ")\n";
    }

    // 12) Store the circle center (for lookAtPoint)
    conversation_center_.set(pt.x, pt.y);

    // We’ll track, each tick, whether each agent is “ready” (at‐spot + oriented).
    // Here we clear any previous state (just in case this node is reused).
    readiness_.clear();
    readiness_.resize(N, false);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus ConversationFormationNode::onRunning()
  {
    // 1) Re‐read dt each tick
    auto dt_msg = getInput<double>("time_step");
    if (!dt_msg)
      throw BT::RuntimeError("ConversationFormationNode: missing input [time_step] in onRunning",
                             dt_msg.error());
    dt_ = dt_msg.value();

    size_t N = all_ids_.size();
    const double yaw_tolerance = 0.01; // ~0.57° tolerance

    bool all_ready = true;

    // 2) For each agent, check “at‐spot” & “oriented”
    for (size_t i = 0; i < N; ++i)
    {
      int agent_id = all_ids_[i];

      // Fetch current position
      utils::Vector2d pos = agent_manager_->getAgentPosition(agent_id);
      double ax = pos.getX();
      double ay = pos.getY();

      // Find that agent’s current goal center (the topmost goal)
      // We know we pushed exactly one goal onStart(), so:
      sfm::Goal current_goal = agent_manager_->getAgentGoals(agent_id).front();
      double gx = current_goal.center.getX();
      double gy = current_goal.center.getY();

      // 2a) Check “at spot”: Euclidean distance to the goal center
      double dx = ax - gx;
      double dy = ay - gy;
      double dist_to_goal = std::sqrt(dx * dx + dy * dy);

      if (dist_to_goal > current_goal.radius)
      {
        // Not yet at spot: move toward it
        agent_manager_->updatePosition(agent_id, dt_);
        readiness_[i] = false;
        all_ready = false;
        continue;
      }

      // 2b) Agent is “at its spot” (within goal.radius). Now check orientation.
      // Compute the desired yaw to face the conversation center
      double desired_yaw = std::atan2(conversation_center_.getY() - ay,
                                      conversation_center_.getX() - ax);
      double current_yaw = agent_manager_->getAgentYaw(agent_id);
      double yaw_error = std::remainder(desired_yaw - current_yaw, 2.0 * M_PI);

      if (std::fabs(yaw_error) > yaw_tolerance)
      {
        // Not yet oriented: rotate a bit
        agent_manager_->lookAtPoint(agent_id, conversation_center_);
        readiness_[i] = false;
        all_ready = false;
      }
      else
      {
        // This agent is both “at spot” and “oriented”
        readiness_[i] = true;
      }
    }

    // 3) If all agents are “ready,” start or continue the conversation timer
    if (all_ready)
    {
      // 3a) If this is the first time all are ready, record the start time
      if (conversation_start_time_ == std::chrono::steady_clock::time_point())
      {
        conversation_start_time_ = std::chrono::steady_clock::now();
        // std::cout << "[ConversationFormationNode] All agents in place & oriented. Conversation begins.\n";
      }

      // 3b) Check elapsed time
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                           now - conversation_start_time_)
                           .count();
      if (elapsed >= conversation_duration_)
      {
        // 3c) Conversation is over: restore original goals, return SUCCESS
        for (const auto &kv : original_goals_)
        {
          int id = kv.first;
          const auto &goals = kv.second;
          agent_manager_->clearAndSetAgentGoals(id, goals);
        }
        // std::cout << "[ConversationFormationNode] Conversation ended after "
        //           << conversation_duration_ << " seconds. Restored goals.\n";
        return BT::NodeStatus::SUCCESS;
      }
    }
    else
    {
      // If not all ready yet, we reset conversation_start_time_ so that the timer
      // only counts once everyone is actually in place & oriented.
      conversation_start_time_ = std::chrono::steady_clock::time_point();
    }

    return BT::NodeStatus::RUNNING;
  }

  void ConversationFormationNode::onHalted()
  {
    // If this node is halted prematurely, restore all original goals immediately
    for (const auto &kv : original_goals_)
    {
      int id = kv.first;
      const auto &goals = kv.second;
      agent_manager_->clearAndSetAgentGoals(id, goals);
    }
  }

} // namespace hunav
