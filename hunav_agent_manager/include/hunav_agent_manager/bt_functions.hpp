#pragma once

// forward‐declare so extern compiles
namespace hunav {
  class BTfunctions;
  extern BTfunctions * g_btfunctions;
}

#include "hunav_agent_manager/agent_manager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hunav_msgs/msg/agent.hpp"
#include "hunav_msgs/msg/agents.hpp"

// Behavior Trees
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
//#ifdef ZMQ_FOUND
//#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
//#endif

#include <iostream>
//#include <memory>
#include <chrono>
#include <math.h> /* fabs */
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <map>
#include <geometry_msgs/msg/point.hpp>

// Social Force Model
#include <lightsfm/sfm.hpp>

namespace hunav {

  extern AgentManager* g_agent_manager;

class BTfunctions {
public:
  /**
   * @brief Construct a new BTfunctions object
   *
   */
  BTfunctions();
  /**
   * @brief Destroy the BTfunctions object
   *
   */
  ~BTfunctions();

  void init();

  void setGlobalGoals(const std::map<int,geometry_msgs::msg::Point> &goals) {
    global_goals_ = goals;
  }

  void updateAllAgents(const hunav_msgs::msg::Agent::SharedPtr robot,
                       const hunav_msgs::msg::Agents::SharedPtr msg) {
    agent_manager_.updateAllAgents(robot, msg);
  }

  void updateAgentsAndRobot(const hunav_msgs::msg::Agents::SharedPtr msg) {
    agent_manager_.updateAgentsAndRobot(msg);
  }

  hunav_msgs::msg::Agents getUpdatedAgents() {
    return agent_manager_.getUpdatedAgentsMsg();
  }
  hunav_msgs::msg::Agent getUpdatedAgent(int id) {
    return agent_manager_.getUpdatedAgentMsg(id);
  }

  // bool running();
  // bool ok() { return agent_manager_.canCompute(); }

  sfm::Forces getAgentForces(int id) {
    return agent_manager_.getAgentForces(id);
  };

  // BT Conditions
  BT::NodeStatus robotVisible(BT::TreeNode &self);
  BT::NodeStatus goalReached(BT::TreeNode &self);

  // BT Actions to be registered with SimpleActionNode
  BT::NodeStatus updateGoal(BT::TreeNode &self);
  BT::NodeStatus regularNav(BT::TreeNode &self);
  BT::NodeStatus surprisedNav(BT::TreeNode &self);
  BT::NodeStatus scaredNav(BT::TreeNode &self);
  BT::NodeStatus curiousNav(BT::TreeNode &self);
  BT::NodeStatus threateningNav(BT::TreeNode &self);
  // the impassive behavior is taken into account
  // in the ComputeForces method, by adding the robot
  // to the agent's obstacles. So we do not need to
  // implement the method.
  // BT::NodeStatus impassiveNav(BT::TreeNode &self);

  BT::NodeStatus findNearestAgent(BT::TreeNode& self);
  BT::NodeStatus agentVisible(BT::TreeNode& self);
  BT::NodeStatus saySomething(BT::TreeNode& self);
  BT::NodeStatus lookAtAgent(BT::TreeNode& self);
  BT::NodeStatus lookAtRobot(BT::TreeNode& self);
  BT::NodeStatus lookAtPoint(BT::TreeNode& self);
  BT::NodeStatus isRobotClose(BT::TreeNode& self);
  BT::NodeStatus isAgentClose(BT::TreeNode & self);
  BT::NodeStatus robotFacingAgent(BT::TreeNode & self);
  BT::NodeStatus randomChance(BT::TreeNode& self);
  BT::NodeStatus setGoal(BT::TreeNode & self);
  BT::NodeStatus setGroupId(BT::TreeNode & self);
  BT::NodeStatus isAtPosition(BT::TreeNode& self);
  BT::NodeStatus blockRobot(BT::TreeNode & self);
  BT::NodeStatus blockAgent(BT::TreeNode & self);
  BT::NodeStatus resumeMovement(BT::TreeNode& self);
  BT::NodeStatus stopMovement(BT::TreeNode& self);

  std::map<int,geometry_msgs::msg::Point> global_goals_;
  geometry_msgs::msg::Point getGlobalGoal(int id) const;

private:
  AgentManager agent_manager_;
};

} // namespace hunav
