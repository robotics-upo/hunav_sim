
#ifndef HUNAV_BEHAVIORS__AGENT_MANAGER_HPP_
#define HUNAV_BEHAVIORS__AGENT_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
// #include <ament_index_cpp/get_package_prefix.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include "nav_msgs/msg/OccupancyGrid.hpp"
// WATCH OUT! ALTOUGH THE NAME OF MESSAGE FILES HAVE
// CAPITAL LETTER (e.g. Agent and IsRobotVisible) THE
// FILES GENERATED FROM MESSAGE GENERATION ARE SNAKE CASE!
// ('Agent' goes to 'agent', and 'IsRobotVisible' goes to 'is_robot_visible')
#include "hunav_msgs/msg/agent.hpp"
#include "hunav_msgs/msg/agents.hpp"
//#include "hunav_msgs/srv/is_robot_visible.hpp"
// #include "hunav_msgs/srv/compute_agents.hpp"

// // Behavior Trees
// #include "behaviortree_cpp_v3/behavior_tree.h"
// #include "behaviortree_cpp_v3/bt_factory.h"
// #include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
// #include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
// #include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
// #ifdef ZMQ_FOUND
// #include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
// #endif

#include <iostream>
//#include <memory>
#include <chrono>
#include <math.h> /* fabs */
#include <mutex>
#include <string>

// Social Force Model
#include <lightsfm/sfm.hpp>

namespace hunav
{

struct agentBehavior
{
  int type = 1;
  int state = 0;
  int configuration = 0;  // def, manual, random
  double duration = 40.0;
  bool once = true;
  double vel = 0.0;
  double dist = 0.0;
  double forceFactor = 0.0;
};

struct agent
{
  std::string name;
  int type;
  // int behavior;
  // int behavior_state;
  agentBehavior behavior;
  sfm::Agent sfmAgent;
};

class AgentManager
{
public:
  /**
   * @brief Construct a new Agent Manager object
   *
   */
  AgentManager();
  /**
   * @brief Destroy the Agent Manager object
   *
   */
  ~AgentManager();

  void init();

  // bool running();
  bool canCompute();

  /**
   * @brief method to update the agents
   *
   * @param msg
   */
  void updateAllAgents(const hunav_msgs::msg::Agent::SharedPtr robot_msg, const hunav_msgs::msg::Agents::SharedPtr msg);
  /**
   * @brief method to update the agents
   *
   * @param msg
   */
  bool updateAgents(const hunav_msgs::msg::Agents::SharedPtr msg);
  /**
   * @brief method to update the robot
   *
   * @param msg
   */
  void updateAgentRobot(const hunav_msgs::msg::Agent::SharedPtr msg);
  /**
   * @brief method to update the robot
   *
   * @param msg
   */
  void updateAgentsAndRobot(const hunav_msgs::msg::Agents::SharedPtr msg);
  /**
   * @brief compute the forces of the sfm_agents_
   *
   */
  void computeForces();
  void computeForces(int id);
  /**
   * @brief initialize the sfm_agents_ based on the agents_ vector
   *
   */
  void initializeAgents(const hunav_msgs::msg::Agents::SharedPtr msg);
  /**
   * @brief initialize the srobot_ based on the agent msg of the robot
   *
   */
  void initializeRobot(const hunav_msgs::msg::Agent::SharedPtr msg);

  /**
   * @brief return the vector of agents in format of sfm lib
   *
   */
  std::vector<sfm::Agent> getSFMAgents();

  /**
   * @brief update the agent position by applying the forces
   * for a short time period
   *
   * @param id identifier of the agent
   * @param dt time to compute the agent's movement
   */
  void updatePosition(int id, double dt);
  /**
   * @brief build a Agents msg based on the info of the sfm_agents_
   * @return hunav_msgs::msg::Agents msg
   */
  hunav_msgs::msg::Agents getUpdatedAgentsMsg();

  /**
   * @brief build a Agent msg based on the id of the agent
   * @param id integer id of the agent
   * @return hunav_msgs::msg::Agent msg
   */
  hunav_msgs::msg::Agent getUpdatedAgentMsg(int id);

  /**
   * @brief return the forces of the agent
   * @param id integer id of the agent
   * @return sfm::Forces of the agent
   */
  sfm::Forces getAgentForces(int id)
  {
    return agents_[id].sfmAgent.forces;
  };

  /**
   * @brief computed the squared distance between the robot and the agent
   * indicated by the parameter id.
   *
   * @param id int value that is the agent id and the vector index of the agent
   * @return float
   */
  float robotSquaredDistance(int id);
  
  /**
   * @brief stop the agent translation and changing its orientation to look at
   * the robot
   *
   * @param id int id of the desired agent (index of the agents_ array too)
   */
  void lookAtTheRobot(int id);
  void lookAtAgent(int observer_id, int target_id);
  void lookAtPoint(int agent_id, const utils::Vector2d & target);  
  /**
   * @brief check if the robot is in the field of view of the agent indicated by
   * the parameter id
   *
   * @param id int id of the desired agent (index of the agents_ array too)
   * @param dist maximum distance to detect the robot
   * @return true if the robot is the field of view of the agent
   * @return false otherwise
   */
  bool isRobotVisible(int id, double dist);
  /**
   * @brief check if the robot is in the line of sight of teh agent indicated by
   * the parameter id
   *
   * @param id int id of the desired agent (index of the agents_ array too)
   * @return true if the robot is in the line of sight
   * @return false otherwise
   */
  bool lineOfSight(int id);

  /**
   * @brief change the current navigation goal of the agent and its
   * maximum velocity so the agent approaches the robot
   *
   * @param id identifier of the agent
   * @param dt time to compute the agent's movement
   * @param closest_dist minimum distance between the robot and the agent
   */
  void approximateRobot(int id, double dt, double closest_dist = 1.5, double max_vel = 1.0);

  /**
   * @brief the agent will try to keep more distance from the robot
   *
   * @param id identifier of the agent
   * @param dt time to compute the agent's movement
   */
  void avoidRobot(int id, double dt, double safe_distance = 5.0, double max_vel = 0.6);

  /**
   * @brief the agent will try to block the path of the robot
   *
   * @param id identifier of the agent
   * @param dt time to compute the agent's movement
   */
  void blockRobot(int id, double dt, double front_dist = 1.4);
  void blockAgent(int id, int target_id, double dt, double front_dist = 1.4);

  int findNearestAgent(int id);
  bool isAgentVisible(int observer_id, int target_id, double dist, double fov_threshold = (M_PI / 2.0) + 0.17);
  void approachAgent(int id, int target_id, double dt, double closest_dist = 1.5, double max_vel = 1.5);
  void followAgent(int id, int target_id, double dt, double threshold = 1.5, double max_vel = 1.5);
  void freezeAgent(int id);
  void resumeAgent(int id);

  bool isRobotFacingAgent(int agent_id);
  bool isAgentClose(int observer_id, int target_id, double threshold);
  std::vector<int> findNearestAgents(int id, int k);

  void setAgentGoal(int id, const sfm::Goal & goal);
  void clearAndSetAgentGoal(int id, const sfm::Goal & goal);
  void clearAndSetAgentGoals(int id, const std::list<sfm::Goal> &goals);
  void setAgentGroupId(int id, int group_id);

  void restoreAgentGoals(int agent_id, const std::list<sfm::Goal>& goals);
  void overrideAgentGoals(int agent_id, int target_id);

  const std::unordered_map<int, agent>& getAgents() const { return agents_; }
  std::list<sfm::Goal> getAgentGoals(int id);

  utils::Vector2d computeConversationCenter(int agent_id, double circle_offset);

  double getAgentYaw(int id);
  utils::Vector2d getAgentPosition(int id);
  utils::Vector2d getRobotPosition();
  bool goalReached(int id);
  bool updateGoal(int id);

  int step_count;
  int step_count2;
  bool move;

  inline double normalizeAngle(double a)
  {
    double value = a;
    while (value <= -M_PI)
      value += 2 * M_PI;
    while (value > M_PI)
      value -= 2 * M_PI;
    return value;
  }

  inline utils::Vector2d computeDesiredForce(sfm::Agent& agent) const
  {
    utils::Vector2d desiredDirection;
    if (!agent.goals.empty() && (agent.goals.front().center - agent.position).norm() > agent.goals.front().radius)
    {
      utils::Vector2d diff = agent.goals.front().center - agent.position;
      desiredDirection = diff.normalized();
      agent.forces.desiredForce = agent.params.forceFactorDesired *
                                  (desiredDirection * agent.desiredVelocity - agent.velocity) /
                                  agent.params.relaxationTime;
      agent.antimove = false;
    }
    else
    {
      agent.forces.desiredForce = -agent.velocity / agent.params.relaxationTime;
      agent.antimove = true;
    }
    return desiredDirection;
  }


protected:
  // std::vector<bool> agent_status_;
  // std::unordered_map<int, bool> agents_computed_;
  // int status_;
  bool agents_received_;
  bool robot_received_;
  bool agents_initialized_;
  bool robot_initialized_;
  std::mutex mutex_;
  // std::vector<hunav_msgs::msg::Agent> agents_;
  // std::vector<sfm::Agent> sfm_agents_;
  std::unordered_map<int, agent> agents_;
  // hunav_msgs::msg::Agent robot_;
  agent robot_;
  std_msgs::msg::Header header_;
  // sfm::Agent sfm_robot_;
  float max_dist_view_;
  float max_dist_view_squared_;
  double time_step_secs_;
  rclcpp::Time prev_time_;
  // rclcpp::Clock::SharedPtr clock_;

  // std::string pkg_shared_tree_dir_;
  // std::vector<BT::Tree> trees_;

  // // Topic subscriptions
  // rclcpp::Subscription<hunav_msgs::msg::Agents>::SharedPtr agents_sub_;

  // // Services provided
  // rclcpp::Service<hunav_msgs::srv::ComputeAgents>::SharedPtr
  // agents_srv_;
  std::unordered_map<int, std::list<sfm::Goal>> override_goals_data_;
  std::unordered_map<int, float> orig_desired_vels_;
  std::unordered_map<int, std::chrono::steady_clock::time_point> last_interaction_time_;
};

}  // namespace hunav
#endif  // HUNAV_BEHAVIORS__AGENT_MANAGER_HPP_
