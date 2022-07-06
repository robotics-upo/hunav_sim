
#ifndef HUNAV_BEHAVIORS__AGENT_MANAGER_HPP_
#define HUNAV_BEHAVIORS__AGENT_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
// #include <ament_index_cpp/get_package_prefix.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

namespace hunav {

struct agent {
  std::string name;
  int type;
  int behavior;
  sfm::Agent sfmAgent;
};

class AgentManager {
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
  void updateAllAgents(const hunav_msgs::msg::Agent::SharedPtr robot_msg,
                       const hunav_msgs::msg::Agents::SharedPtr msg);
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
  sfm::Forces getAgentForces(int id) { return agents_[id].sfmAgent.forces; };

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
   */
  void approximateRobot(int id, double dt);

  /**
   * @brief the agent will try to keep more distance from the robot
   *
   * @param id identifier of the agent
   * @param dt time to compute the agent's movement
   */
  void avoidRobot(int id, double dt);

  /**
   * @brief the agent will try to block the path of the robot
   *
   * @param id identifier of the agent
   * @param dt time to compute the agent's movement
   */
  void blockRobot(int id, double dt);

  bool goalReached(int id);
  bool updateGoal(int id);

  int step_count;
  int step_count2;
  bool move;

  inline double normalizeAngle(double a) {
    double value = a;
    while (value <= -M_PI)
      value += 2 * M_PI;
    while (value > M_PI)
      value -= 2 * M_PI;
    return value;
  }

  // inline void computeSocialForce(sfm::Agent &me,
  //                                std::vector<sfm::Agent> &agents) const {
  //   // Agent& agent = agents[index];
  //   // me.forces.socialForce.set(0, 0);
  //   utils::Vector2d socforce;
  //   for (unsigned i = 0; i < agents.size(); i++) {
  //     if (agents[i].id == me.id) {
  //       continue;
  //     }
  //     printf("[SOCFOR] me x: %.2f, y:%.2f -- agent[%i] x:%.2f, y:%.2f\n",
  //            me.position.getX(), me.position.getY(), i,
  //            agents[i].position.getX(), agents[i].position.getY());
  //     utils::Vector2d diff = agents[i].position - me.position;
  //     printf("\t[SOCFOR] position diff x: %.2f, y:%.2f\n", diff.getX(),
  //            diff.getY());
  //     utils::Vector2d diffDirection = diff.normalized();
  //     // printf("[SOCFOR] DiffDirection norm: %.4f\n", diffDirection.norm());
  //     utils::Vector2d velDiff = me.velocity - agents[i].velocity;
  //     utils::Vector2d interactionVector =
  //         me.params.lambda * velDiff + diffDirection;
  //     printf("\t[SOCFOR] interactionVector x: %.2f, y:%.2f\n",
  //            interactionVector.getX(), interactionVector.getY());
  //     double interactionLength = interactionVector.norm();
  //     printf("\t[SOCFOR] interactionLength: %.4f\n", interactionLength);
  //     utils::Vector2d interactionDirection =
  //         interactionVector / interactionLength;
  //     utils::Angle theta = interactionDirection.angleTo(diffDirection);
  //     double B = me.params.gamma * interactionLength;
  //     double thetaRad = theta.toRadian();
  //     double forceVelocityAmount =
  //         -std::exp(-diff.norm() / B - PW(me.params.nPrime * B * thetaRad));
  //     printf("\t[SOCFOR] forceVelocityAmount: %.4f\n", forceVelocityAmount);
  //     double forceAngleAmount =
  //         -theta.sign() *
  //         std::exp(-diff.norm() / B - PW(me.params.n * B * thetaRad));
  //     printf("\t[SOCFOR] forceAngleAmount: %.4f\n", forceAngleAmount);
  //     utils::Vector2d forceVelocity =
  //         forceVelocityAmount * interactionDirection;
  //     utils::Vector2d forceAngle =
  //         forceAngleAmount * interactionDirection.leftNormalVector();
  //     // me.forces.socialForce +=
  //     socforce += me.params.forceFactorSocial * (forceVelocity + forceAngle);
  //     printf("\t[SOCFOR] socfor x: %.2f, y:%.2f\n", socforce.getX(),
  //            socforce.getY());
  //   }
  //   printf("[SOCFOR] social force nr: %.4f, x:%.4f, y:%.4f\n",
  //   socforce.norm(),
  //          socforce.getX(), socforce.getY());
  // }

  inline utils::Vector2d computeDesiredForce(sfm::Agent &agent) const {
    utils::Vector2d desiredDirection;
    if (!agent.goals.empty() &&
        (agent.goals.front().center - agent.position).norm() >
            agent.goals.front().radius) {
      utils::Vector2d diff = agent.goals.front().center - agent.position;
      desiredDirection = diff.normalized();
      agent.forces.desiredForce =
          agent.params.forceFactorDesired *
          (desiredDirection * agent.desiredVelocity - agent.velocity) /
          agent.params.relaxationTime;
      agent.antimove = false;
    } else {
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
};

} // namespace hunav
#endif // HUNAV_BEHAVIORS__AGENT_MANAGER_HPP_
