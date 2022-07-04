#include "hunav_agent_manager/agent_manager.hpp"

namespace hunav_agent_manager {

// using std::placeholders::_1;
// using std::placeholders::_2;

AgentManager::AgentManager() { //: initialized_(false), max_dist_view_(10.0) {

  init();
  printf("[AgentManager.Constructor] AgentManager initialized \n");
}

AgentManager::~AgentManager() {}

void AgentManager::init() {
  agents_initialized_ = false;
  robot_initialized_ = false;
  agents_received_ = false;
  robot_received_ = false;
  max_dist_view_ = 10.0;
  time_step_secs_ = 0.0;
  step_count = 1;
  step_count2 = 1;
  move = false;
  // max_dist_view_squared_ = max_dist_view_ * max_dist_view_;
  printf("[AgentManager.init] initialized \n");
}

float AgentManager::robotSquaredDistance(int id) {
  // std::lock_guard<std::mutex> guard(mutex_);
  // mutex_.lock();

  // hunav_msgs/msg/Agents
  float xa = agents_[id].sfmAgent.position.getX(); // position.position.x;
  float ya = agents_[id].sfmAgent.position.getY(); // position.y;
  float xr = robot_.sfmAgent.position.getX();      // position.x;
  float yr = robot_.sfmAgent.position.getY();      // position.y;

  //   sfm agents
  //   float xa = agents_[request->agent_id].position.getX();
  //   float ya = agents_[request->agent_id].position.getY();
  //   float xr = robot_.position.getX();
  //   float yr = robot_.position.getY();

  // mutex_.unlock();
  // double d = (xr - xa) * (xr - xa) + (yr - ya) * (yr - ya);
  // std::cout << "AgentManager.robotSquaredDistance:" << sqrt(d) << std::endl;
  return (xr - xa) * (xr - xa) + (yr - ya) * (yr - ya);
}

bool AgentManager::lineOfSight(int id) {

  // mutex_.lock();
  float ax = agents_[id].sfmAgent.position.getX();
  float ay = agents_[id].sfmAgent.position.getY();
  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();
  double yaw = agents_[id].sfmAgent.yaw.toRadian();
  // tf2::Quaternion q(
  //     agents_[id].position.orientation.x, agents_[id].position.orientation.y,
  //     agents_[id].position.orientation.z,
  //     agents_[id].position.orientation.w);
  // mutex_.unlock();
  // tf2::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  float nrx = (rx - ax) * cos(yaw) + (ry - ay) * sin(yaw);
  float nry = -(rx - ax) * sin(yaw) + (ry - ay) * cos(yaw);
  float rangle = atan2(nry, nrx);

  if (abs(rangle) > M_PI / 2.0) {
    // std::cout << "[AgentManager.lineOfSight] Agent " << id + 1
    //           << " can NOT see the robot!" << std::endl;
    return false;
  } else {
    // std::cout << "[AgentManager.lineOfSight] Agent " << id + 1
    //           << " is seeing the robot!" << std::endl;

    // ----------------------------------------
    // TODO: do raytracing to check visibility
    // ----------------------------------------

    return true;
  }
}

bool AgentManager::isRobotVisible(int id, double dist) {
  std::lock_guard<std::mutex> guard(mutex_);
  float squared_dist = robotSquaredDistance(id);
  if (squared_dist <= (dist * dist)) {
    return lineOfSight(id);
  } else {
    return false;
  }
}

// void AgentManager::isRobotVisible(
//     const std::shared_ptr<hunav_msgs::srv::IsRobotVisible::Request>
//     request, std::shared_ptr<hunav_msgs::srv::IsRobotVisible::Response>
//     response) {
//   response->visible = isRobotVisible(request->agent_id)
// }

void AgentManager::lookAtTheRobot(int id) {

  std::lock_guard<std::mutex> guard(mutex_);
  // Robot position
  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();
  // Agent position
  float ax = agents_[id].sfmAgent.position.getX();
  float ay = agents_[id].sfmAgent.position.getY();
  float ah = agents_[id].sfmAgent.yaw.toRadian();
  // Transform robot position to agent coords system
  float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
  float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
  utils::Angle robotYaw; // = utils::Angle::fromRadian(atan2(nry, nrx));
  robotYaw.setRadian(atan2(nry, nrx));

  // Change the angle step by step according to
  // the time_step_secs_ and a maximum angular vel
  float max_ang_vel = M_PI; // rad/secs
  utils::Angle max_angle =
      utils::Angle::fromRadian(max_ang_vel * 0.01); // time_step_secs_);
  if (robotYaw.sign() < 0)
    max_angle.setRadian(max_angle.toRadian() * (-1));

  // Update the agent angle
  if (fabs(robotYaw.toRadian()) > max_angle.toRadian()) {
    agents_[id].sfmAgent.yaw = (agents_[id].sfmAgent.yaw + max_angle);
  } else {
    agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;
  }
}

void AgentManager::approximateRobot(int id, double dt) {

  std::lock_guard<std::mutex> guard(mutex_);

  // Robot position
  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();
  float dist = sqrt(robotSquaredDistance(id));

  // if the agent is close to the robot,
  // stop and look at the robot
  if (dist <= 1.5) {
    // printf("Agent %i stoping and looking at the robot! dist: %.2f\n", id,
    // dist);
    // Agent position
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
    float ah = agents_[id].sfmAgent.yaw.toRadian();
    // Transform robot position to agent coords system
    float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
    float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
    utils::Angle robotYaw; // = utils::Angle::fromRadian(atan2(nry, nrx));
    robotYaw.setRadian(atan2(nry, nrx));

    agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;

  } else {

    // Change the agent goal
    sfm::Goal g;
    g.center.set(rx, ry);
    g.radius = robot_.sfmAgent.radius;
    agents_[id].sfmAgent.goals.push_front(g);

    // change agent vel according to the proximity of the robot
    // move slowly when close
    float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
    agents_[id].sfmAgent.desiredVelocity = 1.8 * (dist / max_dist_view_);

    // printf("Agent %i approximating robot! dist: %.2f, desiredvel: %.3f\n",
    // id,
    //        dist, agents_[id].sfmAgent.desiredVelocity);

    // recompute forces
    computeForces(id);
    // update position
    sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

    // restore values just in case the approximation
    // ends in the next iteration
    agents_[id].sfmAgent.goals.pop_front();
    agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;
  }
}

void AgentManager::blockRobot(int id, double dt) {

  std::lock_guard<std::mutex> guard(mutex_);

  // Robot position
  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();

  float h = robot_.sfmAgent.yaw.toRadian();

  // Store the initial set o goals
  std::list<sfm::Goal> gls = agents_[id].sfmAgent.goals;

  // Change the agent goal.
  // We should compute a goal in front of the robot heading.
  float newgx = rx + 1.1 * sin(h);
  float newgy = ry + 1.1 * cos(h);
  sfm::Goal g;
  g.center.set(newgx, newgy);
  g.radius = 0.05; // robot_.sfmAgent.radius;
  agents_[id].sfmAgent.goals.push_front(g);

  // We should change the weights of the sfm agent???

  // change agent vel according to the proximity of the robot
  // move slowly when close
  float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
  // agents_[id].sfmAgent.desiredVelocity = 2.0 * (dist / max_dist_view_);
  agents_[id].sfmAgent.desiredVelocity = 2.0;

  // printf("Agent %i approximating robot! dist: %.2f, desiredvel: %.3f\n",
  // id,
  //        dist, agents_[id].sfmAgent.desiredVelocity);

  // recompute forces
  computeForces(id);
  // update position
  sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

  // if the agent is close to the robot,
  // look at the robot
  float dist = sqrt(robotSquaredDistance(id));
  if (dist <= 1.2) {
    // printf("Agent %i stoping and looking at the robot! dist: %.2f\n", id,
    // dist);
    // Agent position
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
    float ah = agents_[id].sfmAgent.yaw.toRadian();
    // Transform robot position to agent coords system
    float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
    float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
    utils::Angle robotYaw; // = utils::Angle::fromRadian(atan2(nry, nrx));
    robotYaw.setRadian(atan2(nry, nrx));

    agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;
  }

  // restore the goals and the velocity
  agents_[id].sfmAgent.goals = gls;
  agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;
}

void AgentManager::avoidRobot(int id, double dt) {
  std::lock_guard<std::mutex> guard(mutex_);

  // we decrease the maximum velocity
  double init_vel = agents_[id].sfmAgent.desiredVelocity;
  agents_[id].sfmAgent.desiredVelocity = 0.6;
  computeForces(id);

  // We add an extra repulsive force from the robot
  utils::Vector2d minDiff =
      agents_[id].sfmAgent.position - robot_.sfmAgent.position;
  double distance = minDiff.norm() - agents_[id].sfmAgent.radius;

  utils::Vector2d Scaryforce =
      20.0 * (agents_[id].sfmAgent.params.forceSigmaObstacle / distance) *
      minDiff.normalized();
  agents_[id].sfmAgent.forces.globalForce += Scaryforce;

  // update position
  sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

  // restore desired vel
  agents_[id].sfmAgent.desiredVelocity = init_vel;
}

bool AgentManager::goalReached(int id) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!agents_[id].sfmAgent.goals.empty() &&
      (agents_[id].sfmAgent.goals.front().center -
       agents_[id].sfmAgent.position)
              .norm() <= (agents_[id].sfmAgent.goals.front().radius + 0.1)) {
    printf("Agent %s goal reached!", agents_[id].name.c_str());
    return true;
  } else
    return false;
}

bool AgentManager::updateGoal(int id) {
  std::lock_guard<std::mutex> guard(mutex_);

  printf("Updating goal for agent %i\n\n", id);
  sfm::Goal g = agents_[id].sfmAgent.goals.front();
  agents_[id].sfmAgent.goals.pop_front();
  if (agents_[id].sfmAgent.cyclicGoals) {
    agents_[id].sfmAgent.goals.push_back(g);
  }
  return true;
}

void AgentManager::initializeAgents(
    const hunav_msgs::msg::Agents::SharedPtr msg) {

  printf("Initializing SFM Agents...\n");
  for (auto a : msg->agents) {
    agent ag;
    ag.name = a.name;
    ag.type = a.type;
    ag.behavior = a.behavior;
    ag.sfmAgent.id = a.id;
    ag.sfmAgent.groupId = a.group_id;
    ag.sfmAgent.desiredVelocity = a.desired_velocity;
    ag.sfmAgent.radius = a.radius;
    ag.sfmAgent.cyclicGoals = a.cyclic_goals;
    ag.sfmAgent.position.set(a.position.position.x, a.position.position.y);
    ag.sfmAgent.yaw.setRadian(a.yaw);
    ag.sfmAgent.velocity.set(a.velocity.linear.x, a.velocity.linear.y);
    ag.sfmAgent.linearVelocity =
        sqrt(a.velocity.linear.x * a.velocity.linear.x +
             a.velocity.linear.y * a.velocity.linear.y);
    ag.sfmAgent.angularVelocity = a.velocity.angular.z;
    for (auto g : a.goals) {
      sfm::Goal sfmg;
      sfmg.center.setX(g.position.x);
      sfmg.center.setY(g.position.y);
      sfmg.radius = a.goal_radius;
      ag.sfmAgent.goals.push_back(sfmg);
    }
    ag.sfmAgent.obstacles1.clear();
    if (!a.closest_obs.empty()) {
      for (auto obs : a.closest_obs) {
        utils::Vector2d o;
        o.set(obs.x, obs.y);
        ag.sfmAgent.obstacles1.push_back(o);
      }
    }
    ag.sfmAgent.params.forceFactorSocial = 40.0; // 2.1 by default
    // ag.sfmAgent.params.forceFactorDesired = 5.0;
    // ag.sfmAgent.params.forceFactorObstacle = 20.0;

    agents_[ag.sfmAgent.id] = ag;
    printf("\tagent %s, x:%.2f, y:%.2f, th:%.2f\n",
           agents_[ag.sfmAgent.id].name.c_str(),
           agents_[ag.sfmAgent.id].sfmAgent.position.getX(),
           agents_[ag.sfmAgent.id].sfmAgent.position.getY(),
           agents_[ag.sfmAgent.id].sfmAgent.yaw.toRadian());
  }
  agents_initialized_ = true;
  printf("SFM Agents initialized\n");
}

void AgentManager::initializeRobot(
    const hunav_msgs::msg::Agent::SharedPtr msg) {

  robot_.name = msg->name;
  robot_.type = msg->type;
  robot_.behavior = msg->behavior;
  robot_.sfmAgent.id = msg->id;
  robot_.sfmAgent.groupId = msg->group_id;
  robot_.sfmAgent.desiredVelocity = msg->desired_velocity;
  robot_.sfmAgent.radius = msg->radius;
  robot_.sfmAgent.cyclicGoals = msg->cyclic_goals;
  robot_.sfmAgent.position.set(msg->position.position.x,
                               msg->position.position.y);
  robot_.sfmAgent.yaw.setRadian(msg->yaw);
  robot_.sfmAgent.velocity.set(msg->velocity.linear.x, msg->velocity.linear.y);
  robot_.sfmAgent.linearVelocity =
      sqrt(msg->velocity.linear.x * msg->velocity.linear.x +
           msg->velocity.linear.y * msg->velocity.linear.y);
  robot_.sfmAgent.angularVelocity = msg->velocity.angular.z;

  printf("\trobot %i, x:%.2f, y:%.2f\n", robot_.sfmAgent.id,
         robot_.sfmAgent.position.getX(), robot_.sfmAgent.position.getY());

  robot_initialized_ = true;
  printf("SFM Robot initialized\n");
}

bool AgentManager::updateAgents(
    const hunav_msgs::msg::Agents::SharedPtr msg) {

  // update velocities
  // if (agents_initialized_ && time_step_secs_ > 0.0) {

  // Update agents velocites only in the ROS cycle
  for (auto a : msg->agents) {

    // velocities computed by myself
    // double anvel =
    //     normalizeAngle(a.yaw - agents_[a.id].sfmAgent.yaw.toRadian()) /
    //     (time_step_secs_); // * step_count);
    // agents_[a.id].sfmAgent.angularVelocity = anvel;
    // double xi = agents_[a.id].sfmAgent.position.getX();
    // double yi = agents_[a.id].sfmAgent.position.getY();
    // double xf = a.position.position.x;
    // double yf = a.position.position.y;
    // double dist = sqrt((xf - xi) * (xf - xi) + (yf - yi) * (yf - yi));
    // agents_[a.id].sfmAgent.linearVelocity =
    //     dist / (time_step_secs_);              // * step_count);
    // double vx = (xf - xi) / (time_step_secs_); // * step_count);
    // double vy = (yf - yi) / (time_step_secs_); // * step_count);
    // agents_[a.id].sfmAgent.velocity.set(vx, vy);

    // position
    agents_[a.id].sfmAgent.position.set(a.position.position.x,
                                        a.position.position.y);
    agents_[a.id].sfmAgent.yaw.setRadian(a.yaw);

    // velocities
    agents_[a.id].sfmAgent.velocity.set(a.velocity.linear.x,
                                        a.velocity.linear.y);
    agents_[a.id].sfmAgent.linearVelocity =
        sqrt(a.velocity.linear.x * a.velocity.linear.x +
             a.velocity.linear.y * a.velocity.linear.y);
    agents_[a.id].sfmAgent.angularVelocity = a.velocity.angular.z;

    // update goals
    // agents_[a.id].sfmAgent.goals.clear();
    // for (auto g : a.goals) {
    //   sfm::Goal sfmg;
    //   sfmg.center.setX(g.position.x);
    //   sfmg.center.setY(g.position.y);
    //   sfmg.radius = a.goal_radius;
    //   agents_[a.id].sfmAgent.goals.push_back(sfmg);
    // }

    // update closest obstacles
    agents_[a.id].sfmAgent.obstacles1.clear();
    if (!a.closest_obs.empty()) {
      for (auto obs : a.closest_obs) {
        utils::Vector2d o;
        o.set(obs.x, obs.y);
        agents_[a.id].sfmAgent.obstacles1.push_back(o);
      }
    }

    // printf("%s updating agents vels computed lv:%.3f, lvx:%.3f, lvy:%.3f, "
    //        "av:%.3f\n",
    //        a.name.c_str(), agents_[a.id].sfmAgent.linearVelocity,
    //        agents_[a.id].sfmAgent.velocity.getX(),
    //        agents_[a.id].sfmAgent.velocity.getY(),
    //        agents_[a.id].sfmAgent.angularVelocity);
  }
  step_count = 1;
  return true;

  // } else {
  //   step_count++;
  //   return false;
  // }
}

void AgentManager::updateAgentRobot(
    const hunav_msgs::msg::Agent::SharedPtr msg) {
  // Update robot
  robot_.sfmAgent.position.set(msg->position.position.x,
                               msg->position.position.y);
  // tf2::Quaternion q(
  //     robot_.position.orientation.x, robot_.position.orientation.y,
  //     robot_.position.orientation.z, robot_.position.orientation.w);
  // tf2::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  robot_.sfmAgent.yaw.setRadian(msg->yaw);
  robot_.sfmAgent.velocity.set(msg->velocity.linear.x, msg->velocity.linear.y);
  robot_.sfmAgent.linearVelocity =
      sqrt(msg->velocity.linear.x * msg->velocity.linear.x +
           msg->velocity.linear.y * msg->velocity.linear.y);
  robot_.sfmAgent.angularVelocity = msg->velocity.angular.z;
}

// void AgentManager::robotCallback(
//     const hunav_msgs::msg::Agent::SharedPtr msg) const {
//   robot_ = *msg;
//   if (!robot_initialized_) {
//     initializeSFMRobot();
//     robot_initialized_ = true;
//   } else {
//     updateSFMRobot();
//   }
// }

hunav_msgs::msg::Agent AgentManager::getUpdatedAgentMsg(int id) {
  // std::lock_guard<std::mutex> guard(mutex_);
  hunav_msgs::msg::Agent a;
  a.id = agents_[id].sfmAgent.id;
  a.name = agents_[id].name;
  a.type = agents_[id].type;
  a.behavior = agents_[id].behavior;
  a.position.position.x = agents_[id].sfmAgent.position.getX();
  a.position.position.y = agents_[id].sfmAgent.position.getY();
  a.yaw = agents_[id].sfmAgent.yaw.toRadian();
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, agents_[id].sfmAgent.yaw.toRadian());
  a.position.orientation = tf2::toMsg(myQuaternion);
  a.linear_vel = agents_[id].sfmAgent.linearVelocity;
  a.angular_vel = agents_[id].sfmAgent.angularVelocity;
  a.velocity.linear.x = agents_[id].sfmAgent.velocity.getX();
  a.velocity.linear.y = agents_[id].sfmAgent.velocity.getY();
  a.velocity.angular.z = agents_[id].sfmAgent.angularVelocity;

  for (auto g : agents_[id].sfmAgent.goals) {
    geometry_msgs::msg::Pose p;
    p.position.x = g.center.getX();
    p.position.y = g.center.getY();
    a.goals.push_back(p);
  }

  //(agents[i].goals.front().center - agents[i].position).norm() <=
  //          agents[i].goals.front().radius)
  // robot_received_ = false;
  // agents_received_ = false;
  return a;
}

// create a agents msg from the sfm_agents_
hunav_msgs::msg::Agents AgentManager::getUpdatedAgentsMsg() {
  // Make a copy of agents_
  // Then update them with the sfm_agents_ data
  std::lock_guard<std::mutex> guard(mutex_);
  hunav_msgs::msg::Agents agents_msg;
  agents_msg.header = header_;

  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++) {

    hunav_msgs::msg::Agent a = getUpdatedAgentMsg(itr->first);
    agents_msg.agents.push_back(a);
  }
  robot_received_ = false;
  agents_received_ = false;
  return agents_msg;
}

std::vector<sfm::Agent> AgentManager::getSFMAgents() {

  std::vector<sfm::Agent> agents;
  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++) {
    agents.push_back(itr->second.sfmAgent);
  }
  return agents;
}

void AgentManager::computeForces(int id) {

  std::vector<sfm::Agent> otherAgents = getSFMAgents();

  utils::Vector2d ob;
  switch (agents_[id].behavior) {
  case hunav_msgs::msg::Agent::BEH_REGULAR:
    // We add the robot as another human agent.
    otherAgents.push_back(robot_.sfmAgent);
    sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
    break;
  case hunav_msgs::msg::Agent::BEH_IMPASSIVE:
    // the human treats the robot like an obstacle.
    // We add the robot to the obstacles of this agent.
    ob.set(robot_.sfmAgent.position.getX(), robot_.sfmAgent.position.getY());
    agents_[id].sfmAgent.obstacles1.push_back(ob);
    sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
    break;
  default:
    // Compute forces as usual (not taking into account the robot)
    sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
  }

  bool compute = false;
  if (isnan(agents_[id].sfmAgent.forces.desiredForce.norm())) {
    printf("[AgentManager.ComputeForces] \tgoal force is nan. Using zero.. \n");
    agents_[id].sfmAgent.forces.desiredForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.groupForce.norm())) {
    printf("[AgentManager.ComputeForces] \tgroup force is nan. Using zero.. "
           "\n");
    agents_[id].sfmAgent.forces.groupForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.obstacleForce.norm())) {
    printf("[AgentManager.ComputeForces] \tobstacle force is nan. Using "
           "zero.. \n");
    agents_[id].sfmAgent.forces.obstacleForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.socialForce.norm())) {
    printf("[AgentManager.ComputeForces] \tsocial force is nan. Using "
           "zero.. \n");
    agents_[id].sfmAgent.forces.socialForce.set(0, 0);
    compute = true;
  }
  if (compute) {
    agents_[id].sfmAgent.forces.globalForce =
        agents_[id].sfmAgent.forces.desiredForce +
        agents_[id].sfmAgent.forces.socialForce +
        agents_[id].sfmAgent.forces.obstacleForce +
        agents_[id].sfmAgent.forces.groupForce;
  }
}

void AgentManager::computeForces() {

  std::vector<sfm::Agent> otherAgents = getSFMAgents();

  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++) {
    // printf("[AgentManager.ComputeForces] Agent %s, x:%.2f, y:%.2f
    // dvel:%.2f\n",
    //        itr->second.name.c_str(), itr->second.sfmAgent.position.getX(),
    //        itr->second.sfmAgent.position.getY(),
    //        itr->second.sfmAgent.desiredVelocity);

    computeForces(itr->second.sfmAgent.id);

    // printf("[AgentManager.ComputeForces] \tForces. global:%.4f, goal:%.4f "
    //        "soc:%.4f, "
    //        "obs:%.4f \n",
    //        itr->second.sfmAgent.forces.globalForce.norm(),
    //        itr->second.sfmAgent.forces.desiredForce.norm(),
    //        itr->second.sfmAgent.forces.socialForce.norm(),
    //        itr->second.sfmAgent.forces.obstacleForce.norm());
  }
}

void AgentManager::updateAllAgents(
    const hunav_msgs::msg::Agent::SharedPtr robot_msg,
    const hunav_msgs::msg::Agents::SharedPtr agents_msg) {

  std::lock_guard<std::mutex> guard(mutex_);

  // printf("[AgentManager.updateAllAgents] Receiving agents from
  // simulator...\n"); for (auto &a : agents_msg->agents) {
  //   printf("[AgentManager.updateAllAgents]\tagent %s\n", a.name.c_str());
  // }

  header_ = agents_msg->header;

  if (!robot_initialized_) {
    initializeRobot(robot_msg);
  }

  if (!agents_initialized_) {
    // initializeBehaviorTree();
    initializeAgents(agents_msg);
    // prev_time_ = agents_msg->header.stamp;
    // time_step_secs_ = 0.0;
    // computeForces();

  } else if (robot_initialized_ && agents_initialized_) {
    // time_step_secs_ =
    //     (rclcpp::Time(agents_msg->header.stamp) - prev_time_).seconds();
    // printf("AgentManager. time_step_secs_: %.5f\n", time_step_secs_);
    // if (time_step_secs_ < 0.009) {
    //   return;
    // }
    // prev_time_ = agents_msg->header.stamp;
    updateAgentRobot(robot_msg);
    move = updateAgents(agents_msg);
  }
  agents_received_ = true;
  robot_received_ = true;
  computeForces();
}

void AgentManager::updateAgentsAndRobot(
    const hunav_msgs::msg::Agents::SharedPtr agents_msg) {

  std::lock_guard<std::mutex> guard(mutex_);

  // printf("[AgentManager.updateAgentsAndRobot] Receiving agents from "
  //        "simulator...\n");

  header_ = agents_msg->header;

  // The robot is the last agent of the vector!
  // or we could look for the type "robot" in the vector
  hunav_msgs::msg::Agent::SharedPtr rob =
      std::make_shared<hunav_msgs::msg::Agent>(agents_msg->agents.back());

  // we remove the robot from the agents vector
  hunav_msgs::msg::Agents ags = *agents_msg;
  ags.agents.pop_back();

  if (!robot_initialized_) {
    initializeRobot(rob);
  }
  if (!agents_initialized_) {

    initializeAgents(std::make_shared<hunav_msgs::msg::Agents>(ags));
    // prev_time_ = agents_msg->header.stamp;
    // time_step_secs_ = 0.0;

  } else {
    // // time_step_secs_ = 0.001;
    // time_step_secs_ =
    //     (rclcpp::Time(agents_msg->header.stamp) - prev_time_).seconds();
    // if (time_step_secs_ < 0.009) {
    //   return;
    // }
    // prev_time_ = agents_msg->header.stamp;
    updateAgentRobot(rob);
    move = updateAgents(std::make_shared<hunav_msgs::msg::Agents>(ags));
  }

  agents_received_ = true;
  robot_received_ = true;
  computeForces();
}

bool AgentManager::canCompute() {
  if (agents_received_ && robot_received_)
    return true;
  else
    return false;
}

// bool AgentManager::running() {
//   printf("[agentManager.running] running called!\n");
//   std::lock_guard<std::mutex> guard(mutex_);
//   if (std::find(agent_status_.begin(), agent_status_.end(), false) ==
//       agent_status_.end()) {
//     agent_status_.clear();
//     agent_status_.assign(agents_.size(), false);
//     printf("[agentManager.running] return false - task completed\n");
//     return false;
//   } else {
//     printf("[agentManager.running] return true - task running\n");
//     return true;
//   }
// }

void AgentManager::updatePosition(int id, double dt) {

  std::lock_guard<std::mutex> guard(mutex_);

  // double newyaw = atan2(agents_[id].sfmAgent.forces.globalForce.getY(),
  //                      agents_[id].sfmAgent.forces.globalForce.getX());
  // agents_[id].sfmAgent.yaw.setRadian(newyaw);

  // if (!agents_initialized_ || dt < 0.008) {
  //   printf("[agentManager.updatePosition] NOT UPDATING agent id:%i,
  //   dt:%.3f\n",
  //          id, dt);
  //   return;
  // }

  // printf("[agentManager.updatePosition] UPDATING agent id:%i, dt:%.3f\n", id,
  //        dt);

  // printf(
  //     "[agentManager.updatePosition] %s time:%.5f PREVpos x:%.3f, "
  //     "y:%.3f, th:%.3f, lv:%.3f, av:%.3f, velx:%.3f, vely:%.3f\n",
  //     agents_[id].name.c_str(), dt, agents_[id].sfmAgent.position.getX(),
  //     agents_[id].sfmAgent.position.getY(),
  //     agents_[id].sfmAgent.yaw.toRadian(),
  //     agents_[id].sfmAgent.linearVelocity,
  //     agents_[id].sfmAgent.angularVelocity,
  //     agents_[id].sfmAgent.velocity.getX(),
  //     agents_[id].sfmAgent.velocity.getY());

  // printf("[AgentManager.UpdatePosition]\t %s Forces. global:%.4f, goal:%.4f
  // "
  //        "soc:%.4f, "
  //        "obs:%.4f \n",
  //        agents_[id].name.c_str(),
  //        agents_[id].sfmAgent.forces.globalForce.norm(),
  //        agents_[id].sfmAgent.forces.desiredForce.norm(),
  //        agents_[id].sfmAgent.forces.socialForce.norm(),
  //        agents_[id].sfmAgent.forces.obstacleForce.norm());

  sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
  // step_count2 = 1;
  // double newyaw = atan2(agents_[id].sfmAgent.forces.globalForce.getY(),
  //                       agents_[id].sfmAgent.forces.globalForce.getX());
  // agents_[id].sfmAgent.yaw.setRadian(newyaw);

  // printf(
  //     "[agentManager.updatePosition] %s NEWpos x:%.3f, y:%.3f, "
  //     "th:%.3f, lv:%.3f, av:%.3f, velx: %.3f, vely:%.3f\n",
  //     agents_[id].name.c_str(), agents_[id].sfmAgent.position.getX(),
  //     agents_[id].sfmAgent.position.getY(),
  //     agents_[id].sfmAgent.yaw.toRadian(),
  //     agents_[id].sfmAgent.linearVelocity,
  //     agents_[id].sfmAgent.angularVelocity,
  //     agents_[id].sfmAgent.velocity.getX(),
  //     agents_[id].sfmAgent.velocity.getY());

  // if (!agents_[id].sfmAgent.goals.empty()) {
  //   printf("[agentManager.updatePosition] id:%i Current goal x:%.2f, y:
  //   %.2f\n",
  //          id, agents_[id].sfmAgent.goals.front().center.getX(),
  //          agents_[id].sfmAgent.goals.front().center.getY());
  // }
}

} // namespace hunav_agent_manager
