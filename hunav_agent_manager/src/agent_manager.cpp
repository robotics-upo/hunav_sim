#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"

namespace hunav
{

// using std::placeholders::_1;
// using std::placeholders::_2;

AgentManager::AgentManager()
{  //: initialized_(false), max_dist_view_(10.0) {

  init();
  //printf("[AgentManager.Constructor] AgentManager initialized \n");
}

AgentManager::~AgentManager()
{
}

void AgentManager::init()
{
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
  //printf("[AgentManager.init] initialized \n");
}

float AgentManager::robotSquaredDistance(int id)
{
  // std::lock_guard<std::mutex> guard(mutex_);
  // mutex_.lock();

  // hunav_msgs/msg/Agents
  float xa = agents_[id].sfmAgent.position.getX();  // position.position.x;
  float ya = agents_[id].sfmAgent.position.getY();  // position.y;
  float xr = robot_.sfmAgent.position.getX();       // position.x;
  float yr = robot_.sfmAgent.position.getY();       // position.y;

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

bool AgentManager::lineOfSight(int id)
{
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

  if (abs(rangle) > (M_PI / 2.0 + 0.17))
  {
    // std::cout << "[AgentManager.lineOfSight] Agent " << id + 1
    //           << " can NOT see the robot!" << std::endl;
    return false;
  }
  else
  {
    // std::cout << "[AgentManager.lineOfSight] Agent " << id + 1
    //           << " is seeing the robot!" << std::endl;

    // ----------------------------------------
    // TODO: do raytracing to check visibility
    // ----------------------------------------

    return true;
  }
}

bool AgentManager::isRobotVisible(int id, double dist)
{
  std::lock_guard<std::mutex> guard(mutex_);
  float squared_dist = robotSquaredDistance(id);
  if (squared_dist <= (dist * dist))
  {
    return lineOfSight(id);
  }
  else
  {
    return false;
  }
}

// void AgentManager::isRobotVisible(
//     const std::shared_ptr<hunav_msgs::srv::IsRobotVisible::Request>
//     request, std::shared_ptr<hunav_msgs::srv::IsRobotVisible::Response>
//     response) {
//   response->visible = isRobotVisible(request->agent_id)
// }

void AgentManager::lookAtTheRobot(int id)
{
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
  utils::Angle robotYaw;  // = utils::Angle::fromRadian(atan2(nry, nrx));
  robotYaw.setRadian(atan2(nry, nrx));

  // Change the angle step by step according to
  // the time_step_secs_ and a maximum angular vel
  float max_ang_vel = M_PI;                                               // rad/secs
  utils::Angle max_angle = utils::Angle::fromRadian(max_ang_vel * 0.2);  // time_step_secs_);
  if (robotYaw.sign() < 0)
    max_angle.setRadian(max_angle.toRadian() * (-1));

  // Update the agent angle
  if (fabs(robotYaw.toRadian()) > max_angle.toRadian())
  {
    agents_[id].sfmAgent.yaw = (agents_[id].sfmAgent.yaw + max_angle);
  }
  else
  {
    agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;
  }
  agents_[id].behavior.state = 1;
}

void AgentManager::lookAtAgent(int observer_id, int target_id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  
  // Get target agent's position
  float tx = agents_[target_id].sfmAgent.position.getX();
  float ty = agents_[target_id].sfmAgent.position.getY();
  
  // Get observer agent's position and current yaw
  float ox = agents_[observer_id].sfmAgent.position.getX();
  float oy = agents_[observer_id].sfmAgent.position.getY();
  float oh = agents_[observer_id].sfmAgent.yaw.toRadian();
  
  // Transform target position into the observer's coordinate system
  float nrx = (tx - ox) * cos(oh) + (ty - oy) * sin(oh);
  float nry = -(tx - ox) * sin(oh) + (ty - oy) * cos(oh);
  utils::Angle targetAngle;
  targetAngle.setRadian(atan2(nry, nrx));
  
  // Use a fixed time-step (0.01 seconds) and maximum angular velocity (π rad/s)
  float max_ang_vel = M_PI; // rad/s
  utils::Angle max_angle = utils::Angle::fromRadian(max_ang_vel * 0.2);
  if (targetAngle.sign() < 0)
    max_angle.setRadian(max_angle.toRadian() * (-1));

  // Incrementally adjust the observer's yaw
  if (fabs(targetAngle.toRadian()) > max_angle.toRadian())
  {
    agents_[observer_id].sfmAgent.yaw = agents_[observer_id].sfmAgent.yaw + max_angle;
  }
  else
  {
    agents_[observer_id].sfmAgent.yaw = agents_[observer_id].sfmAgent.yaw + targetAngle;
  }
  agents_[observer_id].behavior.state = 1;
}

void AgentManager::lookAtPoint(int agent_id, const utils::Vector2d & target)
{
    std::lock_guard<std::mutex> guard(mutex_);
    
    // Get the agent's current position
    double ax = agents_[agent_id].sfmAgent.position.getX();
    double ay = agents_[agent_id].sfmAgent.position.getY();
    
    // Compute the desired angle from the agent to the target
    double desiredAngle = std::atan2(target.getY() - ay, target.getX() - ax);
    
    // Get the agent's current yaw
    double currentYaw = agents_[agent_id].sfmAgent.yaw.toRadian();
    
    // Compute the angular error using std::remainder to normalize it to [-pi, pi]
    double error = std::remainder(desiredAngle - currentYaw, 2 * M_PI);
    
    // Define maximum angular change per update)
    double maxAngularChange = M_PI * 0.2; 
    
    // Clamp the angular change
    double deltaYaw = (std::fabs(error) > maxAngularChange) ?
                      (error > 0 ? maxAngularChange : -maxAngularChange) :
                      error;
    
    // Update the agent's yaw
    double newYaw = currentYaw + deltaYaw;
    agents_[agent_id].sfmAgent.yaw.setRadian(newYaw);
    
    agents_[agent_id].behavior.state = 1;
    
    // printf("lookAtPoint: Agent %i: currentYaw=%.2f, desiredAngle=%.2f, error=%.2f, newYaw=%.2f\n",
    //        agent_id, currentYaw, desiredAngle, error, newYaw);
}

void AgentManager::approximateRobot(int id, double dt, double closest_dist, double max_vel)
{
  std::lock_guard<std::mutex> guard(mutex_);

  agents_[id].behavior.state = 1;

  // Get the robot's position and velocity
  double rx = robot_.sfmAgent.position.getX();
  double ry = robot_.sfmAgent.position.getY();
  //float robot_speed = robot_.sfmAgent.linearVelocity;

  // Compute difference and Euclidean distance
  double ax = agents_[id].sfmAgent.position.getX();
  double ay = agents_[id].sfmAgent.position.getY();
  double dx = rx - ax;
  double dy = ry - ay;
  double dist = std::hypot(dx, dy);

  // Check if robot is moving towards the agent (collision risk)
  // bool robot_is_moving = (robot_speed > 0.1); // 0.1 m/s threshold
  // bool collision_risk = false;
  
  // if (robot_is_moving && dist < closest_dist * 2.0) { // Within twice the desired distance
  //   // Calculate robot's actual movement direction based on velocity, not yaw
  //   float robot_vx = robot_.sfmAgent.velocity.getX();
  //   float robot_vy = robot_.sfmAgent.velocity.getY();
  //   float actual_robot_speed = std::hypot(robot_vx, robot_vy);
    
  //   // Only check collision if robot is actually translating, not just rotating
  //   if (actual_robot_speed > 0.1) {
  //     // Vector from robot to agent
  //     float dx_robot_to_agent = ax - rx;
  //     float dy_robot_to_agent = ay - ry;
      
  //     // Normalize vectors
  //     float agent_dir_norm = std::hypot(dx_robot_to_agent, dy_robot_to_agent);
      
  //     if (actual_robot_speed > 0.001 && agent_dir_norm > 0.001) {
  //       // Check if robot's actual velocity is towards agent (dot product > 0.5 means angle < 60°)
  //       float dot_product = (robot_vx * dx_robot_to_agent + robot_vy * dy_robot_to_agent) / 
  //                          (actual_robot_speed * agent_dir_norm);
  //       collision_risk = (dot_product > 0.5);
  //     }
  //   }
  // }

  // printf("Agent %i approaching robot: distance=%.2f, closest_dist=%.2f, robot_linear_vel=%.2f, robot_actual_vel=%.2f, collision_risk=%s\n", 
  //        id, dist, closest_dist, robot_speed, 
  //        robot_is_moving ? std::hypot(robot_.sfmAgent.velocity.getX(), robot_.sfmAgent.velocity.getY()) : 0.0,
  //        collision_risk ? "true" : "false");

  // // Case 1: Collision risk - move away to avoid collision
  // if (collision_risk)
  // {
  //   printf("Agent %i: Collision risk detected, moving away temporarily\n", id);
    
  //   // Calculate direction away from robot
  //   double escape_dx = ax - rx;
  //   double escape_dy = ay - ry;
  //   double escape_norm = std::hypot(escape_dx, escape_dy);
    
  //   if (escape_norm > 0.001) {
  //     // Normalize escape direction
  //     escape_dx /= escape_norm;
  //     escape_dy /= escape_norm;
      
  //     // Calculate safe position (further than closest_dist)
  //     double safe_distance = closest_dist * 1.5;
  //     double safe_x = rx + safe_distance * escape_dx;
  //     double safe_y = ry + safe_distance * escape_dy;
      
  //     // Store original goals and create temporary escape goal
  //     std::list<sfm::Goal> original_goals = agents_[id].sfmAgent.goals;
      
  //     sfm::Goal escape_goal;
  //     escape_goal.center.set(safe_x, safe_y);
  //     escape_goal.radius = 0.3;
  //     agents_[id].sfmAgent.goals.push_front(escape_goal);
      
  //     // Use higher velocity for escape
  //     double original_vel = agents_[id].sfmAgent.desiredVelocity;
  //     agents_[id].sfmAgent.desiredVelocity = static_cast<float>(max_vel * 1.2);
      
  //     // Update position
  //     computeForces(id);
  //     sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
      
  //     // Restore original goals and velocity
  //     agents_[id].sfmAgent.goals = original_goals;
  //     agents_[id].sfmAgent.desiredVelocity = static_cast<float>(original_vel);
      
  //     printf("Agent %i escaped to safe position: (%.2f, %.2f)\n", id, safe_x, safe_y);
  //   }
  // }
  // Case 2: At desired distance - look at robot and stay
  // else 
  if (dist <= closest_dist)
  {
      // When the agent is close enough, update its orientation to face the robot
      double desiredAngle = std::atan2(dy, dx);
      double currentYaw = agents_[id].sfmAgent.yaw.toRadian();
      double error = std::remainder(desiredAngle - currentYaw, 2 * M_PI);
      // Clamp the angular change for smoothness
      double maxAngularChange = M_PI * 0.01;
      double deltaYaw = (std::fabs(error) > maxAngularChange) ?
                          (error > 0 ? maxAngularChange : -maxAngularChange) :
                          error;
      double newYaw = currentYaw + deltaYaw;
      agents_[id].sfmAgent.yaw.setRadian(newYaw);
      
      printf("Agent %i at desired distance, looking at robot (angle: %.2f)\n", id, newYaw);
  }
  // Case 3: Too far - approach robot
  else
  {
      // When the agent is not close enough, set a temporary goal at the robot's position
      sfm::Goal g;
      g.center.set(rx, ry);
      g.radius = robot_.sfmAgent.radius;
      agents_[id].sfmAgent.goals.push_front(g);

      double d0 = closest_dist;
      double scaled_vel = max_vel * (dist / (dist + d0));
      double original_vel = agents_[id].sfmAgent.desiredVelocity;
      agents_[id].sfmAgent.desiredVelocity = static_cast<float>(scaled_vel);

      // Recompute forces and update the agent's position
      //double prev_yaw = agents_[id].sfmAgent.yaw.toRadian();
      computeForces(id);
      sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
      // if(agents_[id].sfmAgent.linearVelocity < 0.05)
      // {
      //   agents_[id].sfmAgent.yaw.setRadian(prev_yaw);
      // } 

      // Remove the temporary goal and restore the original velocity
      agents_[id].sfmAgent.goals.pop_front();
      agents_[id].sfmAgent.desiredVelocity = static_cast<float>(original_vel);
      
      printf("Agent %i approaching robot: scaled_vel=%.2f\n", id, scaled_vel);
  }
}

void AgentManager::blockRobot(int id, double dt, double front_dist)
{
  std::lock_guard<std::mutex> guard(mutex_);

  agents_[id].behavior.state = 1;

  // Get robot position, heading, and velocity
  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();
  float robot_yaw = robot_.sfmAgent.yaw.toRadian();
  float robot_speed = robot_.sfmAgent.linearVelocity;

  // Get current agent position
  float ax = agents_[id].sfmAgent.position.getX();
  float ay = agents_[id].sfmAgent.position.getY();
  float distance_to_robot = std::hypot(rx - ax, ry - ay);

  // Calculate the ideal blocking position (in front of robot)
  float gdist = front_dist>0.5 ? front_dist : 1.5;
  float ideal_x = rx + gdist * cos(robot_yaw);
  float ideal_y = ry + gdist * sin(robot_yaw);
  float distance_to_ideal = std::hypot(ideal_x - ax, ideal_y - ay);

  // Check if robot is moving (threshold for "stationary")
  bool robot_is_moving = (robot_speed > 0.1); // 0.1 m/s threshold
  
  // Calculate if agent is approximately in the blocking position
  bool in_blocking_position = (distance_to_ideal <= 0.8) && (distance_to_robot <= (front_dist + 0.8));
  
  // printf("Agent %i blocking robot: robot_yaw=%.2f, robot_speed=%.2f, ideal_pos=(%.2f,%.2f), agent_pos=(%.2f,%.2f), dist_to_ideal=%.2f, in_position=%s, robot_moving=%s\n", 
  //        id, robot_yaw, robot_speed, ideal_x, ideal_y, ax, ay, distance_to_ideal, 
  //        in_blocking_position ? "true" : "false", robot_is_moving ? "true" : "false");

  // Case 1: Agent is in blocking position AND robot is stationary - just look at robot
  if (in_blocking_position && !robot_is_moving)  
  {
    // Calculate the angle from agent to robot (safe custom calculation)
    float angle_to_robot = std::atan2(ry - ay, rx - ax);
    agents_[id].sfmAgent.yaw.setRadian(angle_to_robot);
    
    // Stop the agent by setting velocity to zero
    float original_vel = agents_[id].sfmAgent.desiredVelocity;
    agents_[id].sfmAgent.desiredVelocity = 0.0;
    
    // Compute forces with zero velocity (agent stays in place)
    computeForces(id);
    // Don't update position - agent should stay where it is
    
    // Restore original velocity for next iteration
    agents_[id].sfmAgent.desiredVelocity = original_vel;
    
    printf("Agent %i in blocking position with stationary robot, looking at robot (angle: %.2f)\n", 
           id, angle_to_robot);
  }
  // Case 2: Agent is in blocking position BUT robot is moving - move away to maintain distance
  else if (in_blocking_position && robot_is_moving)
  {
    // Robot is moving toward agent - agent should back away to maintain blocking distance
    
    // // Store the original goals
    // std::list<sfm::Goal> original_goals = agents_[id].sfmAgent.goals;

    // // Calculate a position further away in the same direction (backing away)
    // float backup_distance = gdist + 0.5; // A bit further away
    // float backup_x = rx + backup_distance * cos(robot_yaw);
    // float backup_y = ry + backup_distance * sin(robot_yaw);
    
    // // Create backing away goal
    // sfm::Goal backup_goal;
    // backup_goal.center.set(backup_x, backup_y);
    // backup_goal.radius = 0.2;
    // agents_[id].sfmAgent.goals.push_front(backup_goal);

    // // Use moderate velocity for backing away
    // float original_vel = agents_[id].sfmAgent.desiredVelocity;
    // agents_[id].sfmAgent.desiredVelocity = 1.5; // Moderate speed for repositioning

    // // Compute forces and update position (agent backs away)
    // computeForces(id);
    // sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
    // // Noé test ------------------------
    // float angle_to_robot = std::atan2(ry - ay, rx - ax);
    // agents_[id].sfmAgent.yaw.setRadian(angle_to_robot);
    // //-------------------------------------

    // // Restore original goals and velocity
    // agents_[id].sfmAgent.goals = original_goals;
    // agents_[id].sfmAgent.desiredVelocity = original_vel;
    
    // printf("Agent %i backing away from moving robot: backup_pos=(%.2f,%.2f)\n", 
    //         id, backup_x, backup_y);
  }
  // Case 3: Agent is not in blocking position - move to ideal position
  else
  {
    // Agent needs to move to the blocking position in front of the robot
    
    // Store the original goals
    std::list<sfm::Goal> original_goals = agents_[id].sfmAgent.goals;

    // Create blocking goal at the ideal position in front of robot
    sfm::Goal blocking_goal;
    blocking_goal.center.set(ideal_x, ideal_y);
    blocking_goal.radius = 0.2;
    agents_[id].sfmAgent.goals.push_front(blocking_goal);

    // Store original velocity and increase it for faster approach
    float original_vel = agents_[id].sfmAgent.desiredVelocity;
    agents_[id].sfmAgent.desiredVelocity = 2.5; // Higher velocity for quicker positioning

    // Compute forces and update position (agent moves toward blocking position)
    computeForces(id);
    sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

    // Restore original goals and velocity
    agents_[id].sfmAgent.goals = original_goals;
    agents_[id].sfmAgent.desiredVelocity = original_vel;
    
    printf("Agent %i moving to blocking position: target=(%.2f,%.2f), current=(%.2f,%.2f)\n", 
           id, ideal_x, ideal_y, ax, ay);
  }
}

void AgentManager::blockAgent(int id, int target_id, double dt, double front_dist)
{
    std::lock_guard<std::mutex> guard(mutex_);

    // Mark that the agent is executing a block behavior
    agents_[id].behavior.state = 1;

    // Retrieve the target agent's position and heading
    float tx = agents_[target_id].sfmAgent.position.getX();
    float ty = agents_[target_id].sfmAgent.position.getY();
    float th = agents_[target_id].sfmAgent.yaw.toRadian();

    // Save the current set of goals for the approaching agent
    std::list<sfm::Goal> original_goals = agents_[id].sfmAgent.goals;

    float newgx = tx + front_dist * sin(th);
    float newgy = ty + front_dist * cos(th);
    sfm::Goal blocking_goal;
    blocking_goal.center.set(newgx, newgy);
    blocking_goal.radius = 0.05;
    // Insert the temporary blocking goal at the front
    agents_[id].sfmAgent.goals.push_front(blocking_goal);

    // Temporarily override the agent's desired velocity
    float original_vel = agents_[id].sfmAgent.desiredVelocity;
    agents_[id].sfmAgent.desiredVelocity = 2.0;

    // Recompute forces and update the agent's position
    computeForces(id);
    sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

    // Now check if the agent is close enough to the target
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
    float dist = std::hypot(tx - ax, ty - ay);
    
    if (dist <= 0.6)
    {
        // When close, update orientation to face the target agent
        float currentYaw = agents_[id].sfmAgent.yaw.toRadian();
        // Transform the target agent's position into the approaching agent's coordinate system
        float nrx = (tx - ax) * cos(currentYaw) + (ty - ay) * sin(currentYaw);
        float nry = -(tx - ax) * sin(currentYaw) + (ty - ay) * cos(currentYaw);
        utils::Angle targetYaw;
        targetYaw.setRadian(atan2(nry, nrx));
        // Update the agent's yaw by adding the computed adjustment
        agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + targetYaw;
    }

    // Restore the original goals and desired velocity
    agents_[id].sfmAgent.goals = original_goals;
    agents_[id].sfmAgent.desiredVelocity = original_vel;
}


void AgentManager::avoidRobot(int id, double dt, double safe_distance, double max_vel)
{
  std::lock_guard<std::mutex> guard(mutex_);

  agents_[id].behavior.state = 1;

  // Get robot position and velocity
  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();
  //float robot_vx = robot_.sfmAgent.velocity.getX();
  //float robot_vy = robot_.sfmAgent.velocity.getY();

  // Get current agent position
  float ax = agents_[id].sfmAgent.position.getX();
  float ay = agents_[id].sfmAgent.position.getY();
  float distance_to_robot = std::hypot(rx - ax, ry - ay);

  // Calculate if agent is at a safe distance
  bool at_safe_distance = (distance_to_robot >= safe_distance);
  
  // Track if agent has ever reached safe distance and previous distances
  static std::map<int, bool> agent_reached_safe_distance;
  //static std::map<int, float> previous_distances;
  
  // Check if this is the first time at safe distance
  if (at_safe_distance) {
    if (agent_reached_safe_distance.find(id) == agent_reached_safe_distance.end()) {
      agent_reached_safe_distance[id] = true;
      printf("Agent %i: Reached safe distance for first time\n", id);
    }
  }
  
  // Detect if robot is approaching - based on movement direction and getting closer
  //bool robot_approaching = false;
  //bool distance_decreasing = false;
  
  // // Check if distance is decreasing
  // if (previous_distances.find(id) != previous_distances.end()) {
  //   float prev_distance = previous_distances[id];
  //   distance_decreasing = (distance_to_robot < prev_distance - 0.05); // Robot got 5cm closer
  // }
  //previous_distances[id] = distance_to_robot;
  
  // Check if robot's movement direction is towards the agent
  // bool moving_towards_agent = false;
  // float robot_speed = std::hypot(robot_vx, robot_vy);
  // if (robot_speed > 0.1) { // Robot is moving
  //   // Vector from robot to agent
  //   float dx_robot_to_agent = ax - rx;
  //   float dy_robot_to_agent = ay - ry;
    
  //   // Normalize robot velocity and robot-to-agent vector
  //   float robot_vel_norm = std::hypot(robot_vx, robot_vy);
  //   float agent_dir_norm = std::hypot(dx_robot_to_agent, dy_robot_to_agent);
    
  //   if (robot_vel_norm > 0.001 && agent_dir_norm > 0.001) {
  //     // Calculate dot product to check if robot is moving towards agent
  //     float dot_product = (robot_vx * dx_robot_to_agent + robot_vy * dy_robot_to_agent) / 
  //                        (robot_vel_norm * agent_dir_norm);
      
  //     // If dot product > 0.5, robot is moving towards agent (angle < 60 degrees)
  //     moving_towards_agent = (dot_product > 0.5);
  //   }
  // }
  
  // Robot is approaching if BOTH conditions are true:
  // 1. Distance is decreasing AND 2. Robot is moving towards agent
  //robot_approaching = distance_decreasing && moving_towards_agent;
  
  // printf("Agent %i avoiding robot: distance=%.2f, safe_distance=%.2f, at_safe=%s, distance_decreasing=%s, moving_towards=%s, robot_approaching=%s\n", 
  //        id, distance_to_robot, safe_distance, 
  //        at_safe_distance ? "true" : "false", distance_decreasing ? "true" : "false",
  //        moving_towards_agent ? "true" : "false", robot_approaching ? "true" : "false");

  // Case 1: At safe distance - just look at robot and stay still
  if (at_safe_distance && 
      agent_reached_safe_distance.find(id) != agent_reached_safe_distance.end() && 
      agent_reached_safe_distance[id])
  {
    // Calculate the angle from agent to robot (to look at it)
    float angle_to_robot = std::atan2(ry - ay, rx - ax);
    agents_[id].sfmAgent.yaw.setRadian(angle_to_robot);
    
    // Stop the agent by setting velocity to zero
    float original_vel = agents_[id].sfmAgent.desiredVelocity;
    agents_[id].sfmAgent.desiredVelocity = 0.0;
    
    // Compute forces with zero velocity (agent stays in place)
    computeForces(id);
    // Don't update position - agent should stay where it is
    
    // Restore original velocity for next iteration
    agents_[id].sfmAgent.desiredVelocity = original_vel;
    
    // printf("Agent %i at safe distance, looking at robot and staying still (angle: %.2f, robot_approaching=%s)\n", 
    //        id, angle_to_robot, robot_approaching ? "true" : "false");
  }
  // Case 2: Need to move away - either never reached safe distance OR too close (regardless of robot approach)
  else if (agent_reached_safe_distance.find(id) == agent_reached_safe_distance.end() || 
           !at_safe_distance)
  {
    // printf("Agent %i: Moving away - distance=%.2f, at_safe=%s, never_reached_safe=%s\n", 
    //        id, distance_to_robot, at_safe_distance ? "true" : "false",
    //        (agent_reached_safe_distance.find(id) == agent_reached_safe_distance.end()) ? "true" : "false");
           
    // Calculate direction away from robot
    float dx = ax - rx; // Direction: away from robot
    float dy = ay - ry;
    
    // Normalize the direction vector
    if (distance_to_robot > 0.1) // Avoid division by zero
    {
      dx /= distance_to_robot;
      dy /= distance_to_robot;
    }
    else
    {
      // If too close, just move in current facing direction
      dx = cos(agents_[id].sfmAgent.yaw.toRadian());
      dy = sin(agents_[id].sfmAgent.yaw.toRadian());
    }
    
    // Calculate escape position - move away from robot to safe distance
    float escape_x = rx + safe_distance * dx;
    float escape_y = ry + safe_distance * dy;
    
    // Store the original goals
    std::list<sfm::Goal> original_goals = agents_[id].sfmAgent.goals;

    // Create escape goal away from robot
    sfm::Goal escape_goal;
    escape_goal.center.set(escape_x, escape_y);
    escape_goal.radius = 0.3; // Larger radius for easier reaching
    agents_[id].sfmAgent.goals.push_front(escape_goal);

    // Store original velocity and set escape velocity
    float original_vel = agents_[id].sfmAgent.desiredVelocity;
    
    agents_[id].sfmAgent.desiredVelocity = max_vel;

    // Compute forces and update position (agent moves away from robot)
    computeForces(id);
    sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

    // Restore original goals and velocity
    agents_[id].sfmAgent.goals = original_goals;
    agents_[id].sfmAgent.desiredVelocity = original_vel;
    
  }
}

int AgentManager::findNearestAgent(int id)
{
    std::lock_guard<std::mutex> guard(mutex_);
    int nearest_id = -1;
    double min_distance_sq = std::numeric_limits<double>::max();

    // Get the agent's current position
    float gx = agents_[id].sfmAgent.position.getX();
    float gy = agents_[id].sfmAgent.position.getY();

    // Iterate over all agents
    for (const auto &kv : agents_)
    {
        int candidate_id = kv.first;
        if (candidate_id == id)
            continue;  // Skip itself

        // Get candidate's position
        float cx = kv.second.sfmAgent.position.getX();
        float cy = kv.second.sfmAgent.position.getY();

        // Compute Euclidean distance
        float dx = cx - gx;
        float dy = cy - gy;
        double dist_sq = dx * dx + dy * dy;

        // Update the nearest candidate if this one is closer
        if (dist_sq < min_distance_sq)
        {
            min_distance_sq = dist_sq;
            nearest_id = candidate_id;
        }
    }
    if (nearest_id < 0)
    {
      printf("\n\n\t--> Nearest agent for greeter agent %i found with ID: %i (Distance: %f)\n\n", id, nearest_id, min_distance_sq);
    }
    else
    {
      printf("\n\n\t--> No agent near agent %i \n\n", id);
    }
    

    return nearest_id; 
}

std::vector<int> AgentManager::findNearestAgents(int id, int k)
{
    std::vector<std::pair<int, double>> distances;
    // Get the reference agent position
    double refX = agents_.at(id).sfmAgent.position.getX();
    double refY = agents_.at(id).sfmAgent.position.getY();

    for (const auto &kv : agents_)
    {
        int candidate_id = kv.first;
        if (candidate_id == id)
            continue; // Skip itself

        double cx = kv.second.sfmAgent.position.getX();
        double cy = kv.second.sfmAgent.position.getY();
        double dx = cx - refX;
        double dy = cy - refY;
        double dist_sq = dx * dx + dy * dy;
        distances.push_back({candidate_id, dist_sq});
    }

    // Sort by distance (ascending order)
    std::sort(distances.begin(), distances.end(),
              [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                  return a.second < b.second;
              });

    // Prepare the output vector (up to k agents)
    std::vector<int> nearest;
    for (size_t i = 0; i < distances.size() && i < static_cast<size_t>(k); i++)
    {
        nearest.push_back(distances[i].first);
    }
    return nearest;
}

bool AgentManager::isAgentClose(int observer_id, int target_id, double threshold)
{
    std::lock_guard<std::mutex> guard(mutex_);
    
    auto it_obs = agents_.find(observer_id);
    auto it_target = agents_.find(target_id);
    if (it_obs == agents_.end() || it_target == agents_.end())
    {
        return false;
    }
    
    // Get positions from the agents
    double ox = it_obs->second.sfmAgent.position.getX();
    double oy = it_obs->second.sfmAgent.position.getY();
    double tx = it_target->second.sfmAgent.position.getX();
    double ty = it_target->second.sfmAgent.position.getY();
    
    // Compute the Euclidean distance
    double dist = std::hypot(tx - ox, ty - oy);
    
    return (dist < threshold);
}


bool AgentManager::isAgentVisible(int observer_id, int target_id, double dist, double fov_threshold)
{
    std::lock_guard<std::mutex> guard(mutex_);
    
    // Get the observer's position
    float ox = agents_[observer_id].sfmAgent.position.getX();
    float oy = agents_[observer_id].sfmAgent.position.getY();
    
    // Get the target's position
    float tx = agents_[target_id].sfmAgent.position.getX();
    float ty = agents_[target_id].sfmAgent.position.getY();
    
    // Compute Euclidean distance
    float dx = tx - ox;
    float dy = ty - oy;
    float squared_dist = dx * dx + dy * dy;
    
    // If the target is farther than the specified distance, return false.
    if (squared_dist > (dist * dist))
        return false;
    
    // Compute the observer's heading
    double observer_yaw = agents_[observer_id].sfmAgent.yaw.toRadian();
    
    // Transform the target position into the observer's coordinate frame
    float nrx = (tx - ox) * cos(observer_yaw) + (ty - oy) * sin(observer_yaw);
    float nry = -(tx - ox) * sin(observer_yaw) + (ty - oy) * cos(observer_yaw);
    
    // Calculate the relative angle from the observer to the target
    float relative_angle = atan2(nry, nrx); 
    
    // if (fabs(relative_angle) <= fov_threshold)
    //   printf("\n\n\t---> Agent %i is seeing agent %i\n\n", observer_id, target_id);
    // If the absolute value of the relative angle is within the threshold, the target is visible
    return (fabs(relative_angle) <= fov_threshold); 
}

void AgentManager::approachAgent(int id, int target_id, double dt, double closest_dist, double max_vel)
{
    std::lock_guard<std::mutex> guard(mutex_);
  
    agents_[id].behavior.state = 1;

    // Get the target and agent positions.
    float tx = agents_[target_id].sfmAgent.position.getX();
    float ty = agents_[target_id].sfmAgent.position.getY();
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
  
    // Compute the difference and Euclidean distance.
    float dx = tx - ax;
    float dy = ty - ay;
    float dist = std::hypot(dx, dy);
  
    // printf("\n\n\t---> Distance between Agent (ID: %i) and Target agent (ID: %i) = %.2f\n\n", 
    //       id, target_id, dist);

    if (dist <= closest_dist)
    {
        // printf("\n\n\t---> Agent %i is close enough to agent %i. Updating its orientation to face the target.\n\n", 
        //       id, target_id);
        // When close, update orientation so the agent faces the target.
        float desiredAngle = std::atan2(dy, dx);
        float currentYaw = agents_[id].sfmAgent.yaw.toRadian();
        float error = std::remainder(desiredAngle - currentYaw, 2 * M_PI);
        agents_[id].sfmAgent.yaw.setRadian(currentYaw + error);
        agents_[id].sfmAgent.desiredVelocity = 0.0; // Noé test: stop when close
    }
    else
    {
        // printf("\n\n\t---> Agent %i is not close to agent %i yet: setting a temporary goal at the target's current position\n\n", 
        //       id, target_id);
        // Create a temporary goal at the target's current position.
        sfm::Goal g;
        g.center.set(tx, ty);
        g.radius = agents_[target_id].sfmAgent.radius;
        agents_[id].sfmAgent.goals.push_front(g);
        
        double d0 = closest_dist;
        double scaled_vel = max_vel * (dist / (dist + d0));
        float original_vel = agents_[id].sfmAgent.desiredVelocity;
        agents_[id].sfmAgent.desiredVelocity = static_cast<float>(scaled_vel);
        
        // Recompute forces and update position
        computeForces(id);
        sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
        
        // Remove the temporary goal and restore the original desired velocity
        agents_[id].sfmAgent.goals.pop_front();
        agents_[id].sfmAgent.desiredVelocity = original_vel;
    }
}


void AgentManager::followAgent(int id, int target_id, double dt, double threshold, double max_vel)
{
    std::lock_guard<std::mutex> guard(mutex_);
    
    agents_[id].behavior.state = 1;
    
    // Get the target and agent positions
    float tx = agents_[target_id].sfmAgent.position.getX();
    float ty = agents_[target_id].sfmAgent.position.getY();
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
    
    // Compute differences and Euclidean distance
    float dx = tx - ax;
    float dy = ty - ay;
    float dist = std::hypot(dx, dy);
    
    // Update orientation: make agent face the target
    float desiredAngle = std::atan2(dy, dx);
    float currentYaw = agents_[id].sfmAgent.yaw.toRadian();
    float error = std::remainder(desiredAngle - currentYaw, 2 * M_PI);
    agents_[id].sfmAgent.yaw.setRadian(currentYaw + error);
    
    // Create a temporary goal at the target's current position
    sfm::Goal g;
    g.center.set(tx, ty);
    g.radius = agents_[target_id].sfmAgent.radius;
    agents_[id].sfmAgent.goals.push_front(g);
    
    float original_vel = agents_[id].sfmAgent.desiredVelocity;
    
    if (dist >= threshold)
    {
        // Update forces and position
        computeForces(id);
        sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
    }
    else
    {
        // When within the threshold, scale velocity down rapidly
        double scaled_vel = max_vel * std::pow(dist / threshold, 2);
        agents_[id].sfmAgent.desiredVelocity = static_cast<float>(scaled_vel);
        
        computeForces(id);
        sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
        
        // Restore original desired velocity
        agents_[id].sfmAgent.desiredVelocity = original_vel;
    }
    
    // Remove the temporary goal
    agents_[id].sfmAgent.goals.pop_front();
}



void AgentManager::freezeAgent(int id)
{
    std::lock_guard<std::mutex> guard(mutex_);

    agents_[id].sfmAgent.desiredVelocity = 0.0;

    //printf("\n[AgentManager.freezeAgent] Agent %i frozen \n", id);
}

void AgentManager::resumeAgent(int id)
{
    std::lock_guard<std::mutex> guard(mutex_);

    agents_[id].sfmAgent.desiredVelocity = orig_desired_vels_[id];

    //printf("\n[AgentManager.resumeAgent] Agent %i resumed \n", id);
}

bool AgentManager::isRobotFacingAgent(int agent_id)
{
    float rx = robot_.sfmAgent.position.getX();
    float ry = robot_.sfmAgent.position.getY();
    float r_yaw = robot_.sfmAgent.yaw.toRadian();
    float ax = agents_[agent_id].sfmAgent.position.getX();
    float ay = agents_[agent_id].sfmAgent.position.getY();

    // Compute the angle from the robot to the agent
    double angle_to_agent = std::atan2(ay - ry, ax - rx);

    // Compute the difference between the robot's heading and the angle to the agent
    double angle_diff = angle_to_agent - r_yaw;
    // Normalize the angle difference to [-pi, pi]
    while (angle_diff > M_PI)  angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    return (std::fabs(angle_diff) < 0.3); // 17º approx
}

void AgentManager::setAgentGroupId(int id, int group_id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  agents_[id].sfmAgent.groupId = group_id;
  
  //printf("\nAgent %i group ID set to %i\n", id, group_id);
}

std::list<sfm::Goal> AgentManager::getAgentGoals(int id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  return agents_[id].sfmAgent.goals;
}


void AgentManager::setAgentGoal(int id, const sfm::Goal & goal)
{
  std::lock_guard<std::mutex> guard(mutex_);
  agents_[id].sfmAgent.goals.push_back(goal);
}

void AgentManager::clearAndSetAgentGoals(int id, const std::list<sfm::Goal> &goals)
{
    std::lock_guard<std::mutex> guard(mutex_);
    auto it = agents_.find(id);

    it->second.sfmAgent.goals.clear();
    for (const auto &g : goals)
    {
        it->second.sfmAgent.goals.push_back(g);
    }
}

void AgentManager::clearAndSetAgentGoal(int id, const sfm::Goal & goal)
{
  std::lock_guard<std::mutex> guard(mutex_);
  agents_[id].sfmAgent.goals.clear();
  agents_[id].sfmAgent.goals.push_back(goal);
}

void AgentManager::restoreAgentGoals(int agent_id, const std::list<sfm::Goal>& goals)
{
    std::lock_guard<std::mutex> guard(mutex_);
    agents_[agent_id].sfmAgent.goals = goals;
    //printf("[AgentManager::restoreAgentGoals] Agent %i goals restored.\n", agent_id);
    
}

utils::Vector2d AgentManager::computeConversationCenter(int agent_id, double circle_offset)
{
    std::lock_guard<std::mutex> guard(mutex_);
    auto it = agents_.find(agent_id);
    double x = it->second.sfmAgent.position.getX();
    double y = it->second.sfmAgent.position.getY();
    double yaw = it->second.sfmAgent.yaw.toRadian();
    // Compute the center: offset in the direction of the agent's current heading
    return utils::Vector2d(x + circle_offset * std::cos(yaw),
                            y + circle_offset * std::sin(yaw));
}

utils::Vector2d AgentManager::getRobotPosition()
{
    std::lock_guard<std::mutex> guard(mutex_);
    return robot_.sfmAgent.position;
}

double AgentManager::getAgentYaw(int id) 
{
    std::lock_guard<std::mutex> guard(mutex_);
    return agents_[id].sfmAgent.yaw.toRadian();
}

utils::Vector2d AgentManager::getAgentPosition(int id)
{
    std::lock_guard<std::mutex> guard(mutex_);
    return agents_[id].sfmAgent.position;
}

void AgentManager::overrideAgentGoals(int agent_id, int target_id)
{
  std::lock_guard<std::mutex> guard(mutex_);


  // Override the agent's goals with those of the target.
  agents_[target_id].sfmAgent.goals = agents_[agent_id].sfmAgent.goals;

  
  printf("[AgentManager.overrideAgentGoals] Agent %i now mimics goals from agent %i.\n", agent_id, target_id);
}

bool AgentManager::goalReached(int id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!agents_[id].sfmAgent.goals.empty() &&
      (agents_[id].sfmAgent.goals.front().center - agents_[id].sfmAgent.position).norm() <=
          (agents_[id].sfmAgent.goals.front().radius + 0.1))
  {
    // printf("Agent %s goal reached!", agents_[id].name.c_str());
    return true;
  }
  else
    return false;
}

bool AgentManager::updateGoal(int id)
{
  std::lock_guard<std::mutex> guard(mutex_);

  // printf("Updating goal for agent %i\n\n", id);
  sfm::Goal g = agents_[id].sfmAgent.goals.front();
  agents_[id].sfmAgent.goals.pop_front();
  if (agents_[id].sfmAgent.cyclicGoals)
  {
    agents_[id].sfmAgent.goals.push_back(g);
  }
  return true;
}

void AgentManager::initializeAgents(const hunav_msgs::msg::Agents::SharedPtr msg)
{
  printf("Initializing SFM Agents...\n");
  for (auto a : msg->agents)
  {
    agent ag;
    ag.name = a.name;
    ag.type = a.type;

    // behavior
    ag.behavior.type = a.behavior.type;
    ag.behavior.state = a.behavior.state;
    ag.behavior.configuration = a.behavior.configuration;  // def, manual, random
    ag.behavior.duration = a.behavior.duration;
    ag.behavior.once = a.behavior.once;
    ag.behavior.vel = a.behavior.vel;
    ag.behavior.dist = a.behavior.dist;
    ag.behavior.forceFactor = a.behavior.other_force_factor;

    ag.sfmAgent.id = a.id;
    ag.sfmAgent.groupId = a.group_id;
    ag.sfmAgent.desiredVelocity = a.desired_velocity;
    orig_desired_vels_[ag.sfmAgent.id] = a.desired_velocity;
    ag.sfmAgent.radius = a.radius;
    ag.sfmAgent.cyclicGoals = a.cyclic_goals;
    ag.sfmAgent.position.set(a.position.position.x, a.position.position.y);
    ag.sfmAgent.yaw.setRadian(a.yaw);
    ag.sfmAgent.velocity.set(a.velocity.linear.x, a.velocity.linear.y);
    ag.sfmAgent.linearVelocity =
        sqrt(a.velocity.linear.x * a.velocity.linear.x + a.velocity.linear.y * a.velocity.linear.y);
    ag.sfmAgent.angularVelocity = a.velocity.angular.z;
    for (auto g : a.goals)
    {
      sfm::Goal sfmg;
      sfmg.center.setX(g.position.x);
      sfmg.center.setY(g.position.y);
      sfmg.radius = a.goal_radius;
      ag.sfmAgent.goals.push_back(sfmg);
    }
    ag.sfmAgent.obstacles1.clear();
    if (!a.closest_obs.empty())
    {
      for (auto obs : a.closest_obs)
      {
        utils::Vector2d o;
        o.set(obs.x, obs.y);
        ag.sfmAgent.obstacles1.push_back(o);
      }
    }
    ag.sfmAgent.params.forceFactorSocial = a.behavior.social_force_factor;  // 5.0; // 2.1 by default
    ag.sfmAgent.params.forceFactorDesired = a.behavior.goal_force_factor;
    ag.sfmAgent.params.forceFactorObstacle = a.behavior.obstacle_force_factor;

    agents_[ag.sfmAgent.id] = ag;
    printf("\tagent %s, x:%.2f, y:%.2f, th:%.2f\n", agents_[ag.sfmAgent.id].name.c_str(),
           agents_[ag.sfmAgent.id].sfmAgent.position.getX(), agents_[ag.sfmAgent.id].sfmAgent.position.getY(),
           agents_[ag.sfmAgent.id].sfmAgent.yaw.toRadian());
  }
  agents_initialized_ = true;
  printf("SFM Agents initialized\n");
}

void AgentManager::initializeRobot(const hunav_msgs::msg::Agent::SharedPtr msg)
{
  robot_.name = msg->name;
  robot_.type = msg->type;
  // robot_.behavior = msg->behavior;
  robot_.sfmAgent.id = msg->id;
  robot_.sfmAgent.groupId = msg->group_id;
  robot_.sfmAgent.desiredVelocity = msg->desired_velocity;
  robot_.sfmAgent.radius = msg->radius;
  robot_.sfmAgent.cyclicGoals = msg->cyclic_goals;
  robot_.sfmAgent.position.set(msg->position.position.x, msg->position.position.y);
  robot_.sfmAgent.yaw.setRadian(msg->yaw);
  robot_.sfmAgent.velocity.set(msg->velocity.linear.x, msg->velocity.linear.y);
  robot_.sfmAgent.linearVelocity =
      sqrt(msg->velocity.linear.x * msg->velocity.linear.x + msg->velocity.linear.y * msg->velocity.linear.y);
  robot_.sfmAgent.angularVelocity = msg->velocity.angular.z;

  printf("\trobot %i, x:%.2f, y:%.2f\n", robot_.sfmAgent.id, robot_.sfmAgent.position.getX(),
         robot_.sfmAgent.position.getY());

  robot_initialized_ = true;
  printf("SFM Robot initialized\n");
}

bool AgentManager::updateAgents(const hunav_msgs::msg::Agents::SharedPtr msg)
{
  // update velocities
  // if (agents_initialized_ && time_step_secs_ > 0.0) {

  // Update agents velocites only in the ROS cycle
  for (auto a : msg->agents)
  {
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
    agents_[a.id].sfmAgent.position.set(a.position.position.x, a.position.position.y);
    agents_[a.id].sfmAgent.yaw.setRadian(a.yaw);

    // velocities
    agents_[a.id].sfmAgent.velocity.set(a.velocity.linear.x, a.velocity.linear.y);
    agents_[a.id].sfmAgent.linearVelocity =
        sqrt(a.velocity.linear.x * a.velocity.linear.x + a.velocity.linear.y * a.velocity.linear.y);
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
    if (!a.closest_obs.empty())
    {
      for (auto obs : a.closest_obs)
      {
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

void AgentManager::updateAgentRobot(const hunav_msgs::msg::Agent::SharedPtr msg)
{
  // Update robot
  robot_.sfmAgent.position.set(msg->position.position.x, msg->position.position.y);
  // tf2::Quaternion q(
  //     robot_.position.orientation.x, robot_.position.orientation.y,
  //     robot_.position.orientation.z, robot_.position.orientation.w);
  // tf2::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  robot_.sfmAgent.yaw.setRadian(msg->yaw);
  robot_.sfmAgent.velocity.set(msg->velocity.linear.x, msg->velocity.linear.y);
  robot_.sfmAgent.linearVelocity =
      sqrt(msg->velocity.linear.x * msg->velocity.linear.x + msg->velocity.linear.y * msg->velocity.linear.y);
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

hunav_msgs::msg::Agent AgentManager::getUpdatedAgentMsg(int id)
{
  // std::lock_guard<std::mutex> guard(mutex_);
  hunav_msgs::msg::Agent a;
  a.id = agents_[id].sfmAgent.id;
  a.name = agents_[id].name;
  a.type = agents_[id].type;
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
  // behavior
  a.behavior.type = agents_[id].behavior.type;
  a.behavior.state = agents_[id].behavior.state;
  a.behavior.configuration = agents_[id].behavior.configuration;  // def, manual, random
  a.behavior.duration = agents_[id].behavior.duration;
  a.behavior.once = agents_[id].behavior.once;
  a.behavior.vel = agents_[id].behavior.vel;
  a.behavior.dist = agents_[id].behavior.dist;
  a.behavior.other_force_factor = agents_[id].behavior.forceFactor;
  a.behavior.social_force_factor = agents_[id].sfmAgent.params.forceFactorSocial;  // 5.0; // 2.1 by default
  a.behavior.goal_force_factor = agents_[id].sfmAgent.params.forceFactorDesired;
  a.behavior.obstacle_force_factor = agents_[id].sfmAgent.params.forceFactorObstacle;

  for (auto g : agents_[id].sfmAgent.goals)
  {
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
hunav_msgs::msg::Agents AgentManager::getUpdatedAgentsMsg()
{
  // Make a copy of agents_
  // Then update them with the sfm_agents_ data
  std::lock_guard<std::mutex> guard(mutex_);
  hunav_msgs::msg::Agents agents_msg;
  agents_msg.header = header_;

  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++)
  {
    hunav_msgs::msg::Agent a = getUpdatedAgentMsg(itr->first);
    agents_msg.agents.push_back(a);
  }
  robot_received_ = false;
  agents_received_ = false;
  return agents_msg;
}

std::vector<sfm::Agent> AgentManager::getSFMAgents()
{
  std::vector<sfm::Agent> agents;
  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++)
  {
    agents.push_back(itr->second.sfmAgent);
  }
  return agents;
}

void AgentManager::computeForces(int id)
{
  std::vector<sfm::Agent> otherAgents = getSFMAgents();

  utils::Vector2d ob;
  if (agents_[id].behavior.state == 1)
  {
    switch (agents_[id].behavior.type)
    {
      case hunav_msgs::msg::AgentBehavior::BEH_REGULAR:
        // We add the robot as another human agent.
        otherAgents.push_back(robot_.sfmAgent);
        sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE:
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
    agents_[id].behavior.state = 0;
  }
  else
  {
    otherAgents.push_back(robot_.sfmAgent);
    sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
  }

  bool compute = false;
  if (isnan(agents_[id].sfmAgent.forces.desiredForce.norm()))
  {
    printf("[AgentManager.ComputeForces] \tgoal force is nan. Using zero.. \n");
    agents_[id].sfmAgent.forces.desiredForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.groupForce.norm()))
  {
    printf(
        "[AgentManager.ComputeForces] \tgroup force is nan. Using zero.. "
        "\n");
    agents_[id].sfmAgent.forces.groupForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.obstacleForce.norm()))
  {
    printf(
        "[AgentManager.ComputeForces] \tobstacle force is nan. Using "
        "zero.. \n");
    agents_[id].sfmAgent.forces.obstacleForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.socialForce.norm()))
  {
    // printf(
    //     "[AgentManager.ComputeForces] \tsocial force is nan. Using "
    //     "zero.. \n");
    agents_[id].sfmAgent.forces.socialForce.set(0, 0);
    compute = true;
  }
  if (compute)
  {
    agents_[id].sfmAgent.forces.globalForce =
        agents_[id].sfmAgent.forces.desiredForce + agents_[id].sfmAgent.forces.socialForce +
        agents_[id].sfmAgent.forces.obstacleForce + agents_[id].sfmAgent.forces.groupForce;
  }
  // if(id == 1)
  // {
  //   printf("[AgentManager.ComputeForces] \tForces. global:%.4f, goal:%.4f "
  //          "soc:%.4f, "
  //          "obs:%.4f \n",
  //          agents_[id].sfmAgent.forces.globalForce.norm(),
  //          agents_[id].sfmAgent.forces.desiredForce.norm(),
  //          agents_[id].sfmAgent.forces.socialForce.norm(),
  //          agents_[id].sfmAgent.forces.obstacleForce.norm());
  // }
}

void AgentManager::computeForces()
{
  std::vector<sfm::Agent> otherAgents = getSFMAgents();

  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++)
  {
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

void AgentManager::updateAllAgents(const hunav_msgs::msg::Agent::SharedPtr robot_msg,
                                   const hunav_msgs::msg::Agents::SharedPtr agents_msg)
{
  std::lock_guard<std::mutex> guard(mutex_);

  // printf("[AgentManager.updateAllAgents] Receiving agents from
  // simulator...\n"); for (auto &a : agents_msg->agents) {
  //   printf("[AgentManager.updateAllAgents]\tagent %s\n", a.name.c_str());
  // }

  header_ = agents_msg->header;

  if (!robot_initialized_)
  {
    initializeRobot(robot_msg);
  }

  if (!agents_initialized_)
  {
    // initializeBehaviorTree();
    initializeAgents(agents_msg);
    // prev_time_ = agents_msg->header.stamp;
    // time_step_secs_ = 0.0;
    // computeForces();
  }
  else if (robot_initialized_ && agents_initialized_)
  {
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

void AgentManager::updateAgentsAndRobot(const hunav_msgs::msg::Agents::SharedPtr agents_msg)
{
  std::lock_guard<std::mutex> guard(mutex_);

  // printf("[AgentManager.updateAgentsAndRobot] Receiving agents from "
  //        "simulator...\n");

  header_ = agents_msg->header;

  // The robot is the last agent of the vector!
  // or we could look for the type "robot" in the vector
  hunav_msgs::msg::Agent::SharedPtr rob = std::make_shared<hunav_msgs::msg::Agent>(agents_msg->agents.back());

  // we remove the robot from the agents vector
  hunav_msgs::msg::Agents ags = *agents_msg;
  ags.agents.pop_back();

  if (!robot_initialized_)
  {
    initializeRobot(rob);
  }
  if (!agents_initialized_)
  {
    initializeAgents(std::make_shared<hunav_msgs::msg::Agents>(ags));
    // prev_time_ = agents_msg->header.stamp;
    // time_step_secs_ = 0.0;
  }
  else
  {
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

bool AgentManager::canCompute()
{
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

void AgentManager::updatePosition(int id, double dt)
{
  std::lock_guard<std::mutex> guard(mutex_);

  agents_[id].behavior.state = 0;

  float prev_yaw = agents_[id].sfmAgent.yaw.toRadian();
  sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
  if(agents_[id].sfmAgent.linearVelocity < 0.07)
  {
    // keep previous yaw
    agents_[id].sfmAgent.yaw.setRadian(prev_yaw);
  } 
  
}

}  // namespace hunav
