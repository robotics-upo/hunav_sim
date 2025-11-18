
#include "hunav_agent_manager/bt_functions.hpp"
#include "hunav_agent_manager/agent_say_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <random>
#include <sstream>
#include <iostream>

namespace hunav
{
  AgentManager* g_agent_manager = nullptr;
  BTfunctions * g_btfunctions = nullptr;

BTfunctions::BTfunctions()
{
  // int Box::objectCount = 0;
  // AgentManager::agents_.clear();
  // AgentManager::sfm_agents_.clear();
  init();
  g_agent_manager = &agent_manager_;
  g_btfunctions = this;

  //printf("[BTfunctions.Constructor] initialized!\n");
}

BTfunctions::~BTfunctions()
{
}

void BTfunctions::init()
{
  //printf("[BTfunctions.init] initialized!\n");
  // agent_manager_.init();
}

geometry_msgs::msg::Point BTfunctions::getGlobalGoal(int id) const
{
  auto it = global_goals_.find(id);
  if (it == global_goals_.end()) {
    throw std::runtime_error("BTfunctions::getGlobalGoal: no goal " + std::to_string(id));
  }
  return it->second;
}

BT::NodeStatus BTfunctions::robotVisible(BT::TreeNode& self)
{
  auto idmsg = self.getInput<int>("agent_id");
  if (!idmsg)
  {
    throw BT::RuntimeError("RobotVisible. missing required input [agent_id]: ", idmsg.error());
  }
  auto dmsg = self.getInput<double>("distance");
  if (!dmsg)
  {
    throw BT::RuntimeError("RobotVisible. missing required input [distance]: ", dmsg.error());
  }

  int id = idmsg.value();
  double dist = dmsg.value();
  // std::cout << "BTfunctions.robotVisible. Ticking agent: " << id <<
  // std::endl;
  if (agent_manager_.isRobotVisible(id, dist))
  {
    // std::cout << "BTfunctions.robotVisible. Returning success" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    // std::cout << "BTfunctions.robotVisible. Returning failure" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus BTfunctions::findNearestAgent(BT::TreeNode& self)
{
    auto id_msg = self.getInput<int>("agent_id");
    if (!id_msg)
    {
        throw BT::RuntimeError("findNearestAgent: missing required input [agent_id]: ", id_msg.error());
    }
    int agent_id = id_msg.value();

    int nearest = agent_manager_.findNearestAgent(agent_id);
    if (nearest < 0)
    {
        return BT::NodeStatus::FAILURE; // No candidate found
    }

    self.setOutput("target_agent_id", nearest);

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::agentVisible(BT::TreeNode& self)
{
    // Retrieve the observer's ID.
    auto observer_msg = self.getInput<int>("observer_id");
    auto target_msg = self.getInput<int>("agent_id");
    auto d_msg = self.getInput<double>("distance");
    auto fov_msg = self.getInput<double>("field_of_view");
    if (!observer_msg)
        throw BT::RuntimeError("agentVisible: missing input [observer_id]", observer_msg.error());

    if (!target_msg)
        throw BT::RuntimeError("agentVisible: missing input [agent_id]", target_msg.error());

    if (!d_msg)
        throw BT::RuntimeError("agentVisible: missing input [distance]", d_msg.error());

    if (!fov_msg)
        throw BT::RuntimeError("agentVisible: missing input [field_of_view]", fov_msg.error());
    int observer_id = observer_msg.value();
    int target_id = target_msg.value();
    double dist = d_msg.value();
    double fov = fov_msg.value();

    if (agent_manager_.isAgentVisible(observer_id, target_id, dist, fov))
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}


BT::NodeStatus BTfunctions::saySomething(BT::TreeNode& self)
{
  auto id_msg = self.getInput<int>("agent_id");
  auto message_msg = self.getInput<std::string>("message");
  if (!id_msg)
    throw BT::RuntimeError("SaySomething: missing required input [agent_id]", id_msg.error());
  if (!message_msg)
    throw BT::RuntimeError("SaySomething: missing required input [message]", message_msg.error());
  
  int id = id_msg.value();
  std::string message = message_msg.value();

  // Create the ROS2 string message.
  std_msgs::msg::String ros_msg;
  ros_msg.data = "Agent " + std::to_string(id) + " says: " + message;
  
  // Now, use the AgentSayNode instance to publish the message.
  auto agent_say_node = hunav::AgentSayNode::getInstance();
  if (agent_say_node)
  {
    agent_say_node->publishMessage(ros_msg.data);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("BTfunctions"), "AgentSayNode instance is not set!");
  }
  
  return BT::NodeStatus::SUCCESS;
}



BT::NodeStatus BTfunctions::lookAtAgent(BT::TreeNode& self)
{
  auto obs_msg = self.getInput<int>("observer_id");
  if (!obs_msg)
    throw BT::RuntimeError("lookAtAgent: missing input [observer_id]", obs_msg.error());
  
  auto tgt_msg = self.getInput<int>("target_id");
  if (!tgt_msg)
    throw BT::RuntimeError("lookAtAgent: missing input [target_id]", tgt_msg.error());

  int observer_id = obs_msg.value();
  int target_id = tgt_msg.value();

  agent_manager_.lookAtAgent(observer_id, target_id);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::lookAtRobot(BT::TreeNode & self)
{
  auto id_msg = self.getInput<int>("agent_id");
  if (!id_msg)
  {
    throw BT::RuntimeError("LookAtRobot: missing required input [agent_id]: ", id_msg.error());
  }
  int id = id_msg.value();
  
  // Call the AgentManager function to adjust the agent's orientation toward the robot
  agent_manager_.lookAtTheRobot(id);
  
  return BT::NodeStatus::SUCCESS;
}

// BT::NodeStatus BTfunctions::lookAtPoint(BT::TreeNode & self)
// {
//     auto agent_id_msg = self.getInput<int>("agent_id");
//     if (!agent_id_msg)
//       throw BT::RuntimeError("lookAtPoint: missing input [agent_id]", agent_id_msg.error());
    
//     auto x_msg = self.getInput<double>("target_x");
//     auto y_msg = self.getInput<double>("target_y");
//     if (!x_msg || !y_msg)
//       throw BT::RuntimeError("lookAtPoint: missing input [target_x] or [target_y]");
    
//     int agent_id = agent_id_msg.value();
//     double target_x = x_msg.value();
//     double target_y = y_msg.value();
    
//     utils::Vector2d target(target_x, target_y);
//     agent_manager_.lookAtPoint(agent_id, target);
    
//     return BT::NodeStatus::SUCCESS;
// }

BT::NodeStatus BTfunctions::isRobotClose(BT::TreeNode& self)
{
  // Retrieve the agent id from the blackboard
  auto id_msg = self.getInput<int>("agent_id");
  if (!id_msg)
  {
    throw BT::RuntimeError("IsRobotClose: missing required input [agent_id]: ", id_msg.error());
  }
  int agent_id = id_msg.value();

  // Retrieve the threshold from the blackboard
  auto thresh_msg = self.getInput<double>("threshold");
  if (!thresh_msg)
  {
    throw BT::RuntimeError("IsRobotClose: missing required input [threshold]: ", thresh_msg.error());
  }
  double threshold = thresh_msg.value();

  // Compute the distance from the agent to the robot
  float squared_dist = agent_manager_.robotSquaredDistance(agent_id);
  double distance = std::sqrt(squared_dist);


  // std::cout << "Agent " << agent_id << " is " << distance << " m from the robot." << std::endl;

  // Return SUCCESS if the distance is below the threshold, otherwise FAILURE
  if (distance < threshold)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus BTfunctions::isAgentClose(BT::TreeNode & self)
{
    // Retrieve observer's ID
    auto observer_msg = self.getInput<int>("observer_id");
    if (!observer_msg)
        throw BT::RuntimeError("IsAgentClose: missing input [observer_id]", observer_msg.error());
    
    // Retrieve target agent's ID
    auto target_msg = self.getInput<int>("target_agent_id");
    if (!target_msg)
        throw BT::RuntimeError("IsAgentClose: missing input [target_agent_id]", target_msg.error());
    
    // Retrieve the distance threshold
    auto threshold_msg = self.getInput<double>("threshold");
    if (!threshold_msg)
        throw BT::RuntimeError("IsAgentClose: missing input [threshold]", threshold_msg.error());
    
    int observer_id = observer_msg.value();
    int target_id = target_msg.value();
    double threshold = threshold_msg.value();
    
    bool isClose = agent_manager_.isAgentClose(observer_id, target_id, threshold);
    
    return isClose ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus BTfunctions::randomChance(BT::TreeNode& self)
{
  // Retrieve the probability from the input port
  auto prob_msg = self.getInput<double>("probability");
  if (!prob_msg)
  {
    throw BT::RuntimeError("RandomChanceCondition: missing required input [probability]: ", prob_msg.error());
  }
  double probability = prob_msg.value();

  // Generate a random number in [0.0, 1.0)
  static std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  double random_value = dist(rng);


  // std::cout << "RandomChanceCondition: probability = " << probability
  //           << ", random_value = " << random_value << std::endl;

  // Return SUCCESS if the random number is less than the probability, FAILURE otherwise.
  return (random_value < probability) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus BTfunctions::robotFacingAgent(BT::TreeNode & self)
{
  // Retrieve the agent id from the input port
  auto id_msg = self.getInput<int>("agent_id");
  if (!id_msg)
  {
    throw BT::RuntimeError("IsRobotFacingAgent: missing required input [agent_id]: ", id_msg.error());
  }
  int agent_id = id_msg.value();

  // Call the new AgentManager method
  bool facing = agent_manager_.isRobotFacingAgent(agent_id);
  return facing ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus BTfunctions::setGroupId(BT::TreeNode & self)
{
  auto agent_id_msg = self.getInput<int>("agent_id");
  if (!agent_id_msg)
    throw BT::RuntimeError("setGroupId: missing required input [agent_id]", agent_id_msg.error());

  auto group_id_msg = self.getInput<int>("group_id");
  if (!group_id_msg)
    throw BT::RuntimeError("setGroupId: missing required input [group_id]", group_id_msg.error());

  int agent_id = agent_id_msg.value();
  int group_id = group_id_msg.value();

  agent_manager_.setAgentGroupId(agent_id, group_id);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::setGoal(BT::TreeNode & self)
{
  auto id_msg = self.getInput<int>("agent_id");
  if (!id_msg)
    throw BT::RuntimeError("setGoal: missing required input [agent_id]: ", id_msg.error());

  auto goal_id_msg = self.getInput<int>("goal_id");
  if (!goal_id_msg)
    throw BT::RuntimeError("setGoal: missing required input [goal_id]: ", goal_id_msg.error());

  int agent_id = id_msg.value();
  int goal_id = goal_id_msg.value();

  auto it = global_goals_.find(goal_id);
  if (it == global_goals_.end())
    return BT::NodeStatus::FAILURE;

  const auto & pt = it->second;
  sfm::Goal  goal;
  goal.center.set(pt.x, pt.y);
  goal.radius = 0.1;

  agent_manager_.setAgentGoal(agent_id, goal);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::isAtPosition(BT::TreeNode& self)
{
  auto id_msg = self.getInput<int>("agent_id");
  if (!id_msg)
    throw BT::RuntimeError("isAtPosition: missing required input [agent_id]: ", id_msg.error());

  auto goal_id_msg = self.getInput<int>("goal_id");
  if (!goal_id_msg)
    throw BT::RuntimeError("isAtPosition: missing required input [goal_id]: ", goal_id_msg.error());

  auto tol_msg  = self.getInput<double>("tolerance");
  if (!tol_msg)
    throw BT::RuntimeError("isAtPosition: missing required input [tolerance]: ", tol_msg.error());
 
  // Use a default tolerance if not provided
  double tolerance = (tol_msg) ? tol_msg.value() : 0.1;

  int agent_id = id_msg.value();
  int goal_id = goal_id_msg.value();

  auto it = global_goals_.find(goal_id);
  if (it == global_goals_.end())
    return BT::NodeStatus::FAILURE;

  // Retrieve the agent's current position
  utils::Vector2d pos = agent_manager_.getAgentPosition(agent_id);
  
  // Compute the Euclidean distance to the target
  const auto & pt = it->second;
  double dx = pos.getX() - pt.x;
  double dy = pos.getY() - pt.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // Return SUCCESS if the agent is within tolerance; FAILURE otherwise
  return (distance <= tolerance) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus BTfunctions::blockRobot(BT::TreeNode & self)
{
  // Retrieve required inputs.
  auto id_msg = self.getInput<int>("agent_id");
  auto dt_msg = self.getInput<double>("time_step");
  auto front_dist_msg = self.getInput<double>("front_dist");

  if (!id_msg)
    throw BT::RuntimeError("BlockRobot: missing input [agent_id]", id_msg.error());
  if (!dt_msg)
    throw BT::RuntimeError("BlockRobot: missing input [time_step]", dt_msg.error());
  if (!front_dist_msg)
    throw BT::RuntimeError("BlockRobot: missing input [front_dist]", front_dist_msg.error());

  int agent_id = id_msg.value();
  double dt = dt_msg.value();
  double front_dist = front_dist_msg.value();

  agent_manager_.blockRobot(agent_id, dt, front_dist);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::blockAgent(BT::TreeNode & self)
{
  // Retrieve required inputs.
  auto id_msg = self.getInput<int>("agent_id");
  auto target_id_msg = self.getInput<int>("target_agent_id");
  auto dt_msg = self.getInput<double>("time_step");
  auto front_dist_msg = self.getInput<double>("front_dist");

  if (!id_msg)
    throw BT::RuntimeError("BlockAgent: missing input [agent_id]", id_msg.error());
  if (!target_id_msg)
    throw BT::RuntimeError("BlockAgent: missing input [target_agent_id]", target_id_msg.error());
  if (!dt_msg)
    throw BT::RuntimeError("BlockAgent: missing input [time_step]", dt_msg.error());
  if (!front_dist_msg)
    throw BT::RuntimeError("BlockAgent: missing input [front_dist]", front_dist_msg.error());

  int agent_id = id_msg.value();
  int target_id = target_id_msg.value();
  double dt = dt_msg.value();
  double front_dist = front_dist_msg.value();

  agent_manager_.blockAgent(agent_id, target_id, dt, front_dist);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::stopMovement(BT::TreeNode& self)
{
  auto msg = self.getInput<int>("agent_id");
  if (!msg)
  {
    throw BT::RuntimeError("StopMovement. missing required input [agent_id]: ", msg.error());
  }
  int id = msg.value();

  agent_manager_.freezeAgent(id);

  return BT::NodeStatus::SUCCESS;

}

BT::NodeStatus BTfunctions::resumeMovement(BT::TreeNode& self)
{
  auto msg = self.getInput<int>("agent_id");
  if (!msg)
  {
    throw BT::RuntimeError("ResumeMovement. missing required input [agent_id]: ", msg.error());
  }
  int id = msg.value();
  agent_manager_.resumeAgent(id);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::goalReached(BT::TreeNode& self)
{
  auto msg = self.getInput<int>("agent_id");
  if (!msg)
  {
    throw BT::RuntimeError("GoalReached. missing required input [agent_id]: ", msg.error());
  }
  int id = msg.value();
  if (agent_manager_.goalReached(id))
  {
    // std::cout << "BTfunctions.GoalReached. agent: " << id << " Goal Reached!"
    //          << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    // std::cout << "BTfunctions.GoalReached. agent: " << id << " Goal not
    // reached"
    //           << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus BTfunctions::updateGoal(BT::TreeNode& self)
{
  auto msg = self.getInput<int>("agent_id");
  if (!msg)
  {
    throw BT::RuntimeError("UpdateGoal. missing required input [agent_id]: ", msg.error());
  }
  int id = msg.value();
  if (agent_manager_.updateGoal(id))
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

// BT::NodeStatus BTfunctions::impassiveNav(BT::TreeNode &self) {
//  return regularNav(self);
//}

BT::NodeStatus BTfunctions::regularNav(BT::TreeNode& self)
{
  auto msg = self.getInput<int>("agent_id");
  auto msg2 = self.getInput<double>("time_step");
  if (!msg)
  {
    throw BT::RuntimeError("RegularNav. missing required input [agent_id]: ", msg.error());
  }
  if (!msg2)
  {
    throw BT::RuntimeError("RegularNav. missing required input [time_step]: ", msg2.error());
  }

  int id = msg.value();
  double dt = msg2.value();
  // double dt = (_info.simTime - this->lastUpdate).Double();
  // std::cout << "[BTfunctions.RegularNav] Ticking agent: " << id << " dt: "
  // << dt
  //           << std::endl;
  // Update SFM model position
  agent_manager_.updatePosition(id, dt);
  // sfm::SFM.updatePosition(sfm_agents_[id], time_step_secs_);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::surprisedNav(BT::TreeNode& self)
{
  // auto msg = self.getInput<std::string>("agent_id");
  auto msg = self.getInput<int>("agent_id");
  if (!msg)
  {
    throw BT::RuntimeError("SurprisedNav. missing required input [agent_id]: ", msg.error());
  }
  // int id = std::stoi(msg.value()); // BT::convertFromString<int>(msg);
  int id = msg.value();
  // std::cout << "BTfunctions.SurprisedNav. Ticking agent: " << id <<
  // std::endl;
  // stop the agent and just look at the robot (change the agent orientation)
  agent_manager_.lookAtTheRobot(id);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::curiousNav(BT::TreeNode& self)
{
  auto msg = self.getInput<int>("agent_id");
  auto msg2 = self.getInput<double>("time_step");
  auto msg3 = self.getInput<double>("stop_distance");
  auto msg4 = self.getInput<double>("agent_vel");
  if (!msg)
  {
    throw BT::RuntimeError("CuriousNav. missing required input [agent_id]: ", msg.error());
  }
  if (!msg2)
  {
    throw BT::RuntimeError("CuriousNav. missing required input [time_step]: ", msg2.error());
  }
  if (!msg3)
  {
    throw BT::RuntimeError("CuriousNav. missing required input [stop_distance]: ", msg3.error());
  }
  if (!msg4)
  {
    throw BT::RuntimeError("CuriousNav. missing required input [agent_vel]: ", msg4.error());
  }
  int id = msg.value();
  double dt = msg2.value();
  double dist = msg3.value();
  double vel = msg4.value();

  agent_manager_.approximateRobot(id, dt, dist, vel);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::scaredNav(BT::TreeNode& self)
{
  auto msg = self.getInput<int>("agent_id");
  auto msg2 = self.getInput<double>("time_step");
  auto msg3 = self.getInput<double>("runaway_vel");
  auto msg4 = self.getInput<double>("safe_distance");
  if (!msg)
  {
    throw BT::RuntimeError("ScaredNav. missing required input [agent_id]: ", msg.error());
  }
  if (!msg2)
  {
    throw BT::RuntimeError("ScaredNav. missing required input [time_step]: ", msg2.error());
  }
  if (!msg3)
  {
    throw BT::RuntimeError("ScaredNav. missing required input [runaway_vel]: ", msg3.error());
  }
  if (!msg4)
  {
    throw BT::RuntimeError("ScaredNav. missing required input [safe_distance]: ", msg4.error());
  }
  int id = msg.value();
  // printf("[BTfunctions.ScareNav] After getting id: %i\n", id);
  double dt = msg2.value();
  double vel = msg3.value();
  double safe_distance = msg4.value();

  agent_manager_.avoidRobot(id, dt, safe_distance, vel);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTfunctions::threateningNav(BT::TreeNode& self)
{
  auto msg = self.getInput<int>("agent_id");
  auto msg2 = self.getInput<double>("time_step");
  auto msg3 = self.getInput<double>("goal_dist");
  if (!msg)
  {
    throw BT::RuntimeError("threateningNav. missing required input [agent_id]: ", msg.error());
  }
  if (!msg2)
  {
    throw BT::RuntimeError("threateningNav. missing required input [time_step]: ", msg2.error());
  }
  if (!msg3)
  {
    throw BT::RuntimeError("threateningNav. missing required input [goal_dist]: ", msg3.error());
  }
  int id = msg.value();
  double dt = msg2.value();
  double gdist = msg3.value();

  agent_manager_.blockRobot(id, dt, gdist);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace hunav
