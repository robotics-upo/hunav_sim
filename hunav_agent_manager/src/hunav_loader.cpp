#include "hunav_agent_manager/hunav_loader.hpp"
#include "random"
#include <vector>
#include <algorithm>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace hunav
{

// HunavLoader::HunavLoader()
//     : Node("hunav_loader",
//            rclcpp::NodeOptions()
//                .allow_undeclared_parameters(true)
//                .automatically_declare_parameters_from_overrides(true)) {
HunavLoader::HunavLoader() : Node("hunav_loader")
{
  /* node parameter declaration */
  // std::string base_world = this->declare_parameter<std::string>(
  //    "base_world", std::string("empty.world"));

  // Read yaml_base_name parameter (derive from the config file name)
  std::string yaml_base_name = this->declare_parameter<std::string>("yaml_base_name", std::string("warehouse_agents"));
  RCLCPP_INFO(this->get_logger(), "yaml_base_name: %s", yaml_base_name.c_str());

  // Read simulator parameter (optional for backward compatibility)
  std::string simulator = this->declare_parameter<std::string>("simulator", std::string("Gazebo"));
  RCLCPP_INFO(this->get_logger(), "simulator: %s", simulator.c_str());

  std::string map = this->declare_parameter<std::string>("map", std::string("warehouse"));
  RCLCPP_INFO(this->get_logger(), "map: %s", map.c_str());

  // Read publish_people parameter
  bool publish_people = this->declare_parameter<bool>("publish_people", true);
  RCLCPP_INFO(this->get_logger(), "publish_people: %s", publish_people ? "true" : "false");

  //this->declare_parameter(std::string("agents"), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  //rclcpp::Parameter array_agents = this->get_parameter("agents");
  //auto agent_names = array_agents.as_string_array();
  // Declare `agents` as a string array with an empty default to avoid
  // ParameterUninitializedException when the param is not provided.
  auto agent_names = this->declare_parameter<std::vector<std::string>>("agents", std::vector<std::string>{});
  RCLCPP_INFO(this->get_logger(), "Number of agents: %zu", agent_names.size());
  
  //   rclcpp::Parameter agents_array;
  //   this->get_parameter_or("agents", agents_array,
  //                          rclcpp::Parameter("agents", "[]"));
  
  // Read and display global goals
  RCLCPP_INFO(this->get_logger(), "Global goals:");
  for (int test_id = 1; ; ++test_id) {
    std::string x_key = "global_goals." + std::to_string(test_id) + ".x";
    std::string y_key = "global_goals." + std::to_string(test_id) + ".y";
    
    try {
      double x = this->declare_parameter<double>(x_key, std::numeric_limits<double>::max());
      double y = this->declare_parameter<double>(y_key, std::numeric_limits<double>::max());
      
      if (x != std::numeric_limits<double>::max() && y != std::numeric_limits<double>::max()) {
        RCLCPP_INFO(this->get_logger(), "  goal %d: (%.3f, %.3f)", test_id, x, y);
      } else {
        // This goal doesn't exist in the YAML, stop checking
        break;
      }
    } catch (const std::exception& e) {
      // This goal doesn't exist, stop checking
      break;
    }
  }
  
  for (auto name : agent_names)
  {
    RCLCPP_INFO(this->get_logger(), "Agent name: %s", name.c_str());
    int id = this->declare_parameter<int>(name + ".id", -1);
    RCLCPP_INFO(this->get_logger(), "\tid: %i", id);
    int skin = this->declare_parameter<int>(name + ".skin", -1);
    RCLCPP_INFO(this->get_logger(), "\tskin: %i", skin);
    int group_id = this->declare_parameter<int>(name + ".group_id", -1);
    RCLCPP_INFO(this->get_logger(), "\tgroup_id: %i", group_id);
    double max_vel = this->declare_parameter<double>(name + ".max_vel", 1.0);
    RCLCPP_INFO(this->get_logger(), "\tmax_vel: %.2f", max_vel);
    double radius = this->declare_parameter<double>(name + ".radius", 0.35);
    RCLCPP_INFO(this->get_logger(), "\tradius: %.2f", radius);

    // Behavior 
    // int behavior = this->declare_parameter<int>(name + ".behavior.type", 0);
    std::string type = this->declare_parameter<std::string>(name + ".behavior.type", "Regular");
    RCLCPP_INFO(this->get_logger(), "\tbeh type: %s", type.c_str());
    int conf = this->declare_parameter<int>(name + ".behavior.configuration", 0);
    RCLCPP_INFO(this->get_logger(), "\tbeh configuration: %i", conf);

    // Configuration mode 'manual' - take the loaded values from the file:
    double duration = this->declare_parameter<double>(name + ".behavior.duration", 40.0);
    bool run_once = this->declare_parameter<bool>(name + ".behavior.once", true);
    double vel = this->declare_parameter<double>(name + ".behavior.vel", 1.0);
    if (vel > 1.8)
    {
      vel = 1.8;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.vel", vel));
    }
    else if (vel < 0.0)
    {
      vel = 0.0;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.vel", vel));
    }
    double dist = this->declare_parameter<double>(name + ".behavior.dist", 0.0);

    double facGoal = this->declare_parameter<double>(name + ".behavior.goal_force_factor", 2.0);

    if (conf != 1)  // 1->custom
    {               // ForceFactorDesired   [2, 5]  - def: 2
      if (facGoal < 2.0)
      {
        facGoal = 2.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.goal_force_factor", facGoal));
      }
      else if (facGoal > 5.0)
      {
        facGoal = 5.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.goal_force_factor", facGoal));
      }
    }

    double facObstacle = this->declare_parameter<double>(name + ".behavior.obstacle_force_factor", 10.0);
    if (conf != 1)
    {  // ForceFactorObstacle  [2, 50] - def: 10
      if (facObstacle < 2.0)
      {
        facObstacle = 2.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.obstacle_force_factor", facObstacle));
      }
      else if (facObstacle > 50.0)
      {
        facObstacle = 50.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.obstacle_force_factor", facObstacle));
      }
    }

    double facSocial = this->declare_parameter<double>(name + ".behavior.social_force_factor", 5.0);  // 2.1
    if (conf != 1)
    {  // ForceFactorSocial    [5.0, 20] - def: 5.0
      if (facSocial < 5.0)
      {
        facSocial = 5.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.social_force_factor", facSocial));
      }
      else if (facSocial > 20.0)
      {
        facSocial = 20.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.social_force_factor", facSocial));
      }
    }

    // currently used as an extra repulsive force for the scary behavior
    double facOther = this->declare_parameter<double>(name + ".behavior.other_force_factor", 20.0);
    if (conf != 1)
    {  // Other force factor  [0, 25] - def: 20.0
      if (facOther < 0.0)
      {
        facOther = 0.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.other_force_factor", facOther));
      }
      else if (facSocial > 25.0)
      {
        facOther = 25.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.other_force_factor", facOther));
      }
    }

    // If the configuration mode is default, overwrite the values with the default ones:
    if (conf == hunav_msgs::msg::AgentBehavior::BEH_CONF_DEFAULT)
    {
      // default SFM values
      facGoal = 2.0;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.goal_force_factor", facGoal));
      facObstacle = 10.0;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.obstacle_force_factor", facObstacle));
      facSocial = 5.0;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.social_force_factor", facSocial));

    }
    // If the configuration mode is random, overwrite with random values:
    // NORMAL DISTRIBUTION
    else if (conf == hunav_msgs::msg::AgentBehavior::BEH_CONF_RANDOM_NORMAL)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      // std::uniform_real_distribution<> dis_gff(2.0, 5.0);
      std::normal_distribution<> dis_gff{ 2.0, 1.5 };
      facGoal = dis_gff(gen);
      facGoal = (facGoal < 0.5) ? 0.5 : facGoal;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.goal_force_factor", facGoal));  // def: 2.0
      // std::uniform_real_distribution<> dis_off(8.0, 15.0);
      std::normal_distribution<> dis_off{ 10.0, 4.0 };
      facObstacle = dis_off(gen);
      facObstacle = (facObstacle < 0.5) ? 0.5 : facObstacle;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.obstacle_force_factor", facObstacle));  // def: 10.0
      // std::uniform_real_distribution<> dis_sff(2.1, 15.0);
      std::normal_distribution<> dis_sff{ 4.0, 3.5 };
      facSocial = dis_sff(gen);
      facSocial = (facSocial < 3.0) ? 3.0 : facSocial;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.social_force_factor", facSocial));  // def: 2.1
                                                                                                  // hunav: 5.0
    }
    // If the configuration mode is random, overwrite with random values:
    // UNIFORM DISTRIBUTION
    else if (conf == hunav_msgs::msg::AgentBehavior::BEH_CONF_RANDOM_UNIFORM)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis_gff(2.0, 5.0);
      facGoal = dis_gff(gen);
      this->set_parameter(rclcpp::Parameter(name + ".behavior.goal_force_factor", facGoal));  // def: 2.0
      std::uniform_real_distribution<> dis_off(2.0, 50.0);
      facObstacle = dis_off(gen);
      this->set_parameter(rclcpp::Parameter(name + ".behavior.obstacle_force_factor", facObstacle));  // def: 10.0
      std::uniform_real_distribution<> dis_sff(4.0, 20.0);
      facSocial = dis_sff(gen);
      this->set_parameter(rclcpp::Parameter(name + ".behavior.social_force_factor", facSocial));  // def: 2.1
                                                                                                  // hunav: 5.0
    }

    // print the behavior parameters
    RCLCPP_INFO(this->get_logger(), "\tbeh duration: %.2f", duration);
    RCLCPP_INFO(this->get_logger(), "\tbeh vel: %.2f", vel);
    RCLCPP_INFO(this->get_logger(), "\tbeh just_once: %i", (int)run_once);
    RCLCPP_INFO(this->get_logger(), "\tbeh dist: %.2f", dist);
    RCLCPP_INFO(this->get_logger(), "\tbeh goalForceFactor: %.2f", facGoal);
    RCLCPP_INFO(this->get_logger(), "\tbeh obstacleForceFactor: %.2f", facObstacle);
    RCLCPP_INFO(this->get_logger(), "\tbeh socialForceFactor: %.2f", facSocial);
    RCLCPP_INFO(this->get_logger(), "\tbeh otherForceFactor: %.2f", facOther);

    // init pose
    double posex = this->declare_parameter<double>(name + ".init_pose.x", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tposex: %.2f", posex);
    double posey = this->declare_parameter<double>(name + ".init_pose.y", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tposey: %.2f", posey);
    double posez = this->declare_parameter<double>(name + ".init_pose.z", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tposez: %.2f", posez);
    double poseh = this->declare_parameter<double>(name + ".init_pose.h", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tposeh: %.2f", poseh);

    // Goals
    double goal_radius = this->declare_parameter<double>(name + ".goal_radius", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tgoal_radius: %.2f", goal_radius);
    bool cyclic = this->declare_parameter<bool>(name + ".cyclic_goals", true);
    RCLCPP_INFO(this->get_logger(), "\tcyclic_goals: %i", (int)cyclic);
    
    // Handle new goals structure with global_goals references
    this->declare_parameter(name + ".goals", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
    rclcpp::Parameter goals_param = this->get_parameter(name + ".goals");
    auto goal_ids = goals_param.as_integer_array();
    
    RCLCPP_INFO(this->get_logger(), "\tgoals: [");
    for (size_t i = 0; i < goal_ids.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "\t  %ld%s", goal_ids[i], (i == goal_ids.size()-1) ? "" : ",");
    }
    RCLCPP_INFO(this->get_logger(), "\t]");
    
    // For each goal ID, read the corresponding global goal coordinates
    for (auto goal_id : goal_ids) {
      std::string x_key = "global_goals." + std::to_string(goal_id) + ".x";
      std::string y_key = "global_goals." + std::to_string(goal_id) + ".y";
      
      // Check if already declared, if not declare them
      double gx, gy;
      if (!this->has_parameter(x_key)) {
        gx = this->declare_parameter<double>(x_key, 0.0);
      } else {
        gx = this->get_parameter(x_key).as_double();
      }
      if (!this->has_parameter(y_key)) {
        gy = this->declare_parameter<double>(y_key, 0.0);
      } else {
        gy = this->get_parameter(y_key).as_double();
      }
      RCLCPP_INFO(this->get_logger(), "\t  goal_%ld: (%.3f, %.3f)", goal_id, gx, gy);
    }
  }

  // Create service to provide parameters to other nodes
  get_parameters_srv_ = this->create_service<hunav_msgs::srv::GetParameters>(
    "get_parameters",
    std::bind(&HunavLoader::getParametersService, this, std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(this->get_logger(), "GetParameters service created");

}

HunavLoader::~HunavLoader()
{
}

void HunavLoader::getParametersService(const std::shared_ptr<hunav_msgs::srv::GetParameters::Request> /* request */,
                                      std::shared_ptr<hunav_msgs::srv::GetParameters::Response> response)
{
  // Fill response with current parameter values
  response->publish_people = this->get_parameter("publish_people").as_bool();
  response->map = this->get_parameter("map").as_string();
  response->simulator = this->get_parameter("simulator").as_string();
  response->yaml_base_name = this->get_parameter("yaml_base_name").as_string();
  
  // Collect global goals - check sequentially until we find a gap
  std::vector<int64_t> goal_ids;
  std::vector<double> goal_x_coords;
  std::vector<double> goal_y_coords;
  
  for (int goal_id = 1; ; ++goal_id) {
    std::string x_key = "global_goals." + std::to_string(goal_id) + ".x";
    std::string y_key = "global_goals." + std::to_string(goal_id) + ".y";
    
    // If this goal doesn't exist, we're done
    if (!this->has_parameter(x_key) || !this->has_parameter(y_key)) {
      break;
    }
    
    try {
      double x = this->get_parameter(x_key).as_double();
      double y = this->get_parameter(y_key).as_double();
      
      // Check if values are valid 
      if (x != std::numeric_limits<double>::max() && y != std::numeric_limits<double>::max()) {
        goal_ids.push_back(goal_id);
        goal_x_coords.push_back(x);
        goal_y_coords.push_back(y);
      }
    } catch (const std::exception& e) {
      // Parameter exists but invalid value, skip this one but continue checking next
      RCLCPP_WARN(this->get_logger(), "Failed to read goal %d: %s", goal_id, e.what());
    }
  }
  
  response->goal_ids = goal_ids;
  response->goal_x_coords = goal_x_coords;
  response->goal_y_coords = goal_y_coords;
  
  RCLCPP_INFO(this->get_logger(), "Sent parameters: map=%s, simulator=%s, yaml_base_name=%s, %zu goals",
              response->map.c_str(), response->simulator.c_str(), response->yaml_base_name.c_str(), goal_ids.size());

}

}  // namespace hunav

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::HunavLoader>());

  rclcpp::shutdown();
  return 0;
}