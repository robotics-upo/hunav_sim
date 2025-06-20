#include "hunav_agent_manager/hunav_loader.hpp"
#include "random"

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

  std::string map = this->declare_parameter<std::string>("map", std::string("map.yaml"));

  this->declare_parameter(std::string("agents"), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  rclcpp::Parameter array_agents = this->get_parameter("agents");
  auto agent_names = array_agents.as_string_array();

  //   rclcpp::Parameter agents_array;
  //   this->get_parameter_or("agents", agents_array,
  //                          rclcpp::Parameter("agents", "[]"));

  RCLCPP_INFO(this->get_logger(), "map params: %s", map.c_str());
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
    int behavior = this->declare_parameter<int>(name + ".behavior.type", 0);
    RCLCPP_INFO(this->get_logger(), "\tbehavior type: %i", behavior);
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
      this->set_parameter(rclcpp::Parameter(name + ".behavior.once", true));
      facGoal = 2.0;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.goal_force_factor", facGoal));
      facObstacle = 10.0;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.obstacle_force_factor", facObstacle));
      facSocial = 5.0;
      this->set_parameter(rclcpp::Parameter(name + ".behavior.social_force_factor", facSocial));

      if (behavior == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
      {
        duration = 40.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        vel = max_vel;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.vel", max_vel));
        dist = 1.5;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED)
      {
        duration = 30.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        dist = 4.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
      {
        duration = 40.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        vel = 0.6;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.vel", vel));
        dist = 3.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
        this->set_parameter(rclcpp::Parameter(name + ".behavior.other_force_factor", 20.0));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
      {
        duration = 40.0;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        dist = 1.4;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
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
      // this->set_parameter(rclcpp::Parameter(name + ".behavior.once", true));

      std::normal_distribution<> dis_dur(40.0, 15.0);        // duration
      std::normal_distribution<> dis_vel(0.8, 0.35);         // agent max vel
      std::normal_distribution<> dis_detect_dist(4.5, 2.5);  // distance to detect the robot

      if (behavior == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
      {
        duration = dis_dur(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        vel = dis_vel(gen);
        vel = (vel < 0.4) ? 0.4 : vel;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.vel", vel));
        std::normal_distribution<> dis_dist(1.5, 0.3);  // distance to get close to the robot
        dist = dis_dist(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED)
      {
        duration = dis_dur(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
      {
        duration = dis_dur(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        vel = dis_vel(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.vel", vel));
        dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
        std::normal_distribution<> dis_force(20.0, 6.0);  // repulsive factor from the robot
        facOther = dis_force(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.other_force_factor", facOther));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
      {
        duration = dis_dur(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        // distance in front of the robot to put the robot goal
        std::normal_distribution<> dis_goal_dist(1.4, 0.3);
        dist = dis_goal_dist(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
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
      // this->set_parameter(rclcpp::Parameter(name + ".behavior.once", true));

      std::uniform_real_distribution<> dis_dur(25.0, 60.0);        // duration
      std::uniform_real_distribution<> dis_vel(0.6, 1.2);          // agent max vel
      std::uniform_real_distribution<> dis_detect_dist(2.0, 6.0);  // distance to detect the robot

      if (behavior == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
      {
        duration = dis_dur(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        vel = dis_vel(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.vel", vel));
        std::uniform_real_distribution<> dis_dist(1.0, 2.5);  // distance to get close to the robot
        dist = dis_dist(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED)
      {
        duration = dis_dur(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        dist = dis_detect_dist(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
      {
        duration = dis_dur(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        vel = dis_vel(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.vel", vel));
        dist = dis_detect_dist(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
        std::uniform_real_distribution<> dis_force(10.0, 25.0);  // repulsive factor from the robot
        facOther = dis_force(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.other_force_factor", facOther));
      }
      else if (behavior == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
      {
        duration = dis_dur(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.duration", duration));
        // distance in front of the robot to put the robot goal
        std::uniform_real_distribution<> dis_goal_dist(0.8, 1.9);
        dist = dis_goal_dist(gen);
        this->set_parameter(rclcpp::Parameter(name + ".behavior.dist", dist));
      }
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
    this->declare_parameter(std::string(name + ".goals"), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    rclcpp::Parameter array_goals = this->get_parameter(name + ".goals");
    auto goal_names = array_goals.as_string_array();
    for (auto g : goal_names)
    {
      RCLCPP_INFO(this->get_logger(), "\tGoal: %s", g.c_str());
      double gx = this->declare_parameter<double>(name + "." + g + ".x", 0.0);
      RCLCPP_INFO(this->get_logger(), "\t\tgx: %.2f", gx);
      double gy = this->declare_parameter<double>(name + "." + g + ".y", 0.0);
      RCLCPP_INFO(this->get_logger(), "\t\tgy: %.2f", gy);
      double gh = this->declare_parameter<double>(name + "." + g + ".h", 0.0);
      RCLCPP_INFO(this->get_logger(), "\t\tgh: %.2f", gh);
    }
  }

  //   try {
  //     pkg_shared_tree_dir_ =
  //         ament_index_cpp::get_package_share_directory("hunav_agent_manager");

  //   } catch (ament_index_cpp::PackageNotFoundError) {
  //     RCLCPP_ERROR(this->get_logger(),
  //                  "Package hunav_agent_manager not found in dir: %s!!!",
  //                  pkg_shared_tree_dir_.c_str());
  //   }
  //   pkg_shared_tree_dir_ = pkg_shared_tree_dir_ + "/behavior_trees/";
}

HunavLoader::~HunavLoader()
{
}

}  // namespace hunav

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::HunavLoader>());

  rclcpp::shutdown();
  return 0;
}