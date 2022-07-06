#include "hunav_agent_manager/hunav_loader.hpp"

namespace hunav {

// HunavLoader::HunavLoader()
//     : Node("hunav_loader",
//            rclcpp::NodeOptions()
//                .allow_undeclared_parameters(true)
//                .automatically_declare_parameters_from_overrides(true)) {
HunavLoader::HunavLoader() : Node("hunav_loader") {

  /* node parameter declaration */
  // std::string base_world = this->declare_parameter<std::string>(
  //    "base_world", std::string("empty.world"));

  std::string map =
      this->declare_parameter<std::string>("map", std::string("map.yaml"));

  this->declare_parameter("agents");
  rclcpp::Parameter array_agents = this->get_parameter("agents");
  auto agent_names = array_agents.as_string_array();

  //   rclcpp::Parameter agents_array;
  //   this->get_parameter_or("agents", agents_array,
  //                          rclcpp::Parameter("agents", "[]"));

  RCLCPP_INFO(this->get_logger(), "map params: %s", map.c_str());
  for (auto name : agent_names) {
    RCLCPP_INFO(this->get_logger(), "Agent name: %s", name.c_str());
    int id = this->declare_parameter<int>(name + ".id", -1);
    RCLCPP_INFO(this->get_logger(), "\tid: %i", id);
    int skin = this->declare_parameter<int>(name + ".skin", -1);
    RCLCPP_INFO(this->get_logger(), "\tskin: %i", skin);
    int behavior = this->declare_parameter<int>(name + ".behavior", -1);
    RCLCPP_INFO(this->get_logger(), "\tbehavior: %i", behavior);
    int group_id = this->declare_parameter<int>(name + ".group_id", -1);
    RCLCPP_INFO(this->get_logger(), "\tgroup_id: %i", group_id);
    double max_vel = this->declare_parameter<double>(name + ".max_vel", 1.0);
    RCLCPP_INFO(this->get_logger(), "\tmax_vel: %.2f", max_vel);
    double radius = this->declare_parameter<double>(name + ".radius", 0.35);
    RCLCPP_INFO(this->get_logger(), "\tradius: %.2f", radius);
    double posex = this->declare_parameter<double>(name + ".init_pose.x", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tposex: %.2f", posex);
    double posey = this->declare_parameter<double>(name + ".init_pose.y", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tposey: %.2f", posey);
    double posez = this->declare_parameter<double>(name + ".init_pose.z", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tposez: %.2f", posez);
    double poseh = this->declare_parameter<double>(name + ".init_pose.h", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tposeh: %.2f", poseh);
    double goal_radius =
        this->declare_parameter<double>(name + ".goal_radius", 0.0);
    RCLCPP_INFO(this->get_logger(), "\tgoal_radius: %.2f", goal_radius);
    bool cyclic = this->declare_parameter<bool>(name + ".cyclic_goals", true);
    RCLCPP_INFO(this->get_logger(), "\tcyclic_goals: %i", (int)cyclic);
    // Goals
    this->declare_parameter(name + ".goals");
    rclcpp::Parameter array_goals = this->get_parameter(name + ".goals");
    auto goal_names = array_goals.as_string_array();
    for (auto g : goal_names) {
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

HunavLoader::~HunavLoader() {}

} // namespace hunav

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::HunavLoader>());

  rclcpp::shutdown();
  return 0;
}