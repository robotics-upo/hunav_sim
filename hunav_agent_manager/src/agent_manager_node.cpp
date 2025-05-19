#include "hunav_agent_manager/bt_node.hpp"
#include "hunav_agent_manager/agent_say_node.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  auto bt_node = std::make_shared<hunav::BTnode>();
  auto agent_say_node = std::make_shared<hunav::AgentSayNode>();
  hunav::AgentSayNode::setInstance(agent_say_node);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bt_node);
  executor.add_node(agent_say_node);
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
