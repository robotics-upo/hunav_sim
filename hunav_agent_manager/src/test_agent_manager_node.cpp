
#include "hunav_msgs/msg/agent.hpp"
#include "hunav_msgs/msg/agents.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include "hunav_msgs/srv/is_robot_visible.hpp"
#include "hunav_msgs/srv/compute_agents.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

hunav_msgs::msg::Agent fillInitialRobot() {
  hunav_msgs::msg::Agent r;
  r.id = -1;
  r.type = r.ROBOT;
  r.behavior = r.BEH_REGULAR;
  r.name = "PepeRobot";
  r.group_id = -1;
  r.position.position.x = 2.5; // geometry_msgs/Pose
  r.position.position.y = 0.0; // 1.5;
  r.position.position.z = 0.0;
  r.yaw = 0.0;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, r.yaw);
  r.position.orientation = tf2::toMsg(myQuaternion);
  r.velocity.linear.x = 0.; // geometry_msgs/Twist
  r.velocity.linear.y = 0.;
  r.velocity.angular.z = 0.;
  r.desired_velocity = 1.;
  r.radius = 0.4;
  r.linear_vel = sqrt(r.velocity.linear.x * r.velocity.linear.x +
                      r.velocity.linear.y * r.velocity.linear.y);
  r.angular_vel = r.velocity.angular.z;
  r.cyclic_goals = false;
  // r.current_goal = 0;
  return r;
}

hunav_msgs::msg::Agents fillInitialAgents() {
  hunav_msgs::msg::Agents agents;

  // Agent 1
  hunav_msgs::msg::Agent agent;
  agent.id = 0;
  agent.type = agent.PERSON;
  agent.behavior = agent.BEH_REGULAR;
  agent.name = "Pepe";
  agent.group_id = -1;
  agent.position.position.x = 0.5; // geometry_msgs/Pose
  agent.position.position.y = 0.5;
  agent.position.position.z = 0.0;
  agent.yaw = 1.57;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, agent.yaw);
  agent.position.orientation = tf2::toMsg(myQuaternion);
  agent.velocity.linear.x = 0.3; // geometry_msgs/Twist
  agent.velocity.linear.y = 0.3;
  agent.velocity.angular.z = 0.0;
  agent.desired_velocity = 0.75;
  agent.radius = 0.4;
  agent.linear_vel = sqrt(agent.velocity.linear.x * agent.velocity.linear.x +
                          agent.velocity.linear.y * agent.velocity.linear.y);
  agent.angular_vel = agent.velocity.angular.z;
  geometry_msgs::msg::Pose g1;
  g1.position.x = 1.5;
  g1.position.y = 2.5;
  agent.goals.push_back(g1); // geometry_msgs/Pose[]
  geometry_msgs::msg::Pose g2;
  g2.position.x = 4.5;
  g2.position.y = 4.0;
  agent.goals.push_back(g2);
  agent.cyclic_goals = true;
  // agent.current_goal = 0;

  agents.agents.push_back(agent);

  // Agent 2
  hunav_msgs::msg::Agent agent2;
  agent2.id = 1;
  agent2.type = agent.PERSON;
  agent2.behavior = agent.BEH_SURPRISED;
  agent2.name = "Juanito";
  agent2.group_id = -1;
  agent2.position.position.x = 4.5; // geometry_msgs/Pose
  agent2.position.position.y = 1.0;
  agent2.position.position.z = 0.0;
  agent2.yaw = 0.0;
  myQuaternion.setRPY(0, 0, agent2.yaw);
  agent2.position.orientation = tf2::toMsg(myQuaternion);
  agent2.velocity.linear.x = 0.0; // geometry_msgs/Twist
  agent2.velocity.linear.y = 0.0;
  agent2.velocity.angular.z = 0.0;
  agent2.desired_velocity = 0.8;
  agent2.radius = 0.4;
  agent2.linear_vel = sqrt(agent2.velocity.linear.x * agent2.velocity.linear.x +
                           agent2.velocity.linear.y * agent2.velocity.linear.y);
  agent2.angular_vel = agent2.velocity.angular.z;
  // geometry_msgs::msg::Pose g1;
  g1.position.x = 4.5;
  g1.position.y = 2.5;
  agent2.goals.push_back(g1); // geometry_msgs/Pose[]
  // geometry_msgs::msg::Pose g2;
  g2.position.x = 3.0;
  g2.position.y = 4.0;
  agent2.goals.push_back(g2);
  agent2.cyclic_goals = true;
  // agent2.current_goal = 0;

  agents.agents.push_back(agent2);

  return agents;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("agent_client");
  rclcpp::Client<hunav_msgs::srv::ComputeAgents>::SharedPtr client =
      node->create_client<hunav_msgs::srv::ComputeAgents>("compute_agents");

  auto request =
      std::make_shared<hunav_msgs::srv::ComputeAgents::Request>();

  hunav_msgs::msg::Agent robot = fillInitialRobot();
  hunav_msgs::msg::Agents agents = fillInitialAgents();
  agents.header.frame_id = "odom";
  agents.header.stamp = node->get_clock()->now();
  request->robot = robot;
  request->current_agents = agents;

  // rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
  //     std::chrono::milliseconds(500),
  //     std::bind(&FibonacciActionClient::send_goal, node));

  // Do a loop to call the service every 0.5 seconds
  rclcpp::Rate loop_rate(2); // 2 Hz
  int count = 0;
  while (count < 120 && rclcpp::ok()) {

    // Call the service
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // Update the agents position
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service result received!");
      hunav_msgs::msg::Agents upd_agents = result.get()->updated_agents;
      for (unsigned int i = 0; i < upd_agents.agents.size(); i++) {
        agents.agents[i] = upd_agents.agents[i];
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Agent id %i, new position x: %.2f, y: %.2f",
                    agents.agents[i].id, agents.agents[i].position.position.x,
                    agents.agents[i].position.position.y);
      }
      agents.header.stamp = node->get_clock()->now();
      request->current_agents = agents;

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service compute_agents");
      break;
    }
    count++;
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
