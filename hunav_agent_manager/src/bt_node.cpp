#include "hunav_agent_manager/bt_node.hpp"

// BT_REGISTER_NODES(factory) {
//   hunav_agent_manager::registerBTNodes(factory);
// }

namespace hunav_agent_manager {

using std::placeholders::_1;
using std::placeholders::_2;
// using std::placeholders::_3;

BTnode::BTnode() : Node("hunav_agent_manager") {
  RCLCPP_INFO(this->get_logger(), "Initializing %s node...", this->get_name());
  try {
    pkg_shared_tree_dir_ =
        ament_index_cpp::get_package_share_directory("hunav_agent_manager");

  } catch (ament_index_cpp::PackageNotFoundError) {
    RCLCPP_ERROR(this->get_logger(),
                 "Package hunav_agent_manager not found in dir: %s!!!",
                 pkg_shared_tree_dir_.c_str());
  }
  pkg_shared_tree_dir_ = pkg_shared_tree_dir_ + "/behavior_trees/";
  initialized_ = false;

  // node parameter declaration
  pub_tf_ = this->declare_parameter<bool>("publish_tf", true);
  pub_forces_ = this->declare_parameter<bool>("publish_sfm_forces", true);
  pub_agent_states_ =
      this->declare_parameter<bool>("publish_agent_states", true);

  prev_time_ = this->get_clock()->now();
  // btfunc_.init();

  registerBTNodes();

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  agents_srv_ = this->create_service<hunav_msgs::srv::ComputeAgents>(
      std::string("compute_agents"),
      std::bind(&BTnode::computeAgentsService, this, _1, _2));

  agent_srv_ = this->create_service<hunav_msgs::srv::ComputeAgent>(
      std::string("compute_agent"),
      std::bind(&BTnode::computeAgentService, this, _1, _2));

  move_agent_srv_ = this->create_service<hunav_msgs::srv::MoveAgent>(
      std::string("move_agent"),
      std::bind(&BTnode::moveAgentService, this, _1, _2));

  // agents_sub_ = this->create_subscription<hunav_msgs::msg::Agents>(
  //     "simulation_agents", 1, std::bind(&BTnode::agentsCallback, this, _1));

  // agent_robot_sub_ = this->create_subscription<hunav_msgs::msg::Agent>(
  //     "simulation_robot", 1,
  //     std::bind(&BTnode::agent_robot_callback, this, _1));

  if (pub_forces_) {
    forces_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "sfm_forces", 5);
  }
  if (pub_agent_states_) {
    state_publisher_ =
        this->create_publisher<hunav_msgs::msg::Agents>("human_states", 5);
  }
}

BTnode::~BTnode() {}

void BTnode::registerBTNodes() {

  // Register the conditions
  factory_.registerNodeType<hunav_agent_manager::TimeExpiredCondition>(
      "TimeExpiredCondition");

  BT::PortsList simple_port = {BT::InputPort<std::string>("agent_id")};
  BT::PortsList visibleports = {BT::InputPort<std::string>("agent_id"),
                                BT::InputPort<double>("distance")};
  factory_.registerSimpleCondition(
      "IsRobotVisible", std::bind(&BTfunctions::robotVisible, &btfunc_, _1),
      visibleports);

  factory_.registerSimpleCondition(
      "IsGoalReached", std::bind(&BTfunctions::goalReached, &btfunc_, _1),
      simple_port);

  // Register the actions
  BT::PortsList portsNav = {BT::InputPort<std::string>("agent_id"),
                            BT::InputPort<std::string>("time_step")};

  factory_.registerSimpleAction(
      "UpdateGoal", std::bind(&BTfunctions::updateGoal, &btfunc_, _1),
      simple_port);
  factory_.registerSimpleAction(
      "RegularNav", std::bind(&BTfunctions::regularNav, &btfunc_, _1),
      portsNav);
  factory_.registerSimpleAction(
      "SurprisedNav", std::bind(&BTfunctions::surprisedNav, &btfunc_, _1),
      portsNav);
  factory_.registerSimpleAction(
      "CuriousNav", std::bind(&BTfunctions::curiousNav, &btfunc_, _1),
      portsNav);
  factory_.registerSimpleAction(
      "ScaredNav", std::bind(&BTfunctions::scaredNav, &btfunc_, _1), portsNav);
  factory_.registerSimpleAction(
      "ThreateningNav", std::bind(&BTfunctions::threateningNav, &btfunc_, _1),
      portsNav);

  RCLCPP_INFO(this->get_logger(), "BT nodes registered");
}

void BTnode::initializeBehaviorTree(hunav_msgs::msg::Agents agents) {

  // root_ = std::make_unique<BT::ParallelNode>("root", 1, 1);
  // root_ = std::make_unique<BT::SequenceNode>("root");
  RCLCPP_INFO(this->get_logger(),
              "Initializing Behavior Trees of %lu agents...",
              agents.agents.size());

  for (unsigned int i = 0; i < agents.agents.size(); i++) {

    RCLCPP_INFO(this->get_logger(), "Agent %s behavior: %i",
                agents.agents[i].name.c_str(), agents.agents[i].behavior);

    // BT::Tree tree;
    // RCLCPP_INFO(this->get_logger(), "Setting id: %i", agents.agents[i].id);
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set<int>("id", agents.agents[i].id);
    blackboard->set<double>("dt", 0.0);

    // Check the agent behavior to create the proper behavior tree
    switch (agents.agents[i].behavior) {
    case hunav_msgs::msg::Agent::BEH_REGULAR:
      RCLCPP_INFO(this->get_logger(), "Loading BTRegularNav.xml tree");
      trees_[agents.agents[i].id] = factory_.createTreeFromFile(
          pkg_shared_tree_dir_ + "BTRegularNav.xml", blackboard);
      break;

    case hunav_msgs::msg::Agent::BEH_IMPASSIVE:
      // we load the regularNav tree since the impassive behavior
      // is taken into account in the ComputeForces method,
      // by adding the robot to the agent's obstacles.
      RCLCPP_INFO(this->get_logger(),
                  "Loading BTRegularNav.xml tree (with impassive behavior)");
      trees_[agents.agents[i].id] = factory_.createTreeFromFile(
          pkg_shared_tree_dir_ + "BTRegularNav.xml", blackboard);
      break;

    case hunav_msgs::msg::Agent::BEH_SURPRISED:
      RCLCPP_INFO(this->get_logger(), "Loading BTSurprisedNav.xml tree");
      trees_[agents.agents[i].id] = factory_.createTreeFromFile(
          pkg_shared_tree_dir_ + "BTSurprisedNav.xml", blackboard);
      break;

    case hunav_msgs::msg::Agent::BEH_SCARED:
      RCLCPP_INFO(this->get_logger(), "Loading BTScaredNav.xml tree");
      trees_[agents.agents[i].id] = factory_.createTreeFromFile(
          pkg_shared_tree_dir_ + "BTScaredNav.xml", blackboard);
      break;

    case hunav_msgs::msg::Agent::BEH_CURIOUS:
      RCLCPP_INFO(this->get_logger(), "Loading BTCuriousNav.xml tree");
      trees_[agents.agents[i].id] = factory_.createTreeFromFile(
          pkg_shared_tree_dir_ + "BTCuriousNav.xml", blackboard);
      break;

    case hunav_msgs::msg::Agent::BEH_THREATENING:
      RCLCPP_INFO(this->get_logger(), "Loading BTThreatening.xml tree");
      trees_[agents.agents[i].id] = factory_.createTreeFromFile(
          pkg_shared_tree_dir_ + "BTThreateningNav.xml", blackboard);
      break;

    default:
      RCLCPP_WARN(this->get_logger(),
                  "Behavior of agent %s not defined! Using regular behavior",
                  agents.agents[i].name.c_str());
      RCLCPP_INFO(this->get_logger(), "Loading default tree");
      trees_[agents.agents[i].id] = factory_.createTreeFromFile(
          pkg_shared_tree_dir_ + "BTRegularNav.xml", blackboard);
    }
    RCLCPP_INFO(this->get_logger(),
                "Behavior Tree for agent %s [id:%i] loaded!",
                agents.agents[i].name.c_str(), agents.agents[i].id);

    // Set the id of the agent
    // tree_.rootBlackboard()->set<std::string>("id",
    // std::to_string(agents.agents[i].id));
    // tree_.rootBlackboard()->set<int>("id", agents.agents[i].id);

    // // This logger prints state changes on console
    BT::StdCoutLogger logger_cout(trees_[agents.agents[i].id]);
    // // This logger saves state changes on file
    // std::string filename = "bt_trace_" + std::to_string(agents.agents[i].id);
    // BT::FileLogger logger_file(tree, (filename + ".fbl").c_str());
    // // This logger stores the execution time of each node
    // BT::MinitraceLogger logger_minitrace(tree, (filename + ".json").c_str());

#ifdef ZMQ_FOUND
    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree);
#endif

    // root_->addChild(trees_[trees_.size() - 1].rootNode());
    // BT::printTreeRecursively(root_.get());
    BT::printTreeRecursively(trees_[agents.agents[i].id].rootNode());
  }
  // This logger prints state changes on console
  // BT::StdCoutLogger logger_cout(root_.get());
  // // This logger saves state changes on file
  // std::string filename = "bt_trace_" + std::to_string(agents.agents[i].id);
  // BT::FileLogger logger_file(tree, (filename + ".fbl").c_str());
  // // This logger stores the execution time of each node
  // BT::MinitraceLogger logger_minitrace(tree, (filename + ".json").c_str());
  RCLCPP_INFO(this->get_logger(), "Behavior trees succesfully initiated!");
}

BT::NodeStatus BTnode::tree_tick(double dt) {
  // RCLCPP_INFO(this->get_logger(), "Ticking the tree root!");
  BT::NodeStatus status;
  std::unordered_map<int, BT::Tree>::iterator itr;
  for (itr = trees_.begin(); itr != trees_.end(); itr++) {
    // RCLCPP_INFO(this->get_logger(), "Ticking the tree id %i", itr->first);
    itr->second.rootBlackboard()->set<double>("dt", dt);
    status = itr->second.tickRoot();
  }
  return status;
}

BT::NodeStatus BTnode::tree_tick(int id) {
  // RCLCPP_INFO(this->get_logger(), "Ticking the tree id %i!", id);
  BT::NodeStatus status = trees_[id].tickRoot();
  return status;
}

BT::NodeStatus BTnode::tree_tick(int id, double dt) {
  // RCLCPP_INFO(this->get_logger(), "Ticking the tree id %i!", id);
  trees_[id].rootBlackboard()->set<double>("dt", dt);
  // RCLCPP_INFO(this->get_logger(), "After setting dt:%.4f!", dt);
  BT::NodeStatus status = trees_[id].tickRoot();
  // RCLCPP_INFO(this->get_logger(), "After ticking the tree!");
  return status;
}

void BTnode::computeAgentsService(
    const std::shared_ptr<hunav_msgs::srv::ComputeAgents::Request> request,
    std::shared_ptr<hunav_msgs::srv::ComputeAgents::Response> response) {

  auto ro = std::make_shared<hunav_msgs::msg::Agent>(request->robot);
  auto ag = std::make_shared<hunav_msgs::msg::Agents>(request->current_agents);

  // Update the internal agent states with the
  // received data from the simulator
  btfunc_.updateAllAgents(ro, ag);

  if (!initialized_) {
    RCLCPP_INFO(this->get_logger(), "First service call received!");
    RCLCPP_INFO(this->get_logger(), "robot pose x:%.2f, y:%.2f, th:%.2f",
                ro->position.position.x, ro->position.position.y, ro->yaw);
    RCLCPP_INFO(this->get_logger(), "Agents received: %li", ag->agents.size());

    initializeBehaviorTree(request->current_agents);
    response->updated_agents = btfunc_.getUpdatedAgents();
    prev_time_ = rclcpp::Time(ag->header.stamp);
    initialized_ = true;
    return;
  }

  // rclcpp::Time t = this->get_clock()->now();
  rclcpp::Time t = rclcpp::Time(ag->header.stamp);
  if (pub_tf_)
    publish_agents_tf(t, ro, ag);
  if (pub_forces_)
    publish_agents_forces(t, ag);
  if (pub_agent_states_)
    publish_agent_states(t, ag);

  double time_step_secs =
      (rclcpp::Time(ag->header.stamp) - prev_time_).seconds(); // * 5.0;
  // time_step_secs = 0.05;

  // RCLCPP_INFO(this->get_logger(), "Time step computed: %.4f",
  // time_step_secs);

  // Call the ticks of the behavior trees (they must update the
  // sfm_agents_)

  // if (time_step_secs > 0.008) {
  // Call the ticks of the behavior trees (they must update the
  // sfm_agents_)
  BT::NodeStatus status = tree_tick(time_step_secs);
  prev_time_ = rclcpp::Time(ag->header.stamp);
  //}

  response->updated_agents = btfunc_.getUpdatedAgents();
}

void BTnode::moveAgentService(
    const std::shared_ptr<hunav_msgs::srv::MoveAgent::Request> request,
    std::shared_ptr<hunav_msgs::srv::MoveAgent::Response> response) {

  auto ro = std::make_shared<hunav_msgs::msg::Agent>(request->robot);
  auto ag = std::make_shared<hunav_msgs::msg::Agents>(request->current_agents);

  // Update the internal agent states with the
  // received data from the simulator
  RCLCPP_INFO(this->get_logger(), "Service call received agent id %i",
              request->agent_id);
  btfunc_.updateAllAgents(ro, ag);
  // RCLCPP_INFO(this->get_logger(), "Agents updated!");

  if (!initialized_) {
    RCLCPP_INFO(this->get_logger(), "First service call received!");
    RCLCPP_INFO(this->get_logger(), "robot pose x:%.2f, y:%.2f, th:%.2f",
                ro->position.position.x, ro->position.position.y, ro->yaw);
    RCLCPP_INFO(this->get_logger(), "Agents received: %li", ag->agents.size());

    initializeBehaviorTree(request->current_agents);
    response->updated_agent = btfunc_.getUpdatedAgent(request->agent_id);
    prev_time_ = rclcpp::Time(ag->header.stamp);
    initialized_ = true;
    return;
  }

  // rclcpp::Time t = this->get_clock()->now();
  rclcpp::Time t = rclcpp::Time(ag->header.stamp);
  if (pub_tf_)
    publish_agents_tf(t, ro, ag);
  if (pub_forces_)
    publish_agents_forces(t, ag);
  if (pub_agent_states_)
    publish_agent_states(t, ag);

  double time_step_secs =
      (rclcpp::Time(ag->header.stamp) - prev_time_).seconds();
  // time_step_secs = 0.1;

  // RCLCPP_INFO(this->get_logger(), "Time step computed: %.4f",
  // time_step_secs);

  // we do not tick the tree if the frequency is higher than 100Hz approx
  // if (time_step_secs > 0.008) {
  // Call the ticks of the behavior trees (they must update the
  // sfm_agents_)
  BT::NodeStatus status = tree_tick(request->agent_id, time_step_secs);
  prev_time_ = rclcpp::Time(ag->header.stamp);
  //}

  response->updated_agent = btfunc_.getUpdatedAgent(request->agent_id);
}

void BTnode::computeAgentService(
    const std::shared_ptr<hunav_msgs::srv::ComputeAgent::Request> request,
    std::shared_ptr<hunav_msgs::srv::ComputeAgent::Response> response) {

  // rclcpp::Rate loop_rate(40);
  // while (!btfunc_.ok()) {
  // loop_rate.sleep();
  //}
  BT::NodeStatus status = tree_tick(request->id);
  response->updated_agent = btfunc_.getUpdatedAgent(request->id);
}

// void BTnode::agentsCallback(const hunav_msgs::msg::Agents &msg) {

//   auto ag = std::make_shared<hunav_msgs::msg::Agents>(msg);
//   if (!initialized_) {
//     RCLCPP_INFO(this->get_logger(), "First agent callback received!");
//     RCLCPP_INFO(this->get_logger(), "Agents received: %li",
//     ag->agents.size()); hunav_msgs::msg::Agents ags = msg;
//     ags.agents.pop_back();
//     initializeBehaviorTree(ags);
//     initialized_ = true;
//   }
//   btfunc_.updateAgentsAndRobot(ag);
// }

// void BTnode::agentRobotCallback(const hunav_msgs::msg::Agent &msg) {
//   auto ro = std::make_shared<hunav_msgs::msg::Agent>(msg);
//   btfunc_.updateAgentRobot(ro);
// }

void BTnode::publish_agents_tf(rclcpp::Time t,
                               const hunav_msgs::msg::Agent::SharedPtr robot,
                               const hunav_msgs::msg::Agents::SharedPtr msg) {

  // rclcpp::Time now = this->get_clock()->now();
  // publish robot TF
  geometry_msgs::msg::TransformStamped tr;
  tr.header.stamp = t;
  tr.header.frame_id = msg->header.frame_id;
  tr.child_frame_id = robot->name.c_str();
  tr.transform.translation.x = robot->position.position.x;
  tr.transform.translation.y = robot->position.position.y;
  tr.transform.translation.z = robot->position.position.z;
  tr.transform.rotation = robot->position.orientation;
  // Send the transformation
  tf_broadcaster_->sendTransform(tr);

  for (const auto &a : msg->agents) {
    geometry_msgs::msg::TransformStamped tr2;
    tr2.header.stamp = t;
    tr2.header.frame_id = msg->header.frame_id;
    tr2.child_frame_id = a.name.c_str();
    tr2.transform.translation.x = a.position.position.x;
    tr2.transform.translation.y = a.position.position.y;
    tr2.transform.translation.z = a.position.position.z;
    tr2.transform.rotation = a.position.orientation;
    // Send the transformation
    tf_broadcaster_->sendTransform(tr2);
  }
}

void BTnode::publish_agent_states(
    rclcpp::Time t, const hunav_msgs::msg::Agents::SharedPtr msg) {

  state_publisher_->publish(*msg);
}

void BTnode::publish_agents_forces(
    rclcpp::Time t, const hunav_msgs::msg::Agents::SharedPtr msg) {
  visualization_msgs::msg::MarkerArray markers;
  for (const auto &a : msg->agents) {
    sfm::Forces frs = getAgentForces(a.id);
    publishForceMarker(a.id + 1, a.name, msg->header.frame_id, t,
                       a.position.position, getColor(1, 0, 0, 1),
                       frs.obstacleForce,
                       markers); // RED
    publishForceMarker(a.id + 2, a.name, msg->header.frame_id, t,
                       a.position.position, getColor(0, 0, 1, 1),
                       frs.socialForce,
                       markers); // BLUE
    // publishForceMarker(2, getColor(0, 1, 1, 1), robot_.forces.groupForce,
    //                   markers);
    publishForceMarker(a.id + 3, a.name, msg->header.frame_id, t,
                       a.position.position, getColor(0, 1, 0, 1),
                       frs.desiredForce,
                       markers); // GREEN
    publishForceMarker(a.id + 4, a.name, msg->header.frame_id, t,
                       a.position.position, getColor(1, 1, 1, 1),
                       frs.globalForce,
                       markers); // WHITE
    // publishForceMarker(5, getColor(1, 1, 0, 1), robot_.velocity, markers);
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = t;
    marker.ns = a.name + "/behavior";
    marker.id = a.id;
    marker.action =
        visualization_msgs::msg::Marker::ADD; // force.norm() > 1e-4 ? 0 : 2;
    marker.color = getColor(1, 1, 1, 1);
    marker.lifetime = rclcpp::Duration(1, 0);
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.position = a.position.position;
    marker.pose.position.z = a.position.position.z + 1.0;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, frs.globalForce.angle().toRadian());
    marker.pose.orientation = tf2::toMsg(myQuaternion);
    marker.text = a.name;
    switch (a.behavior) {
    case hunav_msgs::msg::Agent::BEH_REGULAR:
      marker.text = marker.text + "/REGULAR";
      break;
    case hunav_msgs::msg::Agent::BEH_IMPASSIVE:
      marker.text = marker.text + "/IMPASSIVE";
      break;
    case hunav_msgs::msg::Agent::BEH_SURPRISED:
      marker.text = marker.text + "/SURPRISED";
      break;
    case hunav_msgs::msg::Agent::BEH_SCARED:
      marker.text = marker.text + "/SCARED";
      break;
    case hunav_msgs::msg::Agent::BEH_CURIOUS:
      marker.text = marker.text + "/CURIOUS";
      break;
    case hunav_msgs::msg::Agent::BEH_THREATENING:
      marker.text = marker.text + "/THREATENING";
      break;
    default:
      marker.text = marker.text + "/REGULAR";
    }
    markers.markers.push_back(marker);
  }
  forces_publisher_->publish(markers);
}

void BTnode::publishForceMarker(unsigned index, std::string name,
                                std::string frame, rclcpp::Time t,
                                geometry_msgs::msg::Point p,
                                const std_msgs::msg::ColorRGBA &color,
                                const utils::Vector2d &force,
                                visualization_msgs::msg::MarkerArray &markers) {
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.header.frame_id = frame;
  marker.header.stamp = t;
  marker.ns = name + "/robot_forces";
  marker.id = index;
  marker.action = force.norm() > 1e-4 ? 0 : 2;
  marker.color = color;
  marker.lifetime = rclcpp::Duration(1, 0);
  marker.scale.x = std::max(1e-4, force.norm());
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.pose.position = p;
  marker.pose.position.z = 0;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, force.angle().toRadian());
  marker.pose.orientation = tf2::toMsg(myQuaternion);

  markers.markers.push_back(marker);
}

std_msgs::msg::ColorRGBA BTnode::getColor(double r, double g, double b,
                                          double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

} // namespace hunav_agent_manager
