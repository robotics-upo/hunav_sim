#include "hunav_agent_manager/bt_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sys/stat.h>
#include <cstdlib>
#include <rclcpp/node_options.hpp>
#include <rcpputils/split.hpp>
#include <sstream>
#include <vector>
#include <algorithm>

// BT_REGISTER_NODES(factory) {
//   hunav_agent_manager::registerBTNodes(factory);
// }

namespace hunav
{

  using std::placeholders::_1;
  using std::placeholders::_2;
  // using std::placeholders::_3;

  std::string BTnode::share_to_src_path(const std::string& share_path) 
  {
    // Example:
    // input: /home/hunav_gz_classic_ws/install/hunav_gazebo_wrapper/share/hunav_gazebo_wrapper
    // output: /home/hunav_gz_classic_ws/src/hunav_gazebo_wrapper
    
    std::vector<std::string> parts;
    std::stringstream ss(share_path);
    std::string item;
    while (std::getline(ss, item, '/')) {
        if (!item.empty()) parts.push_back(item);
    }

    auto it = std::find(parts.begin(), parts.end(), "install");
    if (it == parts.end() || (it + 1) == parts.end()) {
        RCLCPP_WARN(this->get_logger(),
                    "The path does not have the expected structure ../install/share/package_name: %s",
                    share_path.c_str());
        return share_path;
    }
    
    size_t install_idx = std::distance(parts.begin(), it);
    std::string pkg_name = parts[install_idx + 1];

    std::ostringstream src_path;
    for (size_t i = 0; i < install_idx; ++i) {
        src_path << "/" << parts[i];
    }
    src_path << "/src/" << pkg_name;
    // src_path << "/src/"; // Temporary for isaac


    RCLCPP_DEBUG(this->get_logger(), 
                 "Converted share path to src: %s → %s", 
                 share_path.c_str(), src_path.str().c_str());

    return src_path.str();
  }

  BTnode::BTnode()
  : Node("hunav_agent_manager")
  {
    RCLCPP_INFO(get_logger(), "Initializing %s node...", get_name());
  initialized_ = false;

  // 1) Parameters local to this node
  pub_tf_     = this->declare_parameter<bool>("publish_tf", true);
  pub_forces_ = this->declare_parameter<bool>("publish_sfm_forces", true);

  // 2) Create service client to get parameters from hunav_loader
  get_parameters_client_ = this->create_client<hunav_msgs::srv::GetParameters>("/get_parameters");

  // 3) Get parameters from hunav_loader node
  if (!getParametersFromLoader()) {
    RCLCPP_ERROR(get_logger(), "Failed to get parameters from hunav_loader. Using defaults.");
    // Set default values
    pub_people_     = true;
    map_name_       = "warehouse";
    simulator_name_ = "Gazebo Classic";
    yaml_base_name_ = "warehouse_agents";
  }

  RCLCPP_INFO(get_logger(),
    "Parameters from loader: map=%s, simulator=%s, yaml_base_name=%s, publish_people=%s",
    map_name_.c_str(),
    simulator_name_.c_str(),
    yaml_base_name_.c_str(),
    pub_people_ ? "true" : "false");

  // 4) Hand global goals to BT logic:
  btfunc_.setGlobalGoals(global_goals_);

    // Set the base directory for behavior trees
    {
      std::string package_name;

      if (simulator_name_ == "Gazebo Classic") {
        package_name = "hunav_gazebo_wrapper";
      }
      else if(simulator_name_ == "Gazebo Fortress") {
        package_name = "hunav_gazebo_fortress_wrapper";
      }
      else if (simulator_name_ == "Isaac Sim") {
        package_name = "hunav_isaac_wrapper";
      }
      else { // Webots
        package_name = "hunav_webots_wrapper";
      }
      
      try {
        std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
        std::string src_dir = share_to_src_path(share_dir);
        bt_dir_base_ = src_dir + "/behavior_trees";
        RCLCPP_INFO(this->get_logger(),
                    "Found ROS2 package '%s', behavior trees will be loaded from: %s",
                    package_name.c_str(), bt_dir_base_.c_str());
      }
      catch (const ament_index_cpp::PackageNotFoundError &e) {
        RCLCPP_WARN(this->get_logger(),
                    "ROS2 package '%s' not found,",
                    package_name.c_str());
      }
    }

    prev_time_ = this->get_clock()->now();

    registerBTNodes();
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    agents_srv_ = this->create_service<hunav_msgs::srv::ComputeAgents>(
      "compute_agents", std::bind(&BTnode::computeAgentsService, this, _1, _2));
    agent_srv_ = this->create_service<hunav_msgs::srv::ComputeAgent>(
      "compute_agent", std::bind(&BTnode::computeAgentService, this, _1, _2));
    move_agent_srv_ = this->create_service<hunav_msgs::srv::MoveAgent>(
      "move_agent", std::bind(&BTnode::moveAgentService, this, _1, _2));
    reset_srv_ = this->create_service<hunav_msgs::srv::ResetAgents>(
      "reset_agents", std::bind(&BTnode::resetAgentsService, this, _1, _2));

    if (pub_forces_) {
      forces_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "sfm_forces", 5);
    }
    human_state_publisher_ = this->create_publisher<hunav_msgs::msg::Agents>(
      "human_states", 1);
    robot_state_publisher_ = this->create_publisher<hunav_msgs::msg::Agent>(
      "robot_states", 1);
    if (pub_people_) {
      people_publisher_ = this->create_publisher<people_msgs::msg::People>(
        "people", 1);
    }
  }

  BTnode::~BTnode()
  {
  }

  void BTnode::registerBTNodes()
  {
    // Register the conditions
    factory_.registerNodeType<hunav::TimeExpiredCondition>("TimeExpiredCondition");

    BT::PortsList simple_port = {BT::InputPort<int>("agent_id")};
    BT::PortsList visibleports = {BT::InputPort<int>("agent_id"), BT::InputPort<double>("distance")};
    factory_.registerSimpleCondition("IsRobotVisible", std::bind(&BTfunctions::robotVisible, &btfunc_, _1), visibleports);

    factory_.registerSimpleCondition("IsGoalReached", std::bind(&BTfunctions::goalReached, &btfunc_, _1), simple_port);

    BT::PortsList agent_visible_ports = {
        BT::InputPort<int>("observer_id"),
        BT::InputPort<int>("agent_id"),
        BT::InputPort<double>("distance"),
        BT::InputPort<double>("field_of_view")};
    BT::PortsList proximity_ports = {
        BT::InputPort<int>("agent_id"),
        BT::InputPort<double>("threshold", 1.0, "Distance threshold in meters")};

    BT::PortsList random_ports = {
        BT::InputPort<double>("probability", 0.3, "Probability to return success")};
    BT::PortsList facing_ports = {BT::InputPort<int>("agent_id")};
    BT::PortsList time_since_ports = {
        BT::InputPort<int>("agent_id"),
        BT::InputPort<double>("wait_time", 5.0, "Time in seconds since last interaction")};
    BT::PortsList pos_ports = {
        BT::InputPort<int>("agent_id"),
        BT::InputPort<int>("goal_id"),
        BT::InputPort<double>("tolerance", 0.1, "Tolerance for reaching the target")};
    BT::PortsList proximity_agent_ports = {
        BT::InputPort<int>("observer_id"),
        BT::InputPort<int>("target_agent_id"),
        BT::InputPort<double>("threshold", 1.0, "Distance threshold in meters")};

    factory_.registerSimpleCondition("RandomChanceCondition",
                                     std::bind(&BTfunctions::randomChance, &btfunc_, _1),
                                     random_ports);
    factory_.registerSimpleCondition("IsRobotFacingAgent",
                                     std::bind(&BTfunctions::robotFacingAgent, &btfunc_, _1),
                                     simple_port);
    factory_.registerSimpleCondition("IsAgentVisible",
                                     std::bind(&BTfunctions::agentVisible, &btfunc_, _1),
                                     agent_visible_ports);
    factory_.registerSimpleCondition("IsRobotClose",
                                     std::bind(&BTfunctions::isRobotClose, &btfunc_, _1),
                                     proximity_ports);
    factory_.registerSimpleCondition("IsAtPosition",
                                     std::bind(&BTfunctions::isAtPosition, &btfunc_, _1),
                                     pos_ports);
    factory_.registerSimpleCondition("IsAgentClose",
                                     std::bind(&BTfunctions::isAgentClose, &btfunc_, _1),
                                     proximity_agent_ports);

    // Register the actions
    BT::PortsList reg_nav_ports = {BT::InputPort<int>("agent_id"), BT::InputPort<double>("time_step")};

    BT::PortsList sur_nav_ports = {BT::InputPort<int>("agent_id"), BT::InputPort<double>("time_step"),
                                   BT::InputPort<double>("beh_duration"), BT::InputPort<bool>("only_once")};

    BT::PortsList cur_nav_ports = {BT::InputPort<int>("agent_id"), BT::InputPort<double>("time_step"),
                                   BT::InputPort<double>("beh_duration"), BT::InputPort<bool>("only_once"),
                                   BT::InputPort<double>("agent_vel"), BT::InputPort<double>("stop_distance")};

    BT::PortsList sca_nav_ports = {BT::InputPort<int>("agent_id"), BT::InputPort<double>("time_step"),
                                   BT::InputPort<double>("beh_duration"), BT::InputPort<bool>("only_once"),
                                   BT::InputPort<double>("runaway_vel"), BT::InputPort<double>("scary_force_factor")};

    BT::PortsList thre_nav_ports = {BT::InputPort<int>("agent_id"), BT::InputPort<double>("time_step"),
                                    BT::InputPort<double>("beh_duration"), BT::InputPort<bool>("only_once"),
                                    BT::InputPort<double>("goal_dist")};

    factory_.registerSimpleAction("UpdateGoal", std::bind(&BTfunctions::updateGoal, &btfunc_, _1), simple_port);
    factory_.registerSimpleAction("RegularNav", std::bind(&BTfunctions::regularNav, &btfunc_, _1), reg_nav_ports);
    factory_.registerSimpleAction("SurprisedNav", std::bind(&BTfunctions::surprisedNav, &btfunc_, _1), sur_nav_ports);
    factory_.registerSimpleAction("CuriousNav", std::bind(&BTfunctions::curiousNav, &btfunc_, _1), cur_nav_ports);
    factory_.registerSimpleAction("ScaredNav", std::bind(&BTfunctions::scaredNav, &btfunc_, _1), sca_nav_ports);
    factory_.registerSimpleAction("ThreateningNav", std::bind(&BTfunctions::threateningNav, &btfunc_, _1),
                                  thre_nav_ports);

    BT::PortsList find_target_ports = {BT::InputPort<int>("agent_id"), BT::OutputPort<int>("target_agent_id")};
    BT::PortsList say_ports = {
        BT::InputPort<int>("agent_id"),
        BT::InputPort<std::string>("message")};
    BT::PortsList look_at_agent_ports = {
        BT::InputPort<int>("observer_id"),
        BT::InputPort<int>("target_id")};
    BT::PortsList look_at_point_ports = {
        BT::InputPort<int>("agent_id"),
        BT::InputPort<double>("target_x"),
        BT::InputPort<double>("target_y")};
    BT::PortsList set_group_id_ports = {
        BT::InputPort<int>("agent_id"),
        BT::InputPort<int>("group_id")};
    BT::PortsList go_to_ports = {
        BT::InputPort<int>("agent_id"),
        BT::InputPort<double>("target_x"),
        BT::InputPort<double>("target_y"),
        BT::InputPort<int>("goal_id")};

    factory_.registerSimpleAction("FindNearestAgent",
                                  std::bind(&BTfunctions::findNearestAgent, &btfunc_, _1), find_target_ports);
    factory_.registerSimpleAction("SaySomething",
                                  std::bind(&BTfunctions::saySomething, &btfunc_, _1), say_ports);
    factory_.registerSimpleAction("SetGroupId",
                                  std::bind(&BTfunctions::setGroupId, &btfunc_, _1), set_group_id_ports);
    factory_.registerSimpleAction("SetGoal",
                                  std::bind(&BTfunctions::setGoal, &btfunc_, _1), go_to_ports);
    factory_.registerSimpleAction("StopMovement",
                                  std::bind(&BTfunctions::stopMovement, &btfunc_, _1), simple_port);
    factory_.registerSimpleAction("ResumeMovement",
                                  std::bind(&BTfunctions::resumeMovement, &btfunc_, _1), simple_port);

    // Registration of StatefulAction nodes for timed actions
    factory_.registerNodeType<hunav::StopAndWaitTimerActionNode>("StopAndWaitTimerAction");
    factory_.registerNodeType<hunav::ConversationFormationNode>("ConversationFormation");
    factory_.registerNodeType<hunav::GoToNode>("GoTo");
    factory_.registerNodeType<hunav::ApproachAgentNode>("ApproachAgent");
    factory_.registerNodeType<hunav::FollowAgentNode>("FollowAgent");
    factory_.registerNodeType<hunav::ApproachRobotNode>("ApproachRobot");
    factory_.registerNodeType<hunav::BlockRobotNode>("BlockRobot");
    factory_.registerNodeType<hunav::AvoidRobotNode>("AvoidRobot");
    factory_.registerNodeType<hunav::BlockAgentNode>("BlockAgent");
    factory_.registerNodeType<hunav::GroupWalkNode>("SetGroupWalk");
    factory_.registerNodeType<hunav::IsAnyoneSpeakingNode>("IsAnyoneSpeaking");
    factory_.registerNodeType<hunav::IsSpeakingNode>("IsSpeaking");
    factory_.registerNodeType<hunav::IsAnyoneLookingAtMeNode>("IsAnyoneLookingAtMe");
    factory_.registerNodeType<hunav::IsLookingAtMeNode>("IsLookingAtMe");
    factory_.registerNodeType<hunav::LookAtPointNode>("LookAtPoint");
    factory_.registerNodeType<hunav::LookAtAgentNode>("LookAtAgent");
    factory_.registerNodeType<hunav::LookAtRobotNode>("LookAtRobot");

    // Decorators
    factory_.registerNodeType<hunav::TimeDelayDecorator>("TimeDelay");

    RCLCPP_INFO(this->get_logger(), "BT nodes registered");
  }

  void BTnode::initializeBehaviorTree(const hunav_msgs::msg::Agent &_agent)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Behavior tree of Agent %s, id: %i", _agent.name.c_str(),
                _agent.id);

    // BT::Tree tree;
    // RCLCPP_INFO(this->get_logger(), "Setting id: %i", agents.agents[i].id);
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set<int>("id", (int)_agent.id);
    blackboard->set<double>("dt", 0.0);

    // compose the file name and full path
    const std::string fname = yaml_base_name_
                              + "__agent_"
                              + std::to_string(_agent.id)
                              + "_bt.xml";
    const std::string fullpath = bt_dir_base_ + "/" + fname;
    
    RCLCPP_INFO(this->get_logger(), "Loading BT file: %s", fullpath.c_str());

    try
    {
      trees_[_agent.id] = factory_.createTreeFromFile(fullpath, blackboard);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to load BT XML for agent %d: %s - Check %s for any format mistakes",
          _agent.id, e.what(), fullpath.c_str());
    }

    if (_agent.id == 1)
      {
        publisher_ = std::make_unique<BT::Groot2Publisher>(trees_[_agent.id], 5555);
      }

    RCLCPP_INFO(this->get_logger(), "Behavior Tree for agent %s [id:%i] loaded!", _agent.name.c_str(),
                _agent.id);

    // Set the id of the agent
    // tree_.rootBlackboard()->set<std::string>("id",
    // std::to_string(agents.agents[i].id));
    // tree_.rootBlackboard()->set<int>("id", agents.agents[i].id);

    // // This logger prints state changes on console
    BT::StdCoutLogger logger_cout(trees_[_agent.id]);
    // // This logger saves state changes on file
    // std::string filename = "bt_trace_" + std::to_string(agents.agents[i].id);
    // BT::FileLogger logger_file(tree, (filename + ".fbl").c_str());
    // // This logger stores the execution time of each node
    // BT::MinitraceLogger logger_minitrace(tree, (filename + ".json").c_str());

    // #ifdef ZMQ_FOUND
    //  This logger publish status changes using ZeroMQ. Used by Groot
    //  BT::PublisherZMQ publisher_zmq(tree);
    // #endif

    // root_->addChild(trees_[trees_.size() - 1].rootNode());
    // BT::printTreeRecursively(root_.get());
    BT::printTreeRecursively(trees_[_agent.id].rootNode());
  }

  void BTnode::initializeBehaviorTrees(const hunav_msgs::msg::Agents &_agents)
  {
    // root_ = std::make_unique<BT::ParallelNode>("root", 1, 1);
    // root_ = std::make_unique<BT::SequenceNode>("root");
    RCLCPP_INFO(this->get_logger(), "Initializing Behavior Trees of %lu agents...", _agents.agents.size());

    for (auto a : _agents.agents)
    {
      initializeBehaviorTree(a);
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

  BT::NodeStatus BTnode::tree_tick(double dt)
  {
    // RCLCPP_INFO(this->get_logger(), "Ticking the tree root!");
    BT::NodeStatus status;
    std::unordered_map<int, BT::Tree>::iterator itr;
    for (itr = trees_.begin(); itr != trees_.end(); itr++)
    {
      // RCLCPP_INFO(this->get_logger(), "Ticking the tree id %i", itr->first);
      itr->second.rootBlackboard()->set<double>("dt", dt);
      status = itr->second.tickExactlyOnce();
    }
    return status;
  }

  BT::NodeStatus BTnode::tree_tick(int id)
  {
    // RCLCPP_INFO(this->get_logger(), "Ticking the tree id %i!", id);
    BT::NodeStatus status = trees_[id].tickExactlyOnce();
    return status;
  }

  BT::NodeStatus BTnode::tree_tick(int id, double dt)
  {
    // RCLCPP_INFO(this->get_logger(), "Ticking the tree id %i!", id);
    trees_[id].rootBlackboard()->set<double>("dt", dt);
    // RCLCPP_INFO(this->get_logger(), "After setting dt:%.4f!", dt);
    BT::NodeStatus status = trees_[id].tickExactlyOnce();
    // RCLCPP_INFO(this->get_logger(), "After ticking the tree!");
    return status;
  }

  void BTnode::computeAgentsService(const std::shared_ptr<hunav_msgs::srv::ComputeAgents::Request> request,
                                    std::shared_ptr<hunav_msgs::srv::ComputeAgents::Response> response)
  {
    auto ro = std::make_shared<hunav_msgs::msg::Agent>(request->robot);
    auto ag = std::make_shared<hunav_msgs::msg::Agents>(request->current_agents);

    // Update the internal agent states with the
    // received data from the simulator
    btfunc_.updateAllAgents(ro, ag);

    if (!initialized_)
    {
      RCLCPP_INFO(this->get_logger(), "First service call received!");
      RCLCPP_INFO(this->get_logger(), "robot pose x:%.2f, y:%.2f, th:%.2f", ro->position.position.x,
                  ro->position.position.y, ro->yaw);
      RCLCPP_INFO(this->get_logger(), "Agents received: %li", ag->agents.size());

      initializeBehaviorTrees(request->current_agents);
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
    // if (pub_agent_states_)
    publish_agent_states(t, ag);
    publish_robot_state(t, ro);
    if (pub_people_)
      publish_people(t, ag);

    double time_step_secs = (rclcpp::Time(ag->header.stamp) - prev_time_).seconds();
    // if the time was reset, we get a negative value
    if (time_step_secs < 0.0)
      time_step_secs = 0.0; // 0.05

    //BT::NodeStatus status = tree_tick(time_step_secs);
    (void)tree_tick(time_step_secs);
    prev_time_ = rclcpp::Time(ag->header.stamp);
    //}

    response->updated_agents = btfunc_.getUpdatedAgents();
  }

  void BTnode::resetAgentsService(const std::shared_ptr<hunav_msgs::srv::ResetAgents::Request> request,
                                  std::shared_ptr<hunav_msgs::srv::ResetAgents::Response> response)
  {
    auto ro = std::make_shared<hunav_msgs::msg::Agent>(request->robot);
    auto ag = std::make_shared<hunav_msgs::msg::Agents>(request->current_agents);

    // Update the internal agent states with the
    // received data from the simulator
    btfunc_.updateAllAgents(ro, ag);
    response->ok = true;
  }

  void BTnode::moveAgentService(const std::shared_ptr<hunav_msgs::srv::MoveAgent::Request> request,
                                std::shared_ptr<hunav_msgs::srv::MoveAgent::Response> response)
  {
    auto ro = std::make_shared<hunav_msgs::msg::Agent>(request->robot);
    auto ag = std::make_shared<hunav_msgs::msg::Agents>(request->current_agents);

    // Update the internal agent states with the
    // received data from the simulator
    // RCLCPP_INFO(this->get_logger(), "Service call received agent id %i", request->agent_id);
    btfunc_.updateAllAgents(ro, ag);
    // RCLCPP_INFO(this->get_logger(), "Agents updated!");

    if (!initialized_)
    {
      RCLCPP_INFO(this->get_logger(), "First service call received!");
      RCLCPP_INFO(this->get_logger(), "robot pose x:%.2f, y:%.2f, th:%.2f", ro->position.position.x,
                  ro->position.position.y, ro->yaw);
      RCLCPP_INFO(this->get_logger(), "Agents received: %li", ag->agents.size());

      initializeBehaviorTrees(request->current_agents);
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
    // if (pub_agent_states_)
    publish_agent_states(t, ag);
    publish_robot_state(t, ro);
    if (pub_people_)
      publish_people(t, ag);

    double time_step_secs = (rclcpp::Time(ag->header.stamp) - prev_time_).seconds();
    // time_step_secs = 0.1;

    // RCLCPP_INFO(this->get_logger(), "Time step computed: %.4f",
    // time_step_secs);

    // we do not tick the tree if the frequency is higher than 100Hz approx
    // if (time_step_secs > 0.008) {
    // Call the ticks of the behavior trees (they must update the
    // sfm_agents_)
    //BT::NodeStatus status = tree_tick(request->agent_id, time_step_secs);
    (void)tree_tick(request->agent_id, time_step_secs);
    prev_time_ = rclcpp::Time(ag->header.stamp);
    //}

    response->updated_agent = btfunc_.getUpdatedAgent(request->agent_id);
  }

  void BTnode::computeAgentService(const std::shared_ptr<hunav_msgs::srv::ComputeAgent::Request> request,
                                   std::shared_ptr<hunav_msgs::srv::ComputeAgent::Response> response)
  {
    // rclcpp::Rate loop_rate(40);
    // while (!btfunc_.ok()) {
    // loop_rate.sleep();
    //}
    //BT::NodeStatus status = tree_tick(request->id);
    (void)tree_tick(request->id);
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

  void BTnode::publish_agents_tf(rclcpp::Time t, const hunav_msgs::msg::Agent::SharedPtr robot,
                                 const hunav_msgs::msg::Agents::SharedPtr msg)
  {
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

    for (const auto &a : msg->agents)
    {
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

  void BTnode::publish_agent_states(rclcpp::Time t, const hunav_msgs::msg::Agents::SharedPtr msg)
  {
    (void)t;
    human_state_publisher_->publish(*msg);
  }

  void BTnode::publish_robot_state(rclcpp::Time t, const hunav_msgs::msg::Agent::SharedPtr msg)
  {
    (void)t;
    robot_state_publisher_->publish(*msg);
  }

  void BTnode::publish_people(rclcpp::Time t, const hunav_msgs::msg::Agents::SharedPtr msg)
  {
    people_msgs::msg::People people;
    people.header.stamp = t;
    people.header.frame_id = msg->header.frame_id;
    for (const auto &a : msg->agents)
    {
      people_msgs::msg::Person person;
      person.name = a.name;
      person.position = a.position.position;
      person.position.z = a.yaw;
      person.velocity.x = a.linear_vel * cos(a.yaw);
      person.velocity.y = a.linear_vel * sin(a.yaw);
      // I add the angular velocity in the z coordinate
      person.velocity.z = a.angular_vel;
      person.reliability = 1.0;
      person.tags.push_back(std::to_string(a.id));
      person.tags.push_back(std::to_string(a.group_id));
      person.tags.push_back(std::to_string(a.behavior.type));
      person.tagnames.push_back("id");
      person.tagnames.push_back("group_id");
      person.tagnames.push_back("behavior");
      people.people.push_back(person);
    }
    people_publisher_->publish(people);
  }

  void BTnode::publish_agents_forces(rclcpp::Time t, const hunav_msgs::msg::Agents::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray markers;
    for (const auto &a : msg->agents)
    {
      sfm::Forces frs = getAgentForces(a.id);
      publishForceMarker(a.id + 1, a.name, msg->header.frame_id, t, a.position.position, getColor(1, 0, 0, 1),
                         frs.obstacleForce,
                         markers); // RED
      publishForceMarker(a.id + 2, a.name, msg->header.frame_id, t, a.position.position, getColor(0, 0, 1, 1),
                         frs.socialForce,
                         markers); // BLUE
      // publishForceMarker(2, getColor(0, 1, 1, 1), robot_.forces.groupForce,
      //                   markers);
      publishForceMarker(a.id + 3, a.name, msg->header.frame_id, t, a.position.position, getColor(0, 1, 0, 1),
                         frs.desiredForce,
                         markers); // GREEN
      publishForceMarker(a.id + 4, a.name, msg->header.frame_id, t, a.position.position, getColor(1, 1, 1, 1),
                         frs.globalForce,
                         markers); // WHITE
      // publishForceMarker(5, getColor(1, 1, 0, 1), robot_.velocity, markers);
      visualization_msgs::msg::Marker marker;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = t;
      marker.ns = a.name + "/behavior";
      marker.id = a.id;
      marker.action = visualization_msgs::msg::Marker::ADD; // force.norm() > 1e-4 ? 0 : 2;
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
      switch (a.behavior.type)
      {
      case hunav_msgs::msg::AgentBehavior::BEH_REGULAR:
        marker.text = marker.text + "/REGULAR";
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE:
        marker.text = marker.text + "/IMPASSIVE";
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_SURPRISED:
        marker.text = marker.text + "/SURPRISED";
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_SCARED:
        marker.text = marker.text + "/SCARED";
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_CURIOUS:
        marker.text = marker.text + "/CURIOUS";
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_THREATENING:
        marker.text = marker.text + "/THREATENING";
        break;
      default:
        marker.text = marker.text + "/REGULAR";
      }
      markers.markers.push_back(marker);
    }
    forces_publisher_->publish(markers);
  }

  void BTnode::publishForceMarker(unsigned index, std::string name, std::string frame, rclcpp::Time t,
                                  geometry_msgs::msg::Point p, const std_msgs::msg::ColorRGBA &color,
                                  const utils::Vector2d &force, visualization_msgs::msg::MarkerArray &markers)
  {
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

  std_msgs::msg::ColorRGBA BTnode::getColor(double r, double g, double b, double a)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  bool BTnode::getParametersFromLoader()
  {
    RCLCPP_INFO(get_logger(), "Waiting for hunav_loader get_parameters service...");
    
    // Wait for service to be available with timeout
    if (!get_parameters_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Service /get_parameters not available after waiting");
      return false;
    }

    // Create request
    auto request = std::make_shared<hunav_msgs::srv::GetParameters::Request>();
    
    // Call service synchronously
    auto future = get_parameters_client_->async_send_request(request);
    
    // Wait for result with timeout
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      
      // Store received parameters
      pub_people_ = response->publish_people;
      map_name_ = response->map;
      simulator_name_ = response->simulator;
      yaml_base_name_ = response->yaml_base_name;
      
      // Process global goals
      global_goals_.clear();
      for (size_t i = 0; i < response->goal_ids.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = response->goal_x_coords[i];
        pt.y = response->goal_y_coords[i];
        pt.z = 0.0;
        global_goals_[response->goal_ids[i]] = pt;
        
        RCLCPP_INFO(get_logger(),
          "Loaded global goal %ld → (%.3f, %.3f)", response->goal_ids[i], pt.x, pt.y);
      }
      
      RCLCPP_INFO(get_logger(), "Successfully retrieved parameters from hunav_loader");
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to call service /get_parameters");
      return false;
    }
  }

} // namespace hunav
