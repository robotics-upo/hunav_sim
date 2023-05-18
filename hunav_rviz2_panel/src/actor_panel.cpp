#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <QDebug>

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>

#include "yaml-cpp/yaml.h"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "std_msgs/msg/string.hpp"
#include "rviz_common/tool.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "headers/actor_panel.hpp"
#include "headers/goal_pose_updater.hpp"

#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

namespace hunav_rviz2_panel
{
  // To get the coordinates when the map is clicked.
  GoalPoseUpdater GoalUpdater;

  // Constructor, creates the panel.
  ActorPanel::ActorPanel(QWidget *parent)
      : rviz_common::Panel(parent), rclcpp::Node("hunav_agents")
  {
    
    QVBoxLayout *topic_button = new QVBoxLayout;
    QHBoxLayout *layout = new QHBoxLayout;
    open_button = new QPushButton("Open");
    actors = new QLineEdit;
    actor_button = new QPushButton("Create agents");
    checkbox = new QCheckBox("Use default directory", this);

    actors->setValidator(new QIntValidator(0, 100, this));

    topic_button->addWidget(new QLabel("Open yaml file (agents.yaml)"));
    topic_button->addWidget(open_button);
    topic_button->addWidget(new QLabel("Set number of agents to generate: "));
    topic_button->addWidget(actors);
    topic_button->addWidget(actor_button);
    checkbox->setChecked(true);
    topic_button->addWidget(checkbox);

    connect(actor_button, SIGNAL(clicked()), this, SLOT(addAgent()));
    connect(open_button, SIGNAL(clicked()), this, SLOT(parseYaml()));

    layout->addLayout(topic_button);
    setLayout(layout);

    // Create publisher for agents' 
    // Agents' intial pose are published in /hunav_agent topic.
    initial_pose_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("hunav_agent", rclcpp::QoS(1).transient_local());
    // Agents' goals are published in /hunav_goals.
    goals_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("hunav_goals", rclcpp::QoS(1).transient_local());

  }

  ActorPanel::~ActorPanel(){

  }

  // Shows the agent creation window.
  void ActorPanel::addAgent(){
    
    window = new QWidget;
    window->setWindowFlags(window->windowFlags() & ~Qt::WindowCloseButtonHint); // Unables close button

    open_button->setEnabled(false);
    actor_button->setEnabled(false);

    QVBoxLayout *topic_layout = new QVBoxLayout(window);
    QHBoxLayout *layout = new QHBoxLayout;
    QPushButton *directory;
    // Combobox for behavior selection.
    behavior_combobox = new QComboBox();
    // Combobox for skin selection.
    skin_combobox = new QComboBox();
    initial_pose_button = new QPushButton("Set initial pose");
    reset_initial_pose = new QPushButton("Reset initial pose");
    reset_goals = new QPushButton("Reset goals");
    QLabel *num_goals_set_label = new QLabel("Set number of goals");
    num_goals_set = new QLineEdit();
    // exit_button = new QPushButton("Exit agent creation");

    num_goals_set->setValidator(new QIntValidator(0, 100, this));

    // Only removes markers when the "Create agents" button is clicked (If agent_count > 1 means that agents are being created, 
    // so markers need to stay in place)
    if(agent_count == 1){
      removeCurrentMarkers();
    }

    if(!checkbox->isChecked() && show_file_selector_once == true){
      directory = new QPushButton("Choose directory");
      
      topic_layout->addWidget(new QLabel("Select the directory where the file is going to be saved:"));
      topic_layout->addWidget(directory);
    }

    // If user does not provides number of agents, by default creates 1 agent.
    if(actors->text().isEmpty()){
      topic_layout->addWidget(new QLabel("Agent " + QString::number(agent_count) + "/" + QString::number(1)));
    }
    else{
      topic_layout->addWidget(new QLabel("Agent " + QString::number(agent_count) + "/" + actors->text()));
    }
    
    topic_layout->addWidget(new QLabel("Agent's name:"));
    agent_name = new QLineEdit;
    topic_layout->addWidget(agent_name);
    topic_layout->addWidget(new QLabel("Behavior:"));

    behavior_combobox->addItem("Regular");
    behavior_combobox->addItem("Impassive");
    behavior_combobox->addItem("Surprised");
    behavior_combobox->addItem("Scared");
    behavior_combobox->addItem("Curious");
    behavior_combobox->addItem("Threatening");
    topic_layout->addWidget(behavior_combobox);

    topic_layout->addWidget(new QLabel("Skin:"));
    skin_combobox->addItem("Elegant man");
    skin_combobox->addItem("Casual man");
    skin_combobox->addItem("Elegant woman");
    skin_combobox->addItem("Regular man");
    skin_combobox->addItem("Worker man");
    skin_combobox->addItem("Blue jeans");
    skin_combobox->addItem("Green t-shirt");
    skin_combobox->addItem("Blue t-shirt");
    skin_combobox->addItem("Red t-shirt");
    topic_layout->addWidget(skin_combobox);

    goals_button = new QPushButton("Set goals");
    save_button = new QPushButton("Save");

    save_button->setEnabled(false);
    goals_button->setEnabled(false);
    reset_goals->setEnabled(false);
    reset_initial_pose->setEnabled(false);
    // exit_button->setEnabled(true);

    topic_layout->addWidget(initial_pose_button);
    topic_layout->addWidget(reset_initial_pose);
    topic_layout->addWidget(num_goals_set_label);
    topic_layout->addWidget(num_goals_set);
    topic_layout->addWidget(goals_button);
    topic_layout->addWidget(reset_goals);
    topic_layout->addWidget(save_button);
    // topic_layout->addWidget(exit_button);

    layout->addLayout(topic_layout);
    setLayout(layout);
    
    window->show();

    connect(save_button, SIGNAL(clicked()), this, SLOT(saveAgents()));
    connect(reset_goals, SIGNAL(clicked()), this, SLOT(resetGoal()));
    connect(reset_initial_pose, SIGNAL(clicked()), this, SLOT(resetInitialPose()));
    connect(goals_button, SIGNAL(clicked()), this, SLOT(getNewGoal()));
    connect(initial_pose_button, SIGNAL(clicked()), this, SLOT(setInitialPose()));
    // connect(exit_button, SIGNAL(clicked()), this, SLOT(exitAgentCreation()));

    if(!checkbox->isChecked() && show_file_selector_once){
      connect(directory, &QPushButton::clicked, this, [=](){
        openFileExplorer(false);
      });
      //connect(directory, SIGNAL(clicked()), this, SLOT(openFileExplorer()));
      show_file_selector_once = false;
    }
  }

  // Window that allow user to start using the HuNavGoal tool to select the agent's initial pose.
  void ActorPanel::setInitialPose(){
    
    // Connects the HuNav Panel with the HuNavGoal tool.
    initial_pose_connection = new QObject();
    initial_pose_connection->connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onInitialPose(double,double,double,QString)));

    window2 = new QWidget();
    window2->setWindowFlags(window->windowFlags() & ~Qt::WindowCloseButtonHint); // Unables close button

    topic_layout_init_pose = new QVBoxLayout(window2);
    QHBoxLayout *layout = new QHBoxLayout;
    QPushButton *close_button = new QPushButton("Close");
    
    topic_layout_init_pose->addWidget(new QLabel("Initial pose"));
    topic_layout_init_pose->addWidget(new QLabel("Use HunavGoals tool to select the initial pose"));
    topic_layout_init_pose->addWidget(close_button);
    
    layout->addLayout(topic_layout_init_pose);

    window2->show();

    initial_pose_connection->deleteLater();
    
    connect(close_button, SIGNAL(clicked()), this, SLOT(closeInitialPoseWindow()));
  }

    // Get point from map to store it in initial pose.
  void ActorPanel::onInitialPose(double x, double y, double theta, QString frame)
  {

    visualization_msgs::msg::Marker marker;

    // Need this variable in order to remove initial pose markers when initial pose button is clicked again.
    initial_pose_set = true;
    initial_pose = geometry_msgs::msg::PoseStamped();

    initial_pose.header.stamp = rclcpp::Clock().now();
    initial_pose.header.frame_id = frame.toStdString();
    initial_pose.pose.position.x = x;
    initial_pose.pose.position.y = y;
    pose.pose.position.z = theta;
    initial_pose.pose.position.z = 1.25;
    theta = 0;

    // Random color selector
    srand((unsigned) time(NULL));
    randomRGB();

    marker = createMarker(x, y, marker_id, "person", "create");
    marker_id++;

    initial_pose_marker_array = visualization_msgs::msg::MarkerArray();
    initial_pose_marker_array.markers.push_back(marker);

    initial_pose_publisher->publish(std::move(initial_pose_marker_array));

    oldPose = initial_pose;
    stored_pose = initial_pose;

    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onInitialPose(double,double,double,QString)));

    // Update the panel with the initial pose
    topic_layout_init_pose->addWidget(new QLabel("Initial pose coordinates"));
    topic_layout_init_pose->addWidget(new QLabel("x: " + QString::fromStdString(std::to_string(initial_pose.pose.position.x))));        
    topic_layout_init_pose->addWidget(new QLabel("y: " + QString::fromStdString(std::to_string(initial_pose.pose.position.y))));    
    topic_layout_init_pose->addWidget(new QLabel("z: " + QString::fromStdString(std::to_string(initial_pose.pose.position.z))));

    window->activateWindow();
    window2->activateWindow();
  }

  // Window that allow users to start using the HuNavGoal tool to select the agent's goals.
  void ActorPanel::getNewGoal(){

    window1 = new QWidget();
    window1->setWindowFlags(window->windowFlags() & ~Qt::WindowCloseButtonHint); // Unables close button
    
    goals_layout = new QVBoxLayout(window1);
    QHBoxLayout *layout = new QHBoxLayout;
    QPushButton *close_button = new QPushButton("Close");
    goals_number = 1;
    QString goal = "Goals";
    finish_arrow = false;

    // Reset markers for removing only the current goal.
    markers_array_to_remove.clear();

    // Connects the HuNav Panel with the HuNavGoal tool.
    goals_connection = new QObject();
    goals_connection->connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onNewGoal(double,double,double,QString)));

    goals_layout->addWidget(new QLabel(goal));

    // If num_goals not set, by default will be 3 goals.
    if(num_goals_set->text().isEmpty()){
      num_goals_set->setText(QString::number(3));
    }

    // Fill goal's vector
    for(int j = 0; j < num_goals_set->text().toInt(); j++){
      goals.push_back("G" + QString::number(j));
    }

    if(num_goals_set->text().toInt() == 0){
      num_goals_set->setText("1");
    }

    goals_remaining = new QLabel("Goals " + QString::number(goals_number) + "/" + num_goals_set->text());

    goals_layout->addWidget(new QLabel("Use HunavGoals tool to select goals"));
    goals_layout->addWidget(goals_remaining);
    goals_layout->addWidget(close_button);

    layout->addLayout(goals_layout);
    setLayout(layout);

    window1->show();

    // Close connection with thw HuNavGoal tool
    goals_connection->deleteLater();

    // Removes older data from inital_pose
    initial_pose = geometry_msgs::msg::PoseStamped();
    
    connect(close_button, SIGNAL(clicked()), this, SLOT(closeGoalsWindow()));
    
  }

  // Get point from map to store goals.
  void ActorPanel::onNewGoal(double x, double y, double theta, QString frame)
  {
    // Initialize goals marker_array, which contains all goals markers
    marker_array = visualization_msgs::msg::MarkerArray();
    // Marker for new goal
    visualization_msgs::msg::Marker marker;
    // Marker for the arrow that connects each marker
    visualization_msgs::msg::Marker arrow_marker;

    pose = geometry_msgs::msg::PoseStamped();

    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = frame.toStdString();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = theta;
    pose.pose.position.z = 1.25;

    poses.push_back(pose);

    // Update initial pose marker orientation to the next goal
    if(goals_number == 1){
      float x_orientation = x - oldPose.pose.position.x;
      float y_orientation = y - oldPose.pose.position.y;
      float z_orientation = atan2(y_orientation, x_orientation);

      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, z_orientation);

      initial_pose_marker_array.markers.back().pose.orientation = tf2::toMsg(quaternion);

      initial_pose_publisher->publish(std::move(initial_pose_marker_array));
    }
    
    marker = createMarker(x, y, marker_id, "cube", "create");
    // Increment marker id for next marker.
    // Maybe move this inside the function
    marker_id++;
    
    arrow_marker = createArrowMarker(oldPose.pose.position.x, oldPose.pose.position.y, x, y, marker_id);

    // Increment marker id for next marker.
    marker_id++;
    
    // Update timestamp to old markers 
    int marker_array_size = static_cast<int>(marker_array.markers.size());
    for(int i = 0; i < marker_array_size; i++){
      marker_array.markers[i].header.stamp = rclcpp::Node::now();
    }

    marker_array.markers.push_back(marker);
    marker_array.markers.push_back(arrow_marker);

    // If goals == num_goals_set means that another arrow has to be added to close the path.
    if(goals_number >= num_goals_set->text().toInt() && finish_arrow == false){
      visualization_msgs::msg::Marker arrow_marker1;

      arrow_marker1 = createArrowMarker(x, y, stored_pose.pose.position.x, stored_pose.pose.position.y, marker_id);

      marker_array.markers.push_back(arrow_marker1);

      goals_publisher->publish(std::move(marker_array));

      finish_arrow = true;
      
      updateGoalsRemainingText();

      disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onNewGoal(double,double,double,QString)));
    }
    else if(finish_arrow == false){
      oldPose = pose;
      first_actor = false;

      goals_publisher->publish(std::move(marker_array));

      QObject::disconnect(goals_connection);

      updateGoalsRemainingText();
    }

    // Iterate goal's vector to show selected goals
    // for(QString aux : goals){
    //   goals_layout->addWidget(new QLabel(aux));  
      
    //   if(!poses.empty()){
        
    //     goals_layout->addWidget(new QLabel("x: " + QString::fromStdString(std::to_string(poses[i].pose.position.x))));
        
    //     goals_layout->addWidget(new QLabel("y: " + QString::fromStdString(std::to_string(poses[i].pose.position.y))));
        
    //     goals_layout->addWidget(new QLabel("z: " + QString::fromStdString(std::to_string(poses[i].pose.position.z))));
        
    //     i++;
    //   }
    //   else{
    //     goals_layout->addWidget(new QLabel("No goals yet"));
    //   }

    // }

    window->activateWindow();
    window1->activateWindow();

  }

  void ActorPanel::updateGoalsRemainingText(){
    goals_remaining->setText("Goals " + QString::number(goals_number) + "/" + num_goals_set->text());  
    goals_number++;
  }

  // Saves all information about agents in yaml file
  void ActorPanel::saveAgents(){

    window->close();
    std::ofstream file;

    // If checkbox is not checked, it means that the user wants to store file in another directory.
    if(!checkbox->isChecked()){
      if(dir.empty()){
        RCLCPP_ERROR(this->get_logger(), "The directory to store the output file has not been selected. Using default directory.");
        dir = ament_index_cpp::get_package_share_directory("hunav_agent_manager") + "/config";
      }
      pkg_shared_tree_dir_ = dir + "/agents.yaml";
    }
    else{
      try {
      pkg_shared_tree_dir_ = ament_index_cpp::get_package_share_directory("hunav_agent_manager");

      } catch (const char* msg) {
        RCLCPP_ERROR(this->get_logger(),
                    "Package hunav_agent_manager not found in dir: %s!!!",
                    pkg_shared_tree_dir_.c_str());
      }
      pkg_shared_tree_dir_ = pkg_shared_tree_dir_ + "/config/agents.yaml";
    }
    
    // Open file to save agents
    file.open(pkg_shared_tree_dir_ , std::ofstream::trunc);
    
    //Check if number of agents isn't empty
    if(actors->text().isEmpty()){
      num_actors = 1;
    }
    else{
      QString temp = actors->text();
      num_actors = temp.toInt();
    }

    // Get input from user
    std::string name;

    // Check name's field isn't empty
    if(agent_name->text().isEmpty()){
      name = "agent" + std::to_string(iterate_actors);
    }
    else{
      name = agent_name->text().toStdString();
    }
    
    std::string behavior = std::to_string(checkComboBox());
    std::string skin = std::to_string(checkComboBoxSkin());

    // Fill name's array for later use
    names.push_back(name);

    YAML::Node agent1;
    agent1["id"] = iterate_actors;
    agent1["skin"] = skin;
    agent1["behavior"] = behavior;
    agent1["group_id"] = "-1";
    agent1["max_vel"] = "1.5";
    agent1["radius"] = "0.4";
    agent1["init_pose"]["x"] = std::to_string(stored_pose.pose.position.x);
    agent1["init_pose"]["y"] = std::to_string(stored_pose.pose.position.y);
    agent1["init_pose"]["z"] = std::to_string(stored_pose.pose.position.z);
    agent1["init_pose"]["h"] = "0.0";
    agent1["goal_radius"] = "0.3";
    agent1["cyclic_goals"] = true;

    for(int i = 0; i < num_goals_set->text().toInt(); i++){
      agent1["goals"].push_back("g" + std::to_string(i));  
    }
    
    for(int i = 0; i < num_goals_set->text().toInt(); i++){
      std::string current_g = "g" + std::to_string(i);
      agent1[current_g]["x"] = std::to_string(poses[i].pose.position.x);
      agent1[current_g]["y"] = std::to_string(poses[i].pose.position.y);
      agent1[current_g]["h"] = std::to_string(poses[i].pose.position.z);
    }

    // Fill agent array
    actors_info.push_back(agent1);

    if(iterate_actors == num_actors){
      YAML::Node hunav_loader;

      //ros__parameters needs two underscores
      hunav_loader["hunav_loader"]["ros__parameters"]["map"] = "cafe";
      hunav_loader["hunav_loader"]["ros__parameters"]["publish_people"] = true;
      
      for (auto i = names.begin(); i != names.end(); ++i)
        hunav_loader["hunav_loader"]["ros__parameters"]["agents"].push_back(*i);

      int names_counter = 0;

      for (auto i = actors_info.begin(); i != actors_info.end(); ++i){
        hunav_loader["hunav_loader"]["ros__parameters"][names[names_counter]] = *i;
        names_counter++;
      }

      RCLCPP_INFO(this->get_logger(), "Generating agents.yaml");
        
      // Writes hunav_loader node to file
      file << hunav_loader;

      // Close file
      file.close();

      RCLCPP_INFO(this->get_logger(), "Agents.yaml generated");

      // Clean variables in order to create new Agent.yaml if neccesary
      actors_info.clear();
      names.clear();
      point.clear();
      iterate_actors = 1;
      agent_count = 1;
      poses.clear();

      // Clear goals_marker in order to get new goals if necessary
      marker_array = visualization_msgs::msg::MarkerArray();
      // Reset initial pose to false in case user does not close the panel after writing the file.
      // If initial pose is not reseted in the next iteration the initial pose could be empty.
      initial_pose_set = false;
      
      show_file_selector_once = true;

      // Enables first windows button in order to allow agent creation again
      open_button->setEnabled(true);
      actor_button->setEnabled(true);
    }
    else{
      iterate_actors++;

      // Need this variable in order to remove initial pose markers when initial pose button is clicked again.
      // To generate the next agent, it has to be set to false in order to not remove the current markers. 
      initial_pose_set = false;
      markers_array_to_remove.clear();
      
      agent_name->clear();

      agent_name->setText("");

      poses.clear();

      agent_count++;

      addAgent();
    }

  }

  // Open a generated yaml file
  void ActorPanel::parseYaml(){

    removeCurrentMarkers();

    // Check if user wants to open file from another directory.
    if(!checkbox->isChecked()){
      openFileExplorer(true);
      pkg_shared_tree_dir_ = dir;
      RCLCPP_INFO(this->get_logger(), "DIR: %s", pkg_shared_tree_dir_.c_str());
    }
    else{
      try {
        pkg_shared_tree_dir_ =
            ament_index_cpp::get_package_share_directory("hunav_agent_manager");
      } 
      catch (const char* msg) {
          RCLCPP_ERROR(this->get_logger(),
                      "Package hunav_agent_manager not found in dir: %s!!!",
                      pkg_shared_tree_dir_.c_str());
      }
      pkg_shared_tree_dir_ = pkg_shared_tree_dir_ + "/config/agents.yaml";
      RCLCPP_INFO(this->get_logger(), "Default directory: %s", pkg_shared_tree_dir_.c_str());
    }
    
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    YAML::Node yaml_file;

    try{
      yaml_file = YAML::LoadFile(pkg_shared_tree_dir_);
    }
    catch(const YAML::BadFile& ex){
      pkg_shared_tree_dir_ = ament_index_cpp::get_package_share_directory("hunav_agent_manager") + "/config/agents.yaml";
      RCLCPP_ERROR(this->get_logger(), "File not found in dir! Using default path: %s.", pkg_shared_tree_dir_.c_str());
      yaml_file = YAML::LoadFile(pkg_shared_tree_dir_);
    }

    std::vector<std::string> agents_vector;
    std::vector<std::string> current_goals_vector;
    int ids = 0;
    std::vector<YAML::Node> current_arrow;

    srand((unsigned) time(NULL));

    int marker_array_size = static_cast<int>(marker_array->markers.size());

    if(marker_array_size > 0){
      // Remove existing markers before publishing new ones.
      removeCurrentMarkers();
    }

    // Get name and number of agents
    YAML::Node agents = yaml_file["hunav_loader"]["ros__parameters"]["agents"];

    // Fill with agent's names
    for(int i = 0; i < static_cast<int>(agents.size()); i++){
      agents_vector.push_back(yaml_file["hunav_loader"]["ros__parameters"]["agents"][i].as<std::string>());
    }

    for(int i = 0; i < static_cast<int>(agents_vector.size()); i++){

      randomRGB();
      
      YAML::Node current_agent = yaml_file["hunav_loader"]["ros__parameters"][agents_vector[i]];
      bool first_arrow = true;
      first_actor = true;
      // Initial pose
      visualization_msgs::msg::Marker initial_marker;
      // Get skin name
      checkParserSkin(current_agent["skin"].as<int>());
      initial_marker = createMarker(current_agent["init_pose"]["x"].as<double>(), current_agent["init_pose"]["y"].as<double>(), ids, "person", "parser");

      marker_array->markers.push_back(initial_marker);

      ids++;

      // Goals markers

      YAML::Node current_goals = current_agent["goals"];

      for(int j = 0; j < static_cast<int>(current_goals.size()); j++){
        current_goals_vector.push_back(current_agent["goals"][j].as<std::string>());
      }

      for(int k = 0; k < static_cast<int>(current_goals_vector.size()); k++){
        
        visualization_msgs::msg::Marker arrow_marker;
        visualization_msgs::msg::Marker marker;
        marker = createMarker(current_agent[current_goals_vector[k]]["x"].as<double>(), current_agent[current_goals_vector[k]]["y"].as<double>(), ids, "cube", "parser");

        if(first_actor){
          float x_orientation = marker.pose.position.x - initial_marker.pose.position.x;
          float y_orientation = marker.pose.position.y - initial_marker.pose.position.y;
          float z_orientation = atan2(y_orientation, x_orientation);

          tf2::Quaternion quaternion;
          quaternion.setRPY(0, 0, z_orientation);
          marker_array->markers.back().pose.orientation = tf2::toMsg(quaternion);
          first_actor = false;
        }

        marker_array->markers.push_back(marker);

        ids++;
        
        if(first_arrow){
          first_arrow = false;

          arrow_marker = createArrowMarker(current_agent["init_pose"]["x"].as<double>(), current_agent["init_pose"]["y"].as<double>(),
          current_agent[current_goals_vector[k]]["x"].as<double>(), current_agent[current_goals_vector[k]]["y"].as<double>(), ids);

          marker_array->markers.push_back(arrow_marker);

          ids++;
        }
        else {         
          arrow_marker = createArrowMarker(current_agent[current_goals_vector[k-1]]["x"].as<double>(), current_agent[current_goals_vector[k-1]]["y"].as<double>(),
          current_agent[current_goals_vector[k]]["x"].as<double>(), current_agent[current_goals_vector[k]]["y"].as<double>(), ids);

          marker_array->markers.push_back(arrow_marker);

          ids++;
        }
        
        if(k == static_cast<int>(current_goals_vector.size() - 1)){
          arrow_marker = createArrowMarker(current_agent[current_goals_vector[k]]["x"].as<double>(), current_agent[current_goals_vector[k]]["y"].as<double>(),
          current_agent["init_pose"]["x"].as<double>(), current_agent["init_pose"]["y"].as<double>(), ids);

          marker_array->markers.push_back(arrow_marker);

          ids++;
        }
        
      }
      
      current_goals_vector.clear();
    }
    
    initial_pose_publisher->publish(std::move(marker_array));
  }

  int ActorPanel::checkComboBox(){
    std::string aux = behavior_combobox->currentText().toStdString();

    if(aux.compare("Regular") == 0){
      return 1;
    }
    else if(aux.compare("Impassive") == 0){
      return 2;
    }
    else if(aux.compare("Surprised") == 0){
      return 3;
    }
    else if(aux.compare("Scared") == 0){
      return 4;
    }
    else if(aux.compare("Curious") == 0){
      return 5;
    }
    else{
      return 6;
    }
  }

  int ActorPanel::checkComboBoxSkin(){
    std::string aux = skin_combobox->currentText().toStdString();

    if(aux.compare("Elegant man") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
      return 0;
    }
    else if(aux.compare("Casual man") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/casual_man.dae";
      return 1;
    }
    else if(aux.compare("Elegant woman") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_woman.dae";
      return 2;
    }
    else if(aux.compare("Regular man") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/regular_man.dae";
      return 3;
    }
    else if(aux.compare("Worker man") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/worker_man.dae";
      return 4;
    }
    else if(aux.compare("Blue jeans") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
      return 5;
    }
    else if(aux.compare("Green t-shirt") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
      return 6;
    }
    else if(aux.compare("Blue t-shirt") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
      return 7;
    }
    else if(aux.compare("Red t-shirt") == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
      return 8;
    }
    else{
      return 0;
    }
  }

  void ActorPanel::checkParserSkin(int skin){
    if(skin == 0){
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
    }
    else if(skin == 1){
      person_skin = "package://hunav_rviz2_panel/meshes/casual_man.dae";
    }
    else if(skin == 2){
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_woman.dae";
    }
    else if(skin == 3){
      person_skin = "package://hunav_rviz2_panel/meshes/regular_man.dae";
    }
    else if(skin == 4){
      person_skin = "package://hunav_rviz2_panel/meshes/worker_man.dae";
    }
    else{
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
    }
  }

  visualization_msgs::msg::Marker ActorPanel::createMarker(double point1_x, double point1_y, double ids, std::string marker_shape, std::string create_or_parser){
    
    visualization_msgs::msg::Marker marker;
    uint32_t shape;
    float scale;
    
    if(marker_shape.compare("person") == 0){
      // The variable create_or_parser is used to know from where we are calling the createMarker function
      // If the function is being called from the creation of agents, we need to check which skin is selected in the combobox
      // If the function is being called from the parser, we already know which skin it has by reading the yaml file.
      if(create_or_parser.compare("create") == 0){
        //Check which skin is selected
        checkComboBoxSkin();
      }
      
      shape = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.mesh_resource = person_skin;
      scale = 1;
      marker.pose.position.z = 0.0;
    }
    else{
      scale = 0.3;
      shape = visualization_msgs::msg::Marker::CUBE;
      //marker.mesh_resource = "package://hunav_rviz2_panel/meshes/ring.dae";
      marker.color.r = rgb[red];
      marker.color.g = rgb[green];
      marker.color.b = rgb[blue];
      marker.color.a = 1.0; // alpha has to be non-zero
      marker.pose.position.z = 0.5;
    }
    
    marker.header.frame_id = "/map";
    marker.header.stamp = rclcpp::Node::now();
    marker.ns = "basic_shapes";
    marker.id = ids;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.mesh_use_embedded_materials = true;

    marker.pose.position.x = point1_x;
    marker.pose.position.y = point1_y;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    markers_array_to_remove.push_back(marker);
    
    return marker;

  }

  visualization_msgs::msg::Marker ActorPanel::createArrowMarker(double point1_x, double point1_y, double point2_x, double point2_y, double ids){
    visualization_msgs::msg::Marker arrow_marker;

    arrow_marker.header.frame_id = "/map";
    arrow_marker.header.stamp = rclcpp::Node::now();
    arrow_marker.id = ids;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point point1;
    point1.x = point1_x;
    point1.y = point1_y;
    point1.z = 0.5;

    geometry_msgs::msg::Point point2;
    point2.x = point2_x;
    point2.y = point2_y;
    point2.z = 0.5;
    
    arrow_marker.points.push_back(point1);
    arrow_marker.points.push_back(point2);

    arrow_marker.scale.x = 0.1;
    arrow_marker.scale.y = 0.3;
    arrow_marker.scale.z = 0.3;

    arrow_marker.color.r = rgb[red];
    arrow_marker.color.g = rgb[green];
    arrow_marker.color.b = rgb[blue];
    arrow_marker.color.a = 1.0f;

    arrow_marker.lifetime = rclcpp::Duration(0);
    arrow_marker.frame_locked = false;

    markers_array_to_remove.push_back(arrow_marker);
    
    return arrow_marker;
  }

  void ActorPanel::closeGoalsWindow(){
    window1->close();
    window->activateWindow();

    if(goals_number != 1){
      goals_button->setEnabled(false);
      save_button->setEnabled(true);
      reset_goals->setEnabled(true);
    }
    
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onNewGoal(double,double,double,QString)));
    first_actor = true;
  }

  void ActorPanel::closeInitialPoseWindow(){
    window2->close();
    window->activateWindow();
    
    if(initial_pose_set){
      initial_pose_button->setEnabled(false);
      goals_button->setEnabled(true);
      reset_initial_pose->setEnabled(true);
    }
    
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onInitialPose(double,double,double,QString)));
    QObject::disconnect(initial_pose_connection);
  }

  void ActorPanel::randomRGB(){
    red = rand()%(1-0 + 1) + 0;
    green = rand()%(1-0 + 1) + 0;
    blue = rand()%(1-0 + 1) + 0;
  }

  void ActorPanel::removeCurrentMarkers(){
    
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    auto marker_array1 = std::make_unique<visualization_msgs::msg::MarkerArray>();
    
    visualization_msgs::msg::Marker markerDeletionIP;
    markerDeletionIP.header.frame_id = "map";    
    markerDeletionIP.action = visualization_msgs::msg::Marker::DELETEALL;

    visualization_msgs::msg::Marker markerDeletionG;
    markerDeletionG.header.frame_id = "map";    
    markerDeletionG.action = visualization_msgs::msg::Marker::DELETEALL;

    marker_array->markers.push_back(markerDeletionIP);
    marker_array1->markers.push_back(markerDeletionG);

    initial_pose_publisher->publish(std::move(marker_array));
    goals_publisher->publish(std::move(marker_array1));  

  }

  void ActorPanel::resetGoal(){
    int size = static_cast<int>(markers_array_to_remove.size());

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker m;

    for(int i = 0; i < size; i++){
      markers_array_to_remove[i].header.frame_id = "map";
      markers_array_to_remove[i].action = visualization_msgs::msg::Marker::DELETE;
      markers.markers.push_back(markers_array_to_remove[i]);
    }

    goals_publisher->publish(markers);
    
    markers_array_to_remove.clear();
    marker_array.markers.clear();
    poses.clear();

    if(initial_pose_button->isEnabled()){
      goals_button->setEnabled(false);
    }
    else{
      goals_button->setEnabled(true);
    }
    
    reset_goals->setEnabled(false);
    save_button->setEnabled(false);

    oldPose = stored_pose;
  }

  void ActorPanel::resetInitialPose(){
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker marker_to_remove = initial_pose_marker_array.markers.back();

    marker_to_remove.header.frame_id = "map";
    marker_to_remove.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(marker_to_remove);

    initial_pose_publisher->publish(markers);

    initial_pose_marker_array.markers.pop_back();

    reset_initial_pose->setEnabled(false);
    initial_pose_button->setEnabled(true);

    resetGoal();
  }

  void ActorPanel::openFileExplorer(bool file){
    QString fileName;
    
    if (file){
      fileName = QFileDialog::getOpenFileName(this, tr("Open file"), "/home", tr("YAML Files (*.yaml)"));
      show_file_selector_once = true;
      checkbox->setChecked(true);
    }
    else{
      fileName = QFileDialog::getExistingDirectory(this, tr("Open folder"), "/home", QFileDialog::ShowDirsOnly);
      window->activateWindow();
    }

    dir = fileName.toStdString();
  }

   // // Destructor, close and disconnect windows.
  // void ActorPanel::exitAgentCreation(){
    
  //   if(window->isVisible()){
  //     window->close();
  //   }
    
  //   if(window1->isVisible()){
  //     window1->close();
  //   }

  //   if(window2->isVisible()){
  //     window2->close();
  //   }

  //   // resetInitialPose();

  //   // Clear goals_marker in order to get new goals if necessary
  //   marker_array = visualization_msgs::msg::MarkerArray();
  //   //Reset initial pose to false in case user does not close the panel after writing the file.
  //   // If initial pose is not reseted in the next iteration the initial pose could be empty.
  //   initial_pose_set = false;
    
  //   show_file_selector_once = true;

  //   // Enables first windows button in order to allow agent creation again
  //   // open_button->setEnabled(true);
  //   // actor_button->setEnabled(true);
  //   initial_pose_set = false;
  //   markers_array_to_remove.clear();
  //   // agent_name->clear();
  //   poses.clear();
  // }
  
  void ActorPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("Topic", output_topic_);
  }

  // Load all configuration data for this panel from the given Config object.
  void ActorPanel::load(const rviz_common::Config &config)
  {
    rviz_common::Panel::load(config);
    /*QString topic;
    if (config.mapGetString("Topic", &topic))
    {
      output_topic_editor_->setText(topic);
      updateTopic();
    }*/
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hunav_rviz2_panel::ActorPanel, rviz_common::Panel)