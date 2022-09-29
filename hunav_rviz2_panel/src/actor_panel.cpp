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

    topic_button->addWidget(new QLabel("Open yaml file (agents.yaml)"));
    
    QPushButton *open_button = new QPushButton("Open");
    topic_button->addWidget(open_button);

    topic_button->addWidget(new QLabel("Set number of agents to generate: "));

    actors = new QLineEdit;
    topic_button->addWidget(actors);

    QPushButton *actor_button = new QPushButton("Create agents");
    topic_button->addWidget(actor_button);

    checkbox = new QCheckBox("Save file in default directory", this);
    checkbox->setChecked(true);
    topic_button->addWidget(checkbox);

    connect(actor_button, SIGNAL(clicked()), this, SLOT(addAgent()));
    connect(open_button, SIGNAL(clicked()), this, SLOT(parseYaml()));

    QHBoxLayout *layout = new QHBoxLayout;
    layout->addLayout(topic_button);
    setLayout(layout);

    // Create publisher for agents' 
    // Agents' intial pose are published in /hunav_agent topic.
    initial_pose_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("hunav_agent", rclcpp::QoS(1).transient_local());
    // Agents' goals are published in /hunav_goals.
    goals_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("hunav_goals", rclcpp::QoS(1).transient_local());

  }

  // Destructor, close and disconnect windows.
  ActorPanel::~ActorPanel(){
    window->close();
    window1->close();
    window2->close();
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onInitialPose(double,double,double,QString)));
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onNewGoal(double,double,double,QString)));
  }

  // Shows the agent creation window.
  void ActorPanel::addAgent(){
    QObject *button = QObject::sender();

    if (button)
    {

      window = new QWidget;

      QVBoxLayout *topic_layout = new QVBoxLayout(window);
      QHBoxLayout *layout = new QHBoxLayout;
      QPushButton *directory;

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
      output_topic_editor_ = new QLineEdit;
      topic_layout->addWidget(output_topic_editor_);

      topic_layout->addWidget(new QLabel("Behavior:"));

      // Combobox for behavior selection.
      behavior_combobox = new QComboBox();

      behavior_combobox->addItem("Regular");
      behavior_combobox->addItem("Impassive");
      behavior_combobox->addItem("Surprised");
      behavior_combobox->addItem("Scared");
      behavior_combobox->addItem("Curious");
      behavior_combobox->addItem("Threathening");

      topic_layout->addWidget(behavior_combobox);

      topic_layout->addWidget(new QLabel("Skin:"));
      
      // Combobox for skin selection.
      skin_combobox = new QComboBox();
      
      skin_combobox->addItem("Blue jeans");
      skin_combobox->addItem("Green t-shirt");
      skin_combobox->addItem("Blue t-shirt");
      skin_combobox->addItem("Red t-shirt");

      topic_layout->addWidget(skin_combobox);

      layout->addLayout(topic_layout);
      setLayout(layout);
      
      QPushButton *initial_pose_button = new QPushButton("Set initial pose");

      QLabel *num_goals_set_label = new QLabel("Set number of goals");
      num_goals_set = new QLineEdit();

      goals_button = new QPushButton("Set goals");
      save_button = new QPushButton("Save");

      save_button->setEnabled(false);
      goals_button->setEnabled(false);

      topic_layout->addWidget(initial_pose_button);
      topic_layout->addWidget(num_goals_set_label);
      topic_layout->addWidget(num_goals_set);
      topic_layout->addWidget(goals_button);
      topic_layout->addWidget(save_button);

      window->show();

      connect(save_button, SIGNAL(clicked()), this, SLOT(saveAgents()));
      connect(goals_button, SIGNAL(clicked()), this, SLOT(getNewGoal()));
      connect(initial_pose_button, SIGNAL(clicked()), this, SLOT(setInitialPose()));
      if(!checkbox->isChecked() && show_file_selector_once){
        connect(directory, SIGNAL(clicked()), this, SLOT(openFileExplorer()));
        show_file_selector_once = false;
      }
    }
  }

  // Window that allow user to start using the HuNavGoal tool to select the agent's initial pose.
  void ActorPanel::setInitialPose(){
    
    initial_pose_connection = new QObject();
    initial_pose_connection->connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onInitialPose(double,double,double,QString)));

    window2 = new QWidget();
    topic_layout_init_pose = new QVBoxLayout(window2);
    QHBoxLayout *layout = new QHBoxLayout;

    
    topic_layout_init_pose->addWidget(new QLabel("Initial pose"));
    topic_layout_init_pose->addWidget(new QLabel("Use HunavGoals tool to select the initial pose"));
    /*
    topic_layout_init_pose->addWidget(new QLabel("x: " + QString::fromStdString(std::to_string(initial_pose.pose.position.x))));
        
    topic_layout_init_pose->addWidget(new QLabel("y: " + QString::fromStdString(std::to_string(initial_pose.pose.position.y))));
    
    topic_layout_init_pose->addWidget(new QLabel("z: " + QString::fromStdString(std::to_string(initial_pose.pose.position.z))));
    */
    QPushButton *close_button = new QPushButton("Close");
    topic_layout_init_pose->addWidget(close_button);
    
    layout->addLayout(topic_layout_init_pose);
    setLayout(layout);

    window2->show();

    initial_pose_connection->deleteLater();

    connect(close_button, SIGNAL(clicked()), this, SLOT(closeInitialPoseWindow()));
  }

  // Window that allow users to start using the HuNavGoal tool to select the agent's goals.
  void ActorPanel::getNewGoal(){

    client_node_ = std::make_shared<rclcpp::Node>("_");

    goals_connection = new QObject();
    goals_connection->connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onNewGoal(double,double,double,QString)));

    window1 = new QWidget();
    QVBoxLayout *topic_layout = new QVBoxLayout(window1);
    QHBoxLayout *layout = new QHBoxLayout;

    goals_number = 1;

    QString goal = "Goals";
    std::vector<QString> goals;
    //int i = 0;

    topic_layout->addWidget(new QLabel(goal));

    // If num_goals not set, by default will be 3 goals.
    if(num_goals_set->text().isEmpty()){
      num_goals_set->setText(QString::number(3));
    }

    // Fill goal's vector
    for(int j = 0; j < num_goals_set->text().toInt(); j++){
      goals.push_back("G" + QString::number(j));
    }

    topic_layout->addWidget(new QLabel("Use HunavGoals tool to select goals"));
    /*
    // Iterate goal's vector to show selected goals
    for(QString aux : goals){
      topic_layout->addWidget(new QLabel(aux));  
      
      if(!poses.empty()){
        
        topic_layout->addWidget(new QLabel("x: " + QString::fromStdString(std::to_string(poses[i].pose.position.x))));
        
        topic_layout->addWidget(new QLabel("y: " + QString::fromStdString(std::to_string(poses[i].pose.position.y))));
        
        topic_layout->addWidget(new QLabel("z: " + QString::fromStdString(std::to_string(poses[i].pose.position.z))));
        
        i++;
      }
      else{
        topic_layout->addWidget(new QLabel("No goals yet"));
      }

    }*/

    QPushButton *close_button = new QPushButton("Close");
    topic_layout->addWidget(close_button);

    layout->addLayout(topic_layout);
    setLayout(layout);

    window1->show();

    goals_connection->deleteLater();

    // Removes older data from inital_pose
    initial_pose = geometry_msgs::msg::PoseStamped();
    
    connect(close_button, SIGNAL(clicked()), this, SLOT(closeGoalsWindow()));
    
  }

  // Get point from map to store it in initial pose.
  void ActorPanel::onInitialPose(double x, double y, double theta, QString frame)
  {
    initial_pose = geometry_msgs::msg::PoseStamped();

    initial_pose.header.stamp = rclcpp::Clock().now();
    initial_pose.header.frame_id = frame.toStdString();
    initial_pose.pose.position.x = x;
    initial_pose.pose.position.y = y;
    pose.pose.position.z = theta;
    initial_pose.pose.position.z = 1.25;
    theta = 0;

    srand((unsigned) time(NULL));
    red = rand()%(1-0 + 1) + 0;
    green = rand()%(1-0 + 1) + 0;
    blue = rand()%(1-0 + 1) + 0;

    visualization_msgs::msg::Marker marker;
    marker = createCylinderMarker(x, y, marker_id);

    marker_id++;

    auto initial_pose_marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    initial_pose_marker_array->markers.push_back(marker);

    initial_pose_publisher->publish(std::move(initial_pose_marker_array));

    oldPose = initial_pose;
    stored_pose = initial_pose;

    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onInitialPose(double,double,double,QString)));

    window->activateWindow();
    window2->activateWindow();
  }

  // Get point from map to store goals.
  void ActorPanel::onNewGoal(double x, double y, double theta, QString frame)
  {

    pose = geometry_msgs::msg::PoseStamped();

    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = frame.toStdString();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = theta;
    pose.pose.position.z = 1.25;

    poses.push_back(pose);

    visualization_msgs::msg::Marker marker;
    marker = createCubeMarker(x, y, marker_id);

    marker_id++;
    
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    
    visualization_msgs::msg::Marker arrow_marker;

    arrow_marker = createArrowMarker(oldPose.pose.position.x, oldPose.pose.position.y, x, y, marker_id);

    // Increment marker id for next marker.
    marker_id++;
    
    int marker_array_size = static_cast<int>(marker_array->markers.size());

    for(int i = 0; i < marker_array_size; i++){
      marker_array->markers[i].header.stamp = rclcpp::Node::now();
    }

    marker_array->markers.push_back(marker);
    marker_array->markers.push_back(arrow_marker);

    // If goals == num_goals_set means that another arrow has to be added to close the path.
    if(goals_number == num_goals_set->text().toInt()){
      visualization_msgs::msg::Marker arrow_marker1;

      arrow_marker1 = createArrowMarker(x,y,stored_pose.pose.position.x, stored_pose.pose.position.y, marker_id);

      marker_array->markers.push_back(arrow_marker1);
    }
    
    oldPose = pose;
    first_actor = false;

    goals_publisher->publish(std::move(marker_array));

    goals_number++;

    QObject::disconnect(goals_connection);

    window->activateWindow();
    window1->activateWindow();

  }

  // Saves all information about agents in yaml file
  void ActorPanel::saveAgents(){

    window->close();

    // If checkbox is not checked, it means that the user wants to store file in another directory.
    if(!checkbox->isChecked()){
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
    std::ofstream file;
    file.open(pkg_shared_tree_dir_);
    
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
    if(output_topic_editor_->text().isEmpty()){
      name = "agent" + std::to_string(iterate_actors);
    }
    else{
      name = output_topic_editor_->text().toStdString();
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
    agent1["init_pose"]["h"] = "0.0";//std::to_string(initial_pose.pose.position.z);
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

      //ros__parameters needs two 
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
        
      //Writes hunav_loader node to file
      file << hunav_loader;

      // Close file
      file.close();

      RCLCPP_INFO(this->get_logger(), "Agents.yaml generated");

    }
    else{
      iterate_actors++;
      
      output_topic_editor_->clear();

      output_topic_editor_->setText("");

      poses.clear();

      agent_count++;

      addAgent();
    }

  }

  // Open a generated yaml file
  void ActorPanel::parseYaml(){
    try {
        pkg_shared_tree_dir_ =
            ament_index_cpp::get_package_share_directory("hunav_agent_manager");

      } catch (const char* msg) {
        RCLCPP_ERROR(this->get_logger(),
                    "Package hunav_agent_manager not found in dir: %s!!!",
                    pkg_shared_tree_dir_.c_str());
      }
    pkg_shared_tree_dir_ = pkg_shared_tree_dir_ + "/config/agents.yaml";

    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    
    YAML::Node yaml_file = YAML::LoadFile(pkg_shared_tree_dir_);
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
      
      // Initial pose
      visualization_msgs::msg::Marker marker;
      marker = createCylinderMarker(current_agent["init_pose"]["x"].as<double>(), current_agent["init_pose"]["y"].as<double>(), ids);

      marker_array->markers.push_back(marker);

      ids++;

      // Goals markers

      YAML::Node current_goals = current_agent["goals"];

      for(int j = 0; j < static_cast<int>(current_goals.size()); j++){
        current_goals_vector.push_back(current_agent["goals"][j].as<std::string>());
      }

      for(int k = 0; k < static_cast<int>(current_goals_vector.size()); k++){

        visualization_msgs::msg::Marker marker;
        marker = createCubeMarker(current_agent[current_goals_vector[k]]["x"].as<double>(), current_agent[current_goals_vector[k]]["y"].as<double>(), ids);

        marker_array->markers.push_back(marker);

        ids++;
        
        if(first_arrow){
          first_arrow = false;

          visualization_msgs::msg::Marker arrow_marker;

          arrow_marker = createArrowMarker(current_agent["init_pose"]["x"].as<double>(), current_agent["init_pose"]["y"].as<double>(),
          current_agent[current_goals_vector[k]]["x"].as<double>(), current_agent[current_goals_vector[k]]["y"].as<double>(), ids);

          marker_array->markers.push_back(arrow_marker);

          ids++;
        }
        else {          
          visualization_msgs::msg::Marker arrow_marker;

          arrow_marker = createArrowMarker(current_agent[current_goals_vector[k-1]]["x"].as<double>(), current_agent[current_goals_vector[k-1]]["y"].as<double>(),
          current_agent[current_goals_vector[k]]["x"].as<double>(), current_agent[current_goals_vector[k]]["y"].as<double>(), ids);

          marker_array->markers.push_back(arrow_marker);

          ids++;
          
        }
        
        if(k == static_cast<int>(current_goals_vector.size() - 1)){
          visualization_msgs::msg::Marker arrow_marker;

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

    if(aux.compare("Blue jeans") == 0){
      return 0;
    }
    else if(aux.compare("Green t-shirt") == 0){
      return 1;
    }
    else if(aux.compare("Blue t-shirt") == 0){
      return 2;
    }
    else if(aux.compare("Red t-shirt") == 0){
      return 3;
    }
    else{
      return 0;
    }
  }

  visualization_msgs::msg::Marker ActorPanel::createCylinderMarker(double point1_x, double point1_y, double ids){
    
    visualization_msgs::msg::Marker marker;
    uint32_t shape = visualization_msgs::msg::Marker::CYLINDER;
    marker.header.frame_id = "/map";
    marker.header.stamp = rclcpp::Node::now();
    marker.ns = "basic_shapes";
    marker.id = ids;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = point1_x;
    marker.pose.position.y = point1_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = rgb[red];
    marker.color.g = rgb[green];
    marker.color.b = rgb[blue];
    marker.color.a = 1.0; // alpha has to be non-zero

    return marker;

  }

  visualization_msgs::msg::Marker ActorPanel::createCubeMarker(double point1_x, double point1_y, double ids){
    
    visualization_msgs::msg::Marker marker;

    uint32_t shape = visualization_msgs::msg::Marker::CUBE;
    marker.header.frame_id = "/map";
    marker.header.stamp = rclcpp::Node::now();
    marker.ns = "basic_shapes";
    marker.id = ids;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = point1_x;
    marker.pose.position.y = point1_y;
    marker.pose.position.z = 0;//current_agent["init_pose"]["z"].as<double>();

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = rgb[red];
    marker.color.g = rgb[green];
    marker.color.b = rgb[blue];
    marker.color.a = 1.0; // alpha has to be non-zero

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
    point1.z = 0.0;

    geometry_msgs::msg::Point point2;
    point2.x = point2_x;
    point2.y = point2_y;
    point2.z = 0.0;
    
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
    
    return arrow_marker;
  }

  void ActorPanel::closeGoalsWindow(){
    window1->close();
    window->activateWindow();
    save_button->setEnabled(true);
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onNewGoal(double,double,double,QString)));
    first_actor = true;
  }

  void ActorPanel::closeInitialPoseWindow(){
    window2->close();
    window->activateWindow();
    goals_button->setEnabled(true);
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onInitialPose(double,double,double,QString)));
    QObject::disconnect(initial_pose_connection);
  }

  void ActorPanel::randomRGB(){
    red = rand()%(1-0 + 1) + 0;
    green = rand()%(1-0 + 1) + 0;
    blue = rand()%(1-0 + 1) + 0;
  }

  void ActorPanel::removeCurrentMarkers(){
    visualization_msgs::msg::Marker markerD;
    markerD.header.frame_id = "map";    
    markerD.action = visualization_msgs::msg::Marker::DELETEALL;

    marker_array->markers.push_back(markerD);

    //initial_pose_publisher->publish(std::move(marker_array));
    goals_publisher->publish(std::move(marker_array));  
  }

  void ActorPanel::openFileExplorer(){
    QString fileName = QFileDialog::getExistingDirectory(this, tr("Open Folder"), "/home", QFileDialog::ShowDirsOnly);
    dir = fileName.toStdString();
    window->activateWindow();
  }
  
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


