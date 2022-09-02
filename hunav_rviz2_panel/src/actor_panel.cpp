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

#include "headers/actor_panel.hpp"
#include "headers/goal_pose_updater.hpp"

using std::placeholders::_1;

namespace hunav_rviz2_panel
{

  GoalPoseUpdater GoalUpdater;

  ActorPanel::ActorPanel(QWidget *parent)
      : rviz_common::Panel(parent), rclcpp::Node("hunav_agents")
  {
    
    QVBoxLayout *topic_button = new QVBoxLayout;

    topic_button->addWidget(new QLabel("Open yaml file"));
    
    QPushButton *open_button = new QPushButton("Open");
    topic_button->addWidget(open_button);

    topic_button->addWidget(new QLabel("Specify number of agents to generate: "));

    actors = new QLineEdit;
    topic_button->addWidget(actors);

    QPushButton *actor_button = new QPushButton("Create agents");
    topic_button->addWidget(actor_button);
    connect(actor_button, SIGNAL(clicked()), this, SLOT(addActor()));

    QHBoxLayout *layout = new QHBoxLayout;
    layout->addLayout(topic_button);
    setLayout(layout);

    initial_pose_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("hunav_agent", rclcpp::QoS(1).transient_local());
    goals_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("hunav_goals", rclcpp::QoS(1).transient_local());

  }

  ActorPanel::~ActorPanel(){
    window->close();
    window1->close();
    window2->close();
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onInitialPose(double,double,double,QString)));
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onNewGoal(double,double,double,QString)));
  }

  void ActorPanel::addActor(){
    QObject *button = QObject::sender();

    if (button)
    {
    
      window = new QWidget;

      QVBoxLayout *topic_layout = new QVBoxLayout(window);
      QHBoxLayout *layout = new QHBoxLayout;
      
      topic_layout->addWidget(new QLabel("Agent"));

      topic_layout->addWidget(new QLabel("Agent's name:"));
      output_topic_editor_ = new QLineEdit;
      topic_layout->addWidget(output_topic_editor_);

      topic_layout->addWidget(new QLabel("Behavior:"));

      behavior_combobox = new QComboBox();

      behavior_combobox->addItem("Regular");
      behavior_combobox->addItem("Impassive");
      behavior_combobox->addItem("Surprised");
      behavior_combobox->addItem("Scared");
      behavior_combobox->addItem("Curious");
      behavior_combobox->addItem("Threathening");

      //behavior_combobox->setCurrentIndex(0);

      topic_layout->addWidget(behavior_combobox);

      topic_layout->addWidget(new QLabel("Skin:"));
      output_topic_editor_2 = new QLineEdit;
      topic_layout->addWidget(output_topic_editor_2);

      layout->addLayout(topic_layout);
      setLayout(layout);
      
      QPushButton *initial_pose_button = new QPushButton("Set initial pose");
      QPushButton *coordinates_button = new QPushButton("Set goals");
      QPushButton *save_button = new QPushButton("Save");

      topic_layout->addWidget(initial_pose_button);
      topic_layout->addWidget(coordinates_button);
      topic_layout->addWidget(save_button);

      window->show();

      connect(save_button, SIGNAL(clicked()), this, SLOT(exitWindow()));
      connect(coordinates_button, SIGNAL(clicked()), this, SLOT(getNewGoal()));
      connect(initial_pose_button, SIGNAL(clicked()), this, SLOT(setInitialPose()));
    }
  }

  void ActorPanel::setInitialPose(){
    
    initial_pose_connection = new QObject();
    initial_pose_connection->connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onInitialPose(double,double,double,QString)));

    window2 = new QWidget();
    QVBoxLayout *topic_layout = new QVBoxLayout(window2);
    QHBoxLayout *layout = new QHBoxLayout;

    
    topic_layout->addWidget(new QLabel("Initial pose"));

    topic_layout->addWidget(new QLabel("x: " + QString::fromStdString(std::to_string(initial_pose.pose.position.x))));
        
    topic_layout->addWidget(new QLabel("y: " + QString::fromStdString(std::to_string(initial_pose.pose.position.y))));
    
    topic_layout->addWidget(new QLabel("z: " + QString::fromStdString(std::to_string(initial_pose.pose.position.z))));

    QPushButton *close_button = new QPushButton("Close");
    topic_layout->addWidget(close_button);
    
    layout->addLayout(topic_layout);
    setLayout(layout);

    window2->show();

    initial_pose_connection->deleteLater();

    connect(close_button, SIGNAL(clicked()), this, SLOT(closeInitialPoseWindow()));
  }

  void ActorPanel::getNewGoal(){

    client_node_ = std::make_shared<rclcpp::Node>("_");

    goals_connection = new QObject();
    goals_connection->connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onNewGoal(double,double,double,QString)));

/*
    QObject::connect(
    &GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onNewGoal(double,double,double,QString)));*/

    window1 = new QWidget();
    QVBoxLayout *topic_layout = new QVBoxLayout(window1);
    QHBoxLayout *layout = new QHBoxLayout;

    goals_number = 1;

    QString goal = "Goals";
    std::vector<QString> goals{"G1","G2","G3"};
    int i = 0;

    topic_layout->addWidget(new QLabel("Please, select all three goals at the same time."));
    topic_layout->addWidget(new QLabel(goal));
/*
    for(geometry_msgs::msg::PoseStamped aux : poses){
      topic_layout->addWidget(new QLabel(goals[i])); 
      coordinates = new QLineEdit;
      coordinates1 = new QLineEdit;
      coordinates2 = new QLineEdit;

      topic_layout->addWidget(new QLabel("x:"));
      coordinates->setText(QString::fromStdString(std::to_string(aux.pose.position.x)));
      topic_layout->addWidget(coordinates);
      
      topic_layout->addWidget(new QLabel("y:"));
      coordinates1->setText(QString::fromStdString(std::to_string(aux.pose.position.y)));
      topic_layout->addWidget(coordinates1);

      topic_layout->addWidget(new QLabel("z:"));
      coordinates2->setText(QString::fromStdString(std::to_string(aux.pose.position.z)));
      topic_layout->addWidget(coordinates2);
      i++;
    }*/

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
    }

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

  void ActorPanel::onInitialPose(double x, double y, double theta, QString frame)
  {
    initial_pose = geometry_msgs::msg::PoseStamped();

    initial_pose.header.stamp = rclcpp::Clock().now();
    initial_pose.header.frame_id = frame.toStdString();
    initial_pose.pose.position.x = x;
    initial_pose.pose.position.y = y;
    initial_pose.pose.position.z = 0.0;
    initial_pose.pose.position.z = theta;

    srand((unsigned) time(NULL));
    red = rand()%(1-0 + 1) + 0;
    green = rand()%(1-0 + 1) + 0;
    blue = rand()%(1-0 + 1) + 0;

    visualization_msgs::msg::Marker marker;
    uint32_t shape = visualization_msgs::msg::Marker::CYLINDER;
    marker.header.frame_id = "/map";
    marker.header.stamp = rclcpp::Node::now();
    marker.ns = "basic_shapes";
    marker.id = marker_id;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
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

    marker_id++;

    auto initial_pose_marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    initial_pose_marker_array->markers.push_back(marker);

    initial_pose_publisher->publish(std::move(initial_pose_marker_array));

    oldPose = initial_pose;
    stored_pose = initial_pose;

    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onInitialPose(double,double,double,QString)));
    
  }

  void ActorPanel::onNewGoal(double x, double y, double theta, QString frame)
  {

    pose = geometry_msgs::msg::PoseStamped();

    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = frame.toStdString();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.position.z = theta;

    poses.push_back(pose);

    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = "/map";
    marker.header.stamp = rclcpp::Node::now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = rgb[red];
    marker.color.g = rgb[green];
    marker.color.b = rgb[blue];
    marker.color.a = 1.0; // alpha has to be non-zero

    marker.lifetime = rclcpp::Duration(0);
    marker.frame_locked = false;

    marker_id++;
    
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    
    visualization_msgs::msg::Marker arrow_marker;

    arrow_marker.header.frame_id = "/map";
    arrow_marker.header.stamp = rclcpp::Node::now();
    arrow_marker.id = marker_id;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point point1;
    point1.x = oldPose.pose.position.x;
    point1.y = oldPose.pose.position.y;
    point1.z = 0.0;

    geometry_msgs::msg::Point point2;
    point2.x = x-0.2;
    point2.y = y-0.2;
    point2.z = 0.0;
    
    arrow_marker.points.push_back(point1);
    arrow_marker.points.push_back(point2);

    arrow_marker.scale.x = 0.1;
    arrow_marker.scale.y = 0.3;
    arrow_marker.scale.z = 0.3;

    //arrow_marker.color.r = 1.0;
    arrow_marker.color.r = rgb[red];
    arrow_marker.color.g = rgb[green];
    arrow_marker.color.b = rgb[blue];
    arrow_marker.color.a = 1.0f;

    arrow_marker.lifetime = rclcpp::Duration(0);
    arrow_marker.frame_locked = false;

    // Increment marker id for next marker
    marker_id++;
    
    int marker_array_size = static_cast<int>(marker_array->markers.size());

    for(int i = 0; i < marker_array_size; i++){
      marker_array->markers[i].header.stamp = rclcpp::Node::now();
    }

    marker_array->markers.push_back(marker);
    marker_array->markers.push_back(arrow_marker);

    // If goals == 3 means that another arrow has to be added to close the path
    if(goals_number == 3){
      visualization_msgs::msg::Marker arrow_marker1;

      arrow_marker1.header.frame_id = "/map";
      arrow_marker1.header.stamp = rclcpp::Node::now();
      arrow_marker1.id = marker_id;
      arrow_marker1.type = visualization_msgs::msg::Marker::ARROW;
      arrow_marker1.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point point3;
      point3.x = x;
      point3.y = y;
      point3.z = 0.0;

      geometry_msgs::msg::Point point4;
      point4.x = stored_pose.pose.position.x;
      point4.y = stored_pose.pose.position.y;
      point4.z = 0.0;
      
      arrow_marker1.points.push_back(point3);
      arrow_marker1.points.push_back(point4);

      arrow_marker1.scale.x = 0.1;
      arrow_marker1.scale.y = 0.3;
      arrow_marker1.scale.z = 0.3;

      //arrow_marker1.color.r = 1.0;
      arrow_marker1.color.r = rgb[red];
      arrow_marker1.color.g = rgb[green];
      arrow_marker1.color.b = rgb[blue];
      arrow_marker1.color.a = 1.0f;

      arrow_marker1.lifetime = rclcpp::Duration(0);
      arrow_marker1.frame_locked = false;

      marker_array->markers.push_back(arrow_marker1);
    }
    
    oldPose = pose;
    first_actor = false;

    goals_publisher->publish(std::move(marker_array));

    goals_number++;

    //disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onNewGoal(double,double,double,QString)));

    QObject::disconnect(goals_connection);
    
  }

  void ActorPanel::exitWindow(){

    window->close();

    std::ofstream file;
    file.open("/home/roberto/Desktop/filename.yaml");

    QString temp = actors->text();
    num_actors = temp.toInt();

    // Get input from user
    std::string name = output_topic_editor_->text().toStdString();
    std::string behavior = std::to_string(checkComboBox());
    std::string skin = output_topic_editor_2->text().toStdString();

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
    agent1["goals"].push_back("g1");
    agent1["goals"].push_back("g2");
    agent1["goals"].push_back("g3");
    agent1["g1"]["x"] = std::to_string(poses[0].pose.position.x);
    agent1["g1"]["y"] = std::to_string(poses[0].pose.position.y);
    agent1["g1"]["h"] = std::to_string(poses[0].pose.position.z);

    agent1["g2"]["x"] = std::to_string(poses[1].pose.position.x);
    agent1["g2"]["y"] = std::to_string(poses[1].pose.position.y);
    agent1["g2"]["h"] = std::to_string(poses[1].pose.position.z);

    agent1["g3"]["x"] = std::to_string(poses[2].pose.position.x);
    agent1["g3"]["y"] = std::to_string(poses[2].pose.position.y);
    agent1["g3"]["h"] = std::to_string(poses[2].pose.position.z);

    // Fill actors array
    actors_info.push_back(agent1);

    if(iterate_actors == num_actors){
      YAML::Node hunav_loader;
      //YAML::Node ros_parameters;
      //YAML::Node map;
      //YAML::Node publish_people;
      //YAML::Node agents;

      hunav_loader["hunav_loader"]["ros_parameters"]["map"] = "cafe";
      hunav_loader["hunav_loader"]["ros_parameters"]["publish_people"] = true;
      
      for (auto i = names.begin(); i != names.end(); ++i)
        hunav_loader["hunav_loader"]["ros_parameters"]["agents"].push_back(*i);
        //agents["agents"].push_back(*i);

      int names_counter = 0;

      for (auto i = actors_info.begin(); i != actors_info.end(); ++i){
        hunav_loader["hunav_loader"]["ros_parameters"][names[names_counter]] = *i;
        names_counter++;
      }
        
      //Write hunav_loader node to file
      file << hunav_loader;

      // Close the file
      file.close();

    }
    else{
      iterate_actors++;
      
      output_topic_editor_->clear();
      output_topic_editor_2->clear();

      output_topic_editor_->setText("");
      output_topic_editor_2->setText("");

      poses.clear();

      addActor();
    }

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

  void ActorPanel::closeGoalsWindow(){
    window1->close();
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onNewGoal(double,double,double,QString)));
    first_actor = true;
  }

  void ActorPanel::closeInitialPoseWindow(){
    window2->close();
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), this, SLOT(onInitialPose(double,double,double,QString)));
    QObject::disconnect(initial_pose_connection);
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


