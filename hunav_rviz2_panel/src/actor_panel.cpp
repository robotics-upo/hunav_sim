#include <stdio.h>
#include <iostream>
#include <fstream>

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
      : rviz_common::Panel(parent), rclcpp::Node("clicked_point")
  {
    
    QVBoxLayout *topic_button = new QVBoxLayout;

    topic_button->addWidget(new QLabel("Open yaml file"));
    
    QPushButton *open_button = new QPushButton("Open");
    topic_button->addWidget(open_button);

    topic_button->addWidget(new QLabel("Specify number of agents to generate: "));

    actors = new QLineEdit;
    topic_button->addWidget(actors);

    QPushButton *actor_button = new QPushButton("Add actor");
    topic_button->addWidget(actor_button);
    connect(actor_button, SIGNAL(clicked()), this, SLOT(addActor()));

    QHBoxLayout *layout = new QHBoxLayout;
    layout->addLayout(topic_button);
    setLayout(layout);

  }

  void ActorPanel::addActor(){
    QObject *button = QObject::sender();

    if (button)
    {
    
      window = new QWidget;

      QVBoxLayout *topic_layout = new QVBoxLayout(window);
      QHBoxLayout *layout = new QHBoxLayout;
      
      topic_layout->addWidget(new QLabel("Actor"));

      topic_layout->addWidget(new QLabel("Actor name:"));
      output_topic_editor_ = new QLineEdit;
      topic_layout->addWidget(output_topic_editor_);

      topic_layout->addWidget(new QLabel("Behavior:"));
      output_topic_editor_1 = new QLineEdit;
      topic_layout->addWidget(output_topic_editor_1);

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
      connect(coordinates_button, SIGNAL(clicked()), this, SLOT(getCoordinates()));
      connect(initial_pose_button, SIGNAL(clicked()), this, SLOT(setInitialPose()));
    }
  }

  void ActorPanel::setInitialPose(){
    
    QObject *c = new QObject();
    c->connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onInitialPose(double,double,double,QString)));

    window2 = new QWidget();
    QVBoxLayout *topic_layout = new QVBoxLayout(window2);
    QHBoxLayout *layout = new QHBoxLayout;

    
    topic_layout->addWidget(new QLabel("Initial pose"));

    topic_layout->addWidget(new QLabel("x: " + QString::fromStdString(std::to_string(initial_pose.pose.position.x))));
        
    topic_layout->addWidget(new QLabel("y: " + QString::fromStdString(std::to_string(initial_pose.pose.position.y))));
    
    topic_layout->addWidget(new QLabel("z: " + QString::fromStdString(std::to_string(initial_pose.pose.position.z))));

    layout->addLayout(topic_layout);
    setLayout(layout);

    window2->show();

    c->deleteLater();
  }

  void ActorPanel::getCoordinates(){

    client_node_ = std::make_shared<rclcpp::Node>("_");

    QObject *c = new QObject();
    c->connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onNewGoal(double,double,double,QString)));

/*
    QObject::connect(
    &GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),
    this, SLOT(onNewGoal(double,double,double,QString)));*/

    window1 = new QWidget();
    QVBoxLayout *topic_layout = new QVBoxLayout(window1);
    QHBoxLayout *layout = new QHBoxLayout;

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

    connect(close_button, SIGNAL(clicked()), this, SLOT(closeGoalsWindow()));

    c->deleteLater();
    
  }

  void ActorPanel::onInitialPose(double x, double y, double theta, QString frame)
  {
    initial_pose = geometry_msgs::msg::PoseStamped(); // Intentar cambiar esto a PointStamped, pero de momento funciona así

    initial_pose.header.stamp = rclcpp::Clock().now();
    initial_pose.header.frame_id = frame.toStdString();
    initial_pose.pose.position.x = x;
    initial_pose.pose.position.y = y;
    initial_pose.pose.position.z = 0.0;
    initial_pose.pose.position.z = theta;
    
  }

  void ActorPanel::onNewGoal(double x, double y, double theta, QString frame)
  {
    pose = geometry_msgs::msg::PoseStamped(); // Intentar cambiar esto a PointStamped, pero de momento funciona así

    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = frame.toStdString();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.position.z = theta;

    poses.push_back(pose);
/*
    flag_resource = "/home/roberto/Desktop/flag.dae";

    Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
    node->attachObject( entity );
    node->setVisible( true );
    node->setPosition( position );
    flag_nodes_.push_back( node );*/
    
  }

  void ActorPanel::topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    point[0] = std::to_string(msg->point.x);
    point[1] = std::to_string(msg->point.y);
    point[2] = std::to_string(msg->point.z);

    //RCLCPP_INFO(this->get_logger(), "I heard '%s'", msg->point.x);
    std::ofstream file;
    file.open("/home/roberto/Desktop/output_clicked.txt");
    file << "I heard " + std::to_string(msg->point.x);
    file.close();
  }

  void ActorPanel::exitWindow(){

    window->close();

    std::ofstream file;
    file.open("/home/roberto/Desktop/filename.yaml");

    QString temp = actors->text();
    num_actors = temp.toInt();

    // Get input from user
    std::string name = output_topic_editor_->text().toStdString();
    std::string behavior = output_topic_editor_1->text().toStdString();
    std::string skin = output_topic_editor_2->text().toStdString();

    // Fill name's array for later use
    names.push_back(name);

    // Node's creation to generate yaml config file
    YAML::Node g3;
    g3["g3"] = YAML::Null;

    YAML::Node g3_info;
    g3_info["x"] = std::to_string(poses[2].pose.position.x);
    g3_info["y"] = std::to_string(poses[2].pose.position.y);
    g3_info["h"] = std::to_string(poses[2].pose.position.z);

    g3["g3"].push_back(g3_info);

    YAML::Node g2;
    g2["g2"] = YAML::Null;
    
    YAML::Node g2_info;
    g2_info["x"] = std::to_string(poses[1].pose.position.x);
    g2_info["y"] = std::to_string(poses[1].pose.position.y);
    g2_info["h"] = std::to_string(poses[1].pose.position.z);

    g2["g2"].push_back(g2_info);

    YAML::Node g1;
    g1["g1"] = YAML::Null;
    
    YAML::Node g1_info;
    g1_info["x"] = std::to_string(poses[0].pose.position.x);
    g1_info["y"] = std::to_string(poses[0].pose.position.y);
    g1_info["h"] = std::to_string(poses[0].pose.position.z);

    g1["g1"].push_back(g1_info);

    YAML::Node goals;
    goals["goals"] = YAML::Null;
    goals["goals"].push_back("g1");
    goals["goals"].push_back("g2");
    goals["goals"].push_back("g3");

    YAML::Node cyclic_goals;
    cyclic_goals["cyclic_goals"] = true;

    YAML::Node goal_radius;
    goal_radius["goal_radius"] = "0.3";

    YAML::Node init_pose;
    init_pose["init_pose"] = YAML::Null;

    YAML::Node init_pose_info;
    init_pose_info["x"] = std::to_string(initial_pose.pose.position.x);
    init_pose_info["y"] = std::to_string(initial_pose.pose.position.y);
    init_pose_info["z"] = std::to_string(initial_pose.pose.position.z);
    init_pose_info["h"] = "0.0";//std::to_string(initial_pose.pose.position.z);

    init_pose["init_pose"].push_back(init_pose_info);

    YAML::Node radius;
    radius["radius"] = "0.4";

    YAML::Node max_vel;
    max_vel["max_vel"] = "1.5";

    YAML::Node group_id;
    group_id["group_id"] = "-1";

    YAML::Node behavior_info;
    behavior_info["behavior"] = behavior;

    YAML::Node skin_info;
    skin_info["skin"] = skin;

    YAML::Node id;
    id["id"] = iterate_actors;

    YAML::Node agent1;
    agent1[name] = YAML::Null;
    agent1[name].push_back(id);
    agent1[name].push_back(skin_info);
    agent1[name].push_back(behavior_info);
    agent1[name].push_back(group_id);
    agent1[name].push_back(max_vel);
    agent1[name].push_back(radius);
    agent1[name].push_back(init_pose);
    agent1[name].push_back(goal_radius);
    agent1[name].push_back(cyclic_goals);
    agent1[name].push_back(goals);
    agent1[name].push_back(g1);
    agent1[name].push_back(g2);
    agent1[name].push_back(g3);

    // Fill actors array
    actors_info.push_back(agent1);

    if(iterate_actors == num_actors){
      YAML::Node hunav_loader;
      YAML::Node ros_parameters;
      YAML::Node map;
      YAML::Node agents;

      hunav_loader["hunav_loader"] = YAML::Null;
      ros_parameters["ros_parameters"] = YAML::Null;
      map["map"] = "cafe";
      agents["agents"] = YAML::Null;

      for (auto i = names.begin(); i != names.end(); ++i)
        agents["agents"].push_back(*i);

      ros_parameters["ros_parameters"].push_back(map);
      ros_parameters["ros_parameters"].push_back(agents);

      for (auto i = actors_info.begin(); i != actors_info.end(); ++i)
        ros_parameters["ros_parameters"].push_back(*i);

      hunav_loader["hunav_loader"].push_back(ros_parameters);

      //Write hunav_loader node to file
      file << hunav_loader;

      // Close the file
      file.close();

    }
    else{
      iterate_actors++;
      
      output_topic_editor_->clear();
      output_topic_editor_1->clear();
      output_topic_editor_2->clear();

      output_topic_editor_->setText("");
      output_topic_editor_1->setText("");
      output_topic_editor_2->setText("");

      poses.clear();
      addActor();
    }

  }

  void ActorPanel::closeGoalsWindow(){
    window1->close();
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
    QString topic;
    if (config.mapGetString("Topic", &topic))
    {
      output_topic_editor_->setText(topic);
      updateTopic();
    }
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hunav_rviz2_panel::ActorPanel, rviz_common::Panel)


