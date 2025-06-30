/**
 * @file actor_panel.cpp
 * @brief Implementation of ActorPanel class for RViz2 agent management
 *
 * This file implements the ActorPanel class which provides a comprehensive
 * interface for creating, editing, and managing human agents in simulation
 * environments. It supports multiple simulators and provides tools for
 * agent configuration, goal assignment, and behavior tree management.
 *
 */

// ================================ SYSTEM INCLUDES ================================
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include "random"

// ================================ QT INCLUDES ================================
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QInputDialog>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QListWidget>
#include <QDialog>
#include <QDialogButtonBox>
#include <QComboBox>
#include <QColor>
#include <QSet>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QtConcurrent/QtConcurrent>

// ================================ ROS2 INCLUDES ================================
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/tool.hpp"
#include <rviz_common/tool_manager.hpp>
#include <nav2_msgs/srv/load_map.hpp>

// ================================ MESSAGE INCLUDES ================================
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ================================ TRANSFORM INCLUDES ================================
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ================================ THIRD-PARTY INCLUDES ================================
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ================================ PROJECT INCLUDES ================================
#include "headers/actor_panel.hpp"
#include "headers/goal_pose_updater.hpp"
#include "hunav_msgs/msg/agents.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace hunav_rviz2_panel
{
  // Global goal pose updater for handling map click events
  GoalPoseUpdater GoalUpdater;

  /**
   * @brief Constructor for ActorPanel
   *
   * Initializes the RViz2 panel for agent management, sets up the UI components,
   * creates ROS2 publishers and subscribers, and configures the panel for
   * creating or editing agent configurations.
   *
   * @param parent Parent widget (RViz2 main window)
   */
  ActorPanel::ActorPanel(QWidget *parent)
      : rviz_common::Panel(parent), rclcpp::Node("hunav_rviz2_panel")
  {
    panel_mode_ = CREATE_MODE;
    // ─── Top‐Level Layout ───
    QVBoxLayout *topic_button = new QVBoxLayout;
    QHBoxLayout *layout = new QHBoxLayout;

    // 1) Header label
    topic_button->addWidget(new QLabel("Create or edit agents configuration file"));

    // 2) “Open YAML” button (left) and (for symmetry) a spacer on the right
    QVBoxLayout *top_buttons = new QVBoxLayout;
    open_button_ = new QPushButton("Load agents YAML", this);
    create_button_ = new QPushButton("Create agents YAML", this);

    open_button_->setCheckable(true);
    create_button_->setCheckable(true);
    top_buttons->addWidget(create_button_);
    auto *hsep = new QFrame(this);
    hsep->setFrameShape(QFrame::HLine);
    hsep->setStyleSheet("color: lightgray;");
    hsep->setFrameShadow(QFrame::Raised);
    hsep->setLineWidth(1);
    hsep->setMidLineWidth(5);
    top_buttons->addWidget(hsep);
    top_buttons->addWidget(open_button_);

    yaml_file_label_ = new QLabel("", this);
    yaml_file_label_->setStyleSheet("font-style: italic; color: gray;");
    yaml_file_label_->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    yaml_file_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    yaml_file_label_->hide();
    top_buttons->addWidget(yaml_file_label_);

    topic_button->addLayout(top_buttons);

    // ─── Simulator + Map Selection ───

    simulator_combo_ = new QComboBox;
    simulator_combo_->addItem("Gazebo", 1.25);
    simulator_combo_->addItem("Isaac Sim", 0.0);
    simulator_combo_->addItem("Webots", 0.01);
    simulator_combo_->setCurrentIndex(-1);

    map_group = new QGroupBox("Select simulator and map:", this);
    {
      // Outer vertical layout for group
      auto *map_layoutv = new QVBoxLayout;

      // (1) First row: simulator combo
      map_layoutv->addWidget(simulator_combo_);

      // (2) Second row: horizontal layout with button + label
      auto *map_layout = new QHBoxLayout;
      map_select_btn_ = new QPushButton("Select map", this);
      map_select_btn_->setEnabled(false);
      current_map_label_ = new QLabel("", this);
      current_map_label_->setMinimumWidth(100);
      map_layout->addWidget(map_select_btn_);
      map_layout->addWidget(current_map_label_);

      map_layoutv->addLayout(map_layout);

      // Apply the single combined layout
      map_group->setLayout(map_layoutv);
    }
    map_group->setEnabled(false);
    topic_button->addWidget(map_group);

    // Only enable “Select map” once a simulator is picked:
    connect(simulator_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx)
            { map_select_btn_->setEnabled(idx >= 0);
              if (panel_mode_ == EDIT_MODE)
                actor_button_->setEnabled(true); });

    connect(map_select_btn_, &QPushButton::clicked,
            this, &ActorPanel::onSelectMap);

    // ─── “Create / Edit agents” Section ───
    n_agents_label_ = new QLabel("Set number of agents to generate:", this);
    n_agents_label_->setEnabled(false);
    topic_button->addWidget(n_agents_label_);

    actors = new QLineEdit(this);
    actors->setEnabled(false);
    topic_button->addWidget(actors);

    actor_button_ = new QPushButton("Generate agents", this);
    actor_button_->setCheckable(true);
    actor_button_->setEnabled(false);
    // topic_button->addWidget(actor_button_);

    edit_goals_button_ = new QPushButton("Edit goals", this);
    edit_goals_button_->setCheckable(true);
    edit_goals_button_->hide();
    // topic_button->addWidget(edit_goals_button_);

    auto *editButtonsLayout = new QHBoxLayout;
    editButtonsLayout->addWidget(actor_button_);
    editButtonsLayout->addWidget(edit_goals_button_);
    topic_button->addLayout(editButtonsLayout);

    // connect(edit_goals_button_, &QPushButton::clicked, this, [this]()
    //         {
    //           removeCurrentMarkers();
    //           publishAgentMarkers();
    //           goal_markers_pub_->publish(goal_markers_);
    //           goal_group_->setEnabled(true);
    //           assign_goals_btn_->setEnabled(true); });

    connect(edit_goals_button_, &QPushButton::clicked, this, [this]()
            {
        // toggle pick‐mode flag
        goal_picking_mode_ = !goal_picking_mode_;

        // reset markers & enable them
        publishAgentMarkers();
        goal_markers_pub_->publish(goal_markers_);

        goal_group_->setEnabled(true);

        // disable “Assign Goals” until we exit pick mode
        assign_goals_btn_->setEnabled(!goal_picking_mode_);

        // the very first time only: show HTML instructions
        if (goal_picking_mode_ && !first_goal_picking_info_shown_) {
          first_goal_picking_info_shown_ = true;
          QString msg = QString(R"(
            <html>
              Click on the map to <b><i>add or edit</i></b> navigation goals.<br>
              To <b>edit</b>, just <b>click on the goal marker</b> you wish to modify.<br>
              <b>Please note</b>: Be sure to keep goals "visible" to each other (no obstacles in between) to avoid navigation issues.<br><br>
              When you’re done, click on <b><i>%1</i></b> again to exit goal-picking mode,<br>
              then click on <b><i>%2</i></b> to assign your changes or <b><i>%3</i></b> to save the file.
            </html>
          )")
            .arg(edit_goals_button_->text())
            .arg(assign_goals_btn_->text())
            .arg(save_bt_btn_->text());
          QMessageBox::information(this, tr("Add/Edit Goals"), msg);
        }

        // switch RViz into the PublishPoint tool when entering pick-mode
        if (goal_picking_mode_) {
          if (auto *tm = getDisplayContext()->getToolManager()) {
            for (int i = 0; i < tm->numTools(); ++i) {
              auto *tool = tm->getTool(i);
              if (QString(tool->getClassId()) == "rviz_default_plugins/PublishPoint") {
                tm->setCurrentTool(tool);
                break;
              }
            }
          }
          // highlight the Edit button so user knows we’re in that mode
          edit_goals_button_->setDown(true);
        } else {
          // exiting pick mode, put the button back up
          edit_goals_button_->setDown(false);
          save_bt_btn_->setEnabled(true);
        } });

    checkbox = new QCheckBox("Use default directory", this);
    checkbox->setChecked(true);
    checkbox->setEnabled(false);

    connect(create_button_, &QPushButton::clicked, this, [this]()
            {
        create_button_->setDown(true);
        open_button_->setDown(false);
        create_button_->setChecked(true);
        open_button_->setChecked(false);

        removeCurrentMarkers();
        clearDisplayedMap();  

        // Wipe out all the in-memory YAML data
        loaded_global_goals_.clear();
        loaded_agent_names_.clear();
        loaded_agent_nodes_.clear();
        loaded_agent_goals_.clear();
        loaded_initial_marker_ids_.clear();
        goal_list_widget_->clear();
        goal_ids_.clear();
        agent_goals_.clear();

        // Reset counters & containers
        panel_mode_      = CREATE_MODE;
        current_edit_idx_= 0;
        iterate_actors_  = 1;
        agent_count      = 1;
        actors_info_.clear();

        // Restore the “create” UI
        map_group->setEnabled(true);
        actors->show();                  // the “# of agents” line-edit
        n_agents_label_->show();
        actor_button_->setText("Generate agents");
        actor_button_->setEnabled(false);
        simulator_combo_->setCurrentIndex(-1);

        // Hide all the EDIT_MODE widgets
        edit_goals_button_->hide();
        reset_goals_button_->hide();
        yaml_file_label_->hide();

        // Put map/group boxes back to the CREATE titles & states
        map_group->setTitle("Select simulator and map:");
        map_select_btn_->show();
        map_select_btn_->setVisible(true);
        current_map_label_->show();
        goal_group_->setTitle("Define agents goals");
        goal_group_->setEnabled(false);
        reset_button_->setEnabled(true); });

    // Enable actor_button_ only if:
    //  • “actors” field is a positive integer,
    //  • a simulator is selected,
    //  • and a map has been loaded (or we’re in CREATE_MODE and will pick a map later).
    connect(actors, &QLineEdit::textChanged, this,
            [this](const QString &txt)
            {
              bool ok;
              int v = txt.toInt(&ok);
              bool haveSim = (simulator_combo_->currentIndex() >= 0);
              bool haveMap = !map_file_.isEmpty();
              actor_button_->setEnabled(ok && v > 0 && haveSim && haveMap);
            });

    // When “Create agents” / “Edit agents” is clicked, delegate to onCreateOrEditAgents():
    connect(actor_button_, &QPushButton::clicked,
            this, &ActorPanel::onCreateOrEditAgents);

    connect(open_button_, &QPushButton::clicked, this, [this]()
            {
              // 1) Ask which simulator we’re editing for
              bool ok = false;
              QStringList sims = { "Gazebo", "Isaac Sim", "Webots" };
              QString sim = QInputDialog::getItem(
                  this,
                  tr("Select Simulator"),
                  tr("Which simulator are you using?"),
                  sims,
                  simulator_combo_->currentIndex(),  // start from whatever’s currently shown
                  false,                        
                  &ok);

              if (!ok) {
                // user cancelled the dialog
                return;
              }

              // 2) Store it in sim combo so everything else sees the right simulator
              simulator_combo_->setCurrentText(sim);

              // 3) Now switch into Edit mode and parse the YAML
              open_button_->setDown(true);
              create_button_->setDown(false);
              open_button_->setChecked(true);
              create_button_->setChecked(false);
              parseYaml(); });

    // ─── Goal‐Picking Group (initially disabled) ───
    goal_group_ = new QGroupBox("Define agents goals");
    goal_group_->setEnabled(false);

    enter_goal_mode_btn_ = new QPushButton("Enter Goal-Picking Mode");
    enter_goal_mode_btn_->setCheckable(true);
    connect(enter_goal_mode_btn_, &QPushButton::clicked,
            this, &ActorPanel::onEnterGoalPickingMode);

    // connect(enter_goal_mode_btn_, &QPushButton::clicked, this, [this]()
    //   {

    //   })

    goal_list_widget_ = new QListWidget;

    // ─── “Reset goals” button (only meaningful in EDIT_MODE) ───
    reset_goals_button_ = new QPushButton("Reset Goals", this);
    reset_goals_button_->setEnabled(false);
    reset_goals_button_->hide(); // initially hidden until we enter EDIT_MODE
    connect(reset_goals_button_, &QPushButton::clicked,
            this, &ActorPanel::onResetLoadedGoals);

    assign_goals_btn_ = new QPushButton("Assign goals to agents");
    assign_goals_btn_->setEnabled(false);
    assign_goals_btn_->setCheckable(true);
    connect(assign_goals_btn_, &QPushButton::clicked,
            this, &ActorPanel::onAssignGoalsClicked);

    save_bt_btn_ = new QPushButton(panel_mode_ == EDIT_MODE ? "Save updated YAML/Regenerate BTs" : "Save agents YAML/Generate BTs");
    save_bt_btn_->setEnabled(false);
    reset_button_ = new QPushButton("Reset", this);
    reset_button_->setToolTip(tr("Clear everything and go back to the initial panel state"));

    summary_area_ = new QVBoxLayout;

    auto *goal_layout = new QVBoxLayout;
    goal_layout->addWidget(enter_goal_mode_btn_);
    goal_layout->addWidget(reset_goals_button_);
    goal_layout->addWidget(goal_list_widget_);
    goal_layout->addWidget(assign_goals_btn_);
    goal_layout->addLayout(summary_area_);
    goal_group_->setLayout(goal_layout);

    topic_button->addWidget(goal_group_);

    topic_button->addWidget(checkbox);

    topic_button->addWidget(save_bt_btn_);
    topic_button->addWidget(reset_button_);
    connect(reset_button_, &QPushButton::clicked, this, &ActorPanel::resetPanel);
    // ─── Behavior‐Tree Group ───
    bt_group_ = new QGroupBox("Configure Behavior Trees");
    bt_group_->setEnabled(true);

    edit_bt_btn_ = new QPushButton("Edit in Groot2");
    edit_bt_btn_->setEnabled(true);
    connect(edit_bt_btn_, &QPushButton::clicked,
            this, &ActorPanel::onEditAllInGroot);

    auto *bt_layout = new QVBoxLayout;
    bt_layout->addWidget(edit_bt_btn_);
    bt_layout->addStretch();
    connect(save_bt_btn_, &QPushButton::clicked,
            this, &ActorPanel::saveAndGenerateAll);
    bt_group_->setLayout(bt_layout);

    topic_button->addWidget(bt_group_);

    // ─── Assemble and publish the panel ───
    layout->addLayout(topic_button);
    setLayout(layout);

    // // Create the “agent” publisher (for initial‐pose markers, etc.)
    // initial_pose_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    //     "hunav_agent", rclcpp::QoS(1).transient_local());

    // map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    //     "/map",
    //     // transient_local == latched, reliable == keep it consistent
    //     rclcpp::QoS(rclcpp::KeepLast(1))
    //         .transient_local()
    //         .reliable());
  }

  /**
   * @brief Initialize the panel after RViz context is available
   *
   * Sets up ROS2 subscriptions and publishers that require access to the
   * RViz display context. This is called after the panel is added to RViz.
   */
  void ActorPanel::onInitialize()
  {
    // 1) grab RViz's DisplayContext → RosNodeAbstraction → raw_node
    auto dc = getDisplayContext();
    if (!dc)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ActorPanel"),
                   "DisplayContext is null; cannot subscribe to /clicked_point");
      return;
    }
    auto node_abstraction = dc->getRosNodeAbstraction().lock();
    if (!node_abstraction)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ActorPanel"),
                   "RosNodeAbstraction could not lock; subscription aborted");
      return;
    }
    auto raw_node = node_abstraction->get_raw_node();

    // 2) now create the PointStamped subscription
    goal_sub_ = raw_node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        rclcpp::QoS(10),
        std::bind(&ActorPanel::onGoalPicked, this, std::placeholders::_1));

    // 3) create the publisher for goal markers
    goal_markers_pub_ = raw_node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "hunav_goals", rclcpp::QoS(10));

    // 4) create the publisher for initial pose markers
    initial_pose_publisher = raw_node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "hunav_agent", rclcpp::QoS(1).transient_local());

    // 5) create the publisher for the map
    map_pub_ = raw_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  }

  // Destructor, close and disconnect windows.
  ActorPanel::~ActorPanel()
  {
    window->close();
    window1->close();
    window2->close();
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double, double, double, QString)), this,
               SLOT(onInitialPose(double, double, double, QString)));
    disconnect(&GoalUpdater, SIGNAL(updateGoal(double, double, double, QString)), this,
               SLOT(onNewGoal(double, double, double, QString)));
  }

  void ActorPanel::onEditAllInGroot()
  {
    // — 1) If we have a loaded YAML, rebuild every marker
    if (!loaded_agent_nodes_.empty())
    {
      // clear any existing markers
      removeCurrentMarkers();

      // (a) draw global goals
      for (auto const &it : loaded_global_goals_)
      {
        int gid = it.first;
        auto pt = it.second;

        // sphere
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "/map";
        sphere.header.stamp = rclcpp::Clock().now();
        sphere.ns = "goal_points";
        sphere.id = gid * 2;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position = pt;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2;
        sphere.color.r = 0.0f;
        sphere.color.g = 0.7f;
        sphere.color.b = 0.7f;
        sphere.color.a = 1.0f;
        goal_markers_.markers.push_back(sphere);

        // label
        visualization_msgs::msg::Marker label = sphere;
        label.ns = "goal_numbers";
        label.id = gid * 2 + 1;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.pose.position.y += 0.7;
        label.pose.position.z = 1.5;
        label.scale.z = 0.7;
        label.text = std::to_string(gid);
        label.color.r = label.color.g = label.color.b = 1.0f;
        label.color.a = 1.0f;
        goal_markers_.markers.push_back(label);
      }

      // (b) draw each agent’s initial pose, ID, its goals & arrows
      const size_t N = loaded_agent_nodes_.size();
      int marker_id = 0;
      for (size_t a = 0; a < N; ++a)
      {
        auto &node = loaded_agent_nodes_[a];
        double ipx = node["init_pose"]["x"].as<double>();
        double ipy = node["init_pose"]["y"].as<double>();
        double yaw = node["init_pose"]["h"] ? node["init_pose"]["h"].as<double>() : 0.0;

        // agent mesh marker
        auto agent_m = createMarker(ipx, ipy, marker_id++, "person", "parser");
        agent_m.ns = "agent_initial";
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        agent_m.pose.orientation = tf2::toMsg(q);
        goal_markers_.markers.push_back(agent_m);

        // agent ID text
        QColor col;
        col.setHsvF(double(a) / N, 0.8, 0.9);
        visualization_msgs::msg::Marker id_txt;
        id_txt.header = agent_m.header;
        id_txt.ns = "agent_id_text";
        id_txt.id = marker_id++;
        id_txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        id_txt.action = visualization_msgs::msg::Marker::ADD;
        id_txt.pose.position.x = ipx;
        id_txt.pose.position.y = ipy + 0.7;
        id_txt.pose.position.z = 0.6;
        id_txt.scale.z = 0.8;
        id_txt.color.r = col.redF();
        id_txt.color.g = col.greenF();
        id_txt.color.b = col.blueF();
        id_txt.color.a = 1.0f;
        id_txt.text = std::to_string(int(a + 1));
        goal_markers_.markers.push_back(id_txt);

        // draw this agent’s goals + connecting arrows
        geometry_msgs::msg::Point prev_pt;
        prev_pt.x = ipx;
        prev_pt.y = ipy;
        prev_pt.z = 0.0;
        for (auto const &gid_node : node["goals"])
        {
          int gid = gid_node.as<int>();
          auto it = loaded_global_goals_.find(gid);
          if (it == loaded_global_goals_.end())
            continue;
          auto gp = it->second;

          // small cube
          auto cube = createMarker(gp.x, gp.y, marker_id++, "cube", "parser");
          cube.ns = "agent_goal";
          cube.color.r = col.redF();
          cube.color.g = col.greenF();
          cube.color.b = col.blueF();
          cube.color.a = 1.0f;
          goal_markers_.markers.push_back(cube);

          // arrow
          auto arrow = createArrowMarker(prev_pt.x, prev_pt.y, gp.x, gp.y, marker_id++);
          arrow.ns = "agent_arrow";
          arrow.color.r = col.redF();
          arrow.color.g = col.greenF();
          arrow.color.b = col.blueF();
          arrow.color.a = 1.0f;
          goal_markers_.markers.push_back(arrow);

          prev_pt.x = gp.x;
          prev_pt.y = gp.y;
        }

        // closing arrow back to start
        if (node["goals"].size() > 0)
        {
          int first_gid = node["goals"][0].as<int>();
          auto it_first = loaded_global_goals_.find(first_gid);
          if (it_first != loaded_global_goals_.end())
          {
            auto first_pt = it_first->second;
            auto closing = createArrowMarker(
                prev_pt.x, prev_pt.y,
                first_pt.x, first_pt.y,
                marker_id++);
            closing.ns = "agent_arrow";
            closing.color.r = col.redF();
            closing.color.g = col.greenF();
            closing.color.b = col.blueF();
            closing.color.a = 1.0f;
            goal_markers_.markers.push_back(closing);
          }
        }
      }

      // publish them all at once
      goal_markers_pub_->publish(goal_markers_);
    }

    // — 2) now pop up the directory‐info & launch Groot as before —
    QString btDir;
    QString simulatorName = simulator_combo_->currentText();
    if (simulatorName == "Gazebo")
    {
      QString shareDir;
      try
      {
        shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory("hunav_gazebo_wrapper"));
      }
      catch (const std::exception &e)
      {
        // fallback to home‐installed wrapper if package not found
        QString homePath = QDir::homePath() + "/hunav_gazebo_wrapper";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_gazebo_wrapper";
        shareDir = QDir(dockerPath).exists() ? dockerPath : homePath;
      }
      btDir = shareDir + "/behavior_trees";
    }
    else if (simulatorName == "Isaac Sim")
    {
      QString base = QDir("/workspace/hunav_isaac_ws/src/Hunav_isaac_wrapper").exists()
                         ? "/workspace/hunav_isaac_ws/src/Hunav_isaac_wrapper"
                         : QDir::homePath() + "/Hunav_isaac_wrapper";
      btDir = base + "/behavior_trees";
    }
    else
    {
      QString base = QDir("/workspace/hunav_isaac_ws/src/hunav_webots_wrapper").exists()
                         ? "/workspace/hunav_isaac_ws/src/hunav_webots_wrapper"
                         : QDir::homePath() + "/hunav_webots_wrapper";
      btDir = base + "/behavior_trees";
    }

    QMessageBox::information(
        this,
        "Behavior Trees Location",
        QString("<html>All generated BehaviorTree XML files are located in:<br><br>"
                "%1<br><br><b>Groot will now launch.</b></html>")
            .arg(btDir));

    QString grootExe = QDir::home().filePath("Groot2/bin/groot2");
    if (!QProcess::startDetached(grootExe, QStringList{}))
    {
      QMessageBox::warning(this, "Launch Error",
                           QString("Failed to launch Groot2 at:\n%1\nPlease check that path.")
                               .arg(grootExe));
    }
  }

  // Utility: load file to string
  static QString loadFile(const QString &path)
  {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
      return {};
    return f.readAll();
  }

  void ActorPanel::addAgent()
  {
    // ─────────────────────────── DETERMINE TOTAL AGENTS ────────────────────────
    // In CREATE_MODE, we read from the "actors" QLineEdit:
    if (panel_mode_ == CREATE_MODE)
    {
      num_agents = actors->text().toInt();
    }
    else if (panel_mode_ == EDIT_MODE) // In EDIT_MODE, override num_agents to be the number of loaded_agent_nodes_:
    {
      num_agents = int(loaded_agent_nodes_.size());
    }
    initAgentColors(num_agents);

    // ─────────────────────── POPUP WINDOW SETUP ─────────────────────────────────
    // We want one window, reused across calls:
    if (!window)
    {
      window = new QWidget;
      QPoint center_left = this->mapToGlobal(QPoint(60, 0));
      window->move(center_left);
    }
    window->setWindowFlag(Qt::WindowStaysOnTopHint);

    // Title:
    window->setWindowTitle(panel_mode_ == EDIT_MODE ? QString("Edit Agent") : QString("Add Agent"));

    // If the layout already existed, clear it out entirely:
    if (topic_layout)
    {
      QLayoutItem *child;
      while ((child = topic_layout->takeAt(0)) != nullptr)
      {
        if (auto w = child->widget())
          delete w;
        delete child;
      }
    }
    else
    {
      topic_layout = new QVBoxLayout;
      window->setLayout(topic_layout);
    }

    // ────────────────────────────── FIELDS IN POPUP ──────────────────────────────

    // (1) Show which agent number this is:
    QString text;
    if (panel_mode_ == CREATE_MODE)
    {
      text = QString("Agent %1 / %2").arg(agent_count).arg(num_agents);
    }
    else
    {
      text = QString("Editing Agent %1 / %2")
                 .arg(current_edit_idx_ + 1)
                 .arg(num_agents);
    }

    auto *header = new QLabel(text, window);
    header->setStyleSheet("font-style: bold;");
    header->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    topic_layout->addWidget(header);

    // ─── NAVIGATION BUTTONS (EDIT MODE ONLY) ───
    if (panel_mode_ == EDIT_MODE)
    {
      // a little row to select agent to edit
      auto *nav = new QHBoxLayout;
      auto *prev = new QPushButton(tr("◀"), window);
      auto *next = new QPushButton(tr("▶"), window);
      nav->addStretch();
      nav->addWidget(prev);
      nav->addWidget(next);
      nav->addStretch();
      topic_layout->addLayout(nav);

      // disable at the ends
      prev->setEnabled(current_edit_idx_ > 0);
      next->setEnabled(current_edit_idx_ + 1 < num_agents);

      connect(prev, &QPushButton::clicked, this, [this]()
              {
          if (current_edit_idx_ > 0)
          {
            current_edit_idx_--;
            window->close();
            addAgent();
          } });
      connect(next, &QPushButton::clicked, this, [this]()
              {
          if (current_edit_idx_ + 1 < num_agents)
          {
            current_edit_idx_++;
            window->close();
            addAgent();
          } });
    }
    // ─────────────────────────── AGENT CONFIGURATION FIELDS ─────────────────────

    // (2) Desired velocity:
    topic_layout->addWidget(new QLabel("Desired vel [m/s]:"));
    agent_desired_vel = new QLineEdit(window);
    agent_desired_vel->setText(QString::number(1.5, 'f', 1)); // default
    topic_layout->addWidget(agent_desired_vel);

    // (3) Behavior type selection
    topic_layout->addWidget(new QLabel("Behavior type:"));
    behavior_type_combobox = new QComboBox(window);
    behavior_type_combobox->addItem("Regular");
    behavior_type_combobox->addItem("Impassive");
    behavior_type_combobox->addItem("Surprised");
    behavior_type_combobox->addItem("Scared");
    behavior_type_combobox->addItem("Curious");
    behavior_type_combobox->addItem("Threatening");
    topic_layout->addWidget(behavior_type_combobox);
    connect(behavior_type_combobox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ActorPanel::checkComboBoxConf);

    // (4) Behavior configuration:
    topic_layout->addWidget(new QLabel("Behavior configuration:"));
    behavior_conf_combobox = new QComboBox(window);
    behavior_conf_combobox->addItem("Default");
    behavior_conf_combobox->addItem("Custom");
    behavior_conf_combobox->addItem("Random-normal distribution");
    behavior_conf_combobox->addItem("Random-uniform distribution");
    behavior_conf_combobox->setCurrentIndex(0);
    topic_layout->addWidget(behavior_conf_combobox);

    // (5) Duration / Only once / Dist / Vel controls:
    dur = new QLabel("Behavior duration:", window);
    dur->setVisible(false);
    beh_duration = new QLineEdit(window);
    beh_duration->setVisible(false);
    once = new QLabel("Behavior only once:", window);
    once->setVisible(false);
    beh_once = new QLineEdit(window);
    beh_once->setVisible(false);
    dist = new QLabel("Behavior visibility dist:", window);
    dist->setVisible(false);
    beh_dist = new QLineEdit(window);
    beh_dist->setVisible(false);
    vel = new QLabel("Behavior agent vel:", window);
    vel->setVisible(false);
    beh_vel = new QLineEdit(window);
    beh_vel->setVisible(false);
    other = new QLabel("Front dist (Threat):", window);
    other->setVisible(false);
    beh_otherff = new QLineEdit(window);
    beh_otherff->setVisible(false);

    topic_layout->addWidget(dur);
    topic_layout->addWidget(beh_duration);
    topic_layout->addWidget(once);
    topic_layout->addWidget(beh_once);
    topic_layout->addWidget(dist);
    topic_layout->addWidget(beh_dist);
    topic_layout->addWidget(vel);
    topic_layout->addWidget(beh_vel);
    topic_layout->addWidget(other);
    topic_layout->addWidget(beh_otherff);

    // (6) Skin (only visible if Gazebo is selected):
    skin_label_ = new QLabel("Skin:", window);
    skin_label_->setVisible(false);
    topic_layout->addWidget(skin_label_);

    skin_combobox = new QComboBox(window);
    skin_combobox->addItem("Elegant man");
    skin_combobox->addItem("Casual man");
    skin_combobox->addItem("Elegant woman");
    skin_combobox->addItem("Regular man");
    skin_combobox->addItem("Worker man");
    skin_combobox->addItem("Blue jeans");
    skin_combobox->addItem("Green t-shirt");
    skin_combobox->addItem("Blue t-shirt");
    skin_combobox->addItem("Red t-shirt");
    skin_combobox->setVisible(false);
    topic_layout->addWidget(skin_combobox);

    // Whenever “Simulator” changes, show/hide Skin:
    connect(simulator_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int)
            {
              bool isGazebo = (simulator_combo_->currentText() == "Gazebo");
              skin_label_->setVisible(isGazebo);
              skin_combobox->setVisible(isGazebo);
            });

    // (7) “Set initial pose” button:
    initial_pose_button = new QPushButton("Set initial pose", window);
    if (panel_mode_ == EDIT_MODE)
      initial_pose_button->setText("Edit initial pose");

    initial_pose_button->setCheckable(true);
    initial_pose_button->setDown(false);
    connect(initial_pose_button, &QPushButton::clicked,
            this, &ActorPanel::setInitialPose);

    // (8) Hidden “GFF / OFF / SFF / …” fields:
    vel = new QLabel("Behavior agent vel:", window);
    vel->setVisible(false);
    topic_layout->addWidget(vel);

    beh_vel = new QLineEdit(window);
    beh_vel->setText("1.0");
    beh_vel->setVisible(false);
    beh_vel->setEnabled(false);
    topic_layout->addWidget(beh_vel);

    gff = new QLabel("Beh Goal Force Factor:", window);
    topic_layout->addWidget(gff);
    beh_gff = new QLineEdit(window);
    beh_gff->setText(QString::number(2.0, 'f', 1));
    beh_gff->setEnabled(false);
    topic_layout->addWidget(beh_gff);

    off = new QLabel("Beh Obstacle Force Factor:", window);
    topic_layout->addWidget(off);
    beh_off = new QLineEdit(window);
    beh_off->setText(QString::number(10.0, 'f', 1));
    beh_off->setEnabled(false);
    topic_layout->addWidget(beh_off);

    sff = new QLabel("Beh Social Force Factor:", window);
    topic_layout->addWidget(sff);
    beh_sff = new QLineEdit(window);
    beh_sff->setText(QString::number(5.0, 'f', 1));
    beh_sff->setEnabled(false);
    topic_layout->addWidget(beh_sff);

    other = new QLabel("Beh Robot Repulsive Force Factor:", window);
    other->setVisible(false);
    topic_layout->addWidget(other);
    beh_otherff = new QLineEdit(window);
    beh_otherff->setText(QString::number(20.0, 'f', 1));
    beh_otherff->setEnabled(false);
    beh_otherff->setVisible(false);
    topic_layout->addWidget(beh_otherff);

    connect(behavior_conf_combobox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &ActorPanel::checkComboBoxConf);

    // (8) “Next / Save & Next” button:
    save_button_ = new QPushButton("Next agent", window);
    save_button_->setEnabled(false);
    topic_layout->addWidget(initial_pose_button);
    topic_layout->addWidget(save_button_);

    connect(save_button_, &QPushButton::clicked, [this]()
            {
    // ─────────────────────────── SAVE CURRENT FIELDS ─────────────────────────
    YAML::Node new_node;

    // (a) id (1-based) and group_id
    int agent_id = (panel_mode_ == EDIT_MODE)
      ? (current_edit_idx_ + 1)
      : agent_count; 
    new_node["id"] = agent_id;
    new_node["group_id"] = -1;

    // (b) skin (if Gazebo)
    if (simulator_combo_->currentText() == "Gazebo")
    {
      new_node["skin"] = skin_combobox->currentIndex();
    }

    // (c) max_vel
    new_node["max_vel"] = agent_desired_vel->text().toDouble();

    // (d) radius, goal_radius, cyclic_goals
    new_node["radius"] = "0.4";
    new_node["goal_radius"] = "0.3";
    new_node["cyclic_goals"] = true;

    // (e) initial pose (x,y,z) must have been set
    if (!initial_pose_set)
    {
      QMessageBox::warning(window,
        tr("Missing Initial Pose"),
        tr("<html>Please click on <i><b>Set initial pose</b></i> before saving.</html>"));
      return;
    }
    new_node["init_pose"]["x"] = QString::number(stored_pose.pose.position.x, 'f', 3).toStdString();
    new_node["init_pose"]["y"] = QString::number(stored_pose.pose.position.y, 'f', 3).toStdString();
    new_node["init_pose"]["z"] = QString::number(stored_pose.pose.position.z, 'f', 3).toStdString();
    tf2::Quaternion q;
    tf2::fromMsg(stored_pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    new_node["init_pose"]["h"] = QString::number(yaw, 'f', 3).toStdString();

    // (f) behavior type
    new_node["behavior"]["type"] = behavior_type_combobox->currentText().toStdString();

    // (g) behavior configuration
    int beh_enum = checkComboBox();
    switch (beh_enum) {
      case hunav_msgs::msg::AgentBehavior::BEH_REGULAR:
      case hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE:
        break;

      case hunav_msgs::msg::AgentBehavior::BEH_SURPRISED:
        new_node["behavior"]["dist"]   = QString::number(beh_dist->text().toDouble(), 'f', 3).toStdString();
        new_node["behavior"]["duration"]              = QString::number(beh_duration->text().toDouble(), 'f', 1).toStdString();
        new_node["behavior"]["once"]             = (beh_once->text().toLower() == "true");
        break;

      case hunav_msgs::msg::AgentBehavior::BEH_SCARED:
        new_node["behavior"]["dist"]   = QString::number(beh_dist->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["duration"]              = QString::number(beh_duration->text().toDouble(), 'f', 1).toStdString();
        new_node["behavior"]["once"]             = (beh_once->text().toLower() == "true");
        new_node["behavior"]["vel"]           = QString::number(beh_vel->text().toDouble(), 'f', 3).toStdString();
        break;

      case hunav_msgs::msg::AgentBehavior::BEH_CURIOUS:
        new_node["behavior"]["dist"]   = QString::number(beh_dist->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["duration"]              = QString::number(beh_duration->text().toDouble(), 'f', 1).toStdString();
        new_node["behavior"]["once"]             = (beh_once->text().toLower() == "true");
        new_node["behavior"]["vel"]             = QString::number(beh_vel->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["dist"]         = QString::number(beh_dist->text().toDouble(), 'f', 2).toStdString();
        break;

      case hunav_msgs::msg::AgentBehavior::BEH_THREATENING:
        new_node["behavior"]["dist"]   = QString::number(beh_dist->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["duration"]              = QString::number(beh_duration->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["once"]             = (beh_once->text().toLower() == "true");
        new_node["behavior"]["other_force_factor"]            = QString::number(beh_otherff->text().toDouble(), 'f', 1).toStdString();
        break;
    }
    QString txt = behavior_conf_combobox->currentText();
    int conf = hunav_msgs::msg::AgentBehavior::BEH_CONF_DEFAULT;
    if      (txt == "Custom")                 conf = hunav_msgs::msg::AgentBehavior::BEH_CONF_CUSTOM;
    else if (txt == "Random-normal distribution")
                                                conf = hunav_msgs::msg::AgentBehavior::BEH_CONF_RANDOM_NORMAL;
    else if (txt == "Random-uniform distribution")
                                                conf = hunav_msgs::msg::AgentBehavior::BEH_CONF_RANDOM_UNIFORM;
    new_node["behavior"]["configuration"] = conf;

    // (h) copy GFF / OFF / SFF from text fields
    new_node["behavior"]["goal_force_factor"]      = QString::number(beh_gff->text().toDouble(), 'f', 1).toStdString();
    new_node["behavior"]["obstacle_force_factor"] = QString::number(beh_off->text().toDouble(), 'f', 1).toStdString();
    new_node["behavior"]["social_force_factor"]   = QString::number(beh_sff->text().toDouble(), 'f', 1).toStdString();
    new_node["behavior"]["other_force_factor"]    = QString::number(beh_otherff->text().toDouble(), 'f', 1).toStdString();

    // ────────────────────── WRITE‐BACK & ADVANCE INDEX ────────────────────────
    if (panel_mode_ == EDIT_MODE)
    {  
      // Overwrite the existing node:
      loaded_agent_nodes_[current_edit_idx_] = new_node;
    }
    else
    {
      // In CREATE_MODE, append to actors_info then move on:
      loaded_agent_nodes_.push_back(new_node);
    }

    // If we are in CREATE_MODE, bump agent_count → possibly spawn next popup:
    if (panel_mode_ == CREATE_MODE)
    {
      agent_count++;
      if (agent_count <= num_agents)
      {
        // Close current popup and show next
        window->close();
        addAgent();
        return;
      }
    }
    else
    {
      // EDIT_MODE: advance current_edit_idx_
      if (current_edit_idx_ + 1 < num_agents)
      {
        current_edit_idx_++;
        window->close();
        addAgent();
        return;
      }
    }

    // ─────────────────────── ALL AGENTS DONE ────────────────────────────────
    window->close();
    actor_button_->setDown(false);
    QMessageBox::information(this,
                             "All agents ready",
                             QString(
                                "<html>"
                                "All agents have been %1.<br><br>"
                                "You may now %2 navigation goals by <br>clicking on <i><b>%3</b></i>."
                                "</html>"
                              )
                               .arg((panel_mode_ == EDIT_MODE) ? "edited" : "created")
                               .arg((panel_mode_ == EDIT_MODE) ? "edit/add" : "pick/assign")
                               .arg((panel_mode_ == EDIT_MODE) ? edit_goals_button_->text() : enter_goal_mode_btn_->text())); });

    // ────────────────────────────────────────────────────────────────────────────

    // (8) Prefill fields if in EDIT_MODE:
    if (panel_mode_ == EDIT_MODE &&
        current_edit_idx_ >= 0 &&
        current_edit_idx_ < static_cast<int>(loaded_agent_nodes_.size()))
    {
      const YAML::Node &agentYAML = loaded_agent_nodes_[current_edit_idx_];

      // — Desired velocity —
      if (agentYAML["max_vel"])
      {
        double dv = agentYAML["max_vel"].as<double>();
        agent_desired_vel->setText(QString::number(dv, 'f', 1));
      }

      // — Behavior type —
      if (agentYAML["behavior"] && agentYAML["behavior"]["type"])
      {
        QString type = QString::fromStdString(
            agentYAML["behavior"]["type"].as<std::string>());
        behavior_type_combobox->setCurrentText(type);
      }

      // — Behavior configuration —
      int conf = hunav_msgs::msg::AgentBehavior::BEH_CONF_DEFAULT;
      if (agentYAML["behavior"] && agentYAML["behavior"]["configuration"])
        conf = agentYAML["behavior"]["configuration"].as<int>();
      switch (conf)
      {
      case hunav_msgs::msg::AgentBehavior::BEH_CONF_CUSTOM:
        behavior_conf_combobox->setCurrentText("Custom");
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_CONF_RANDOM_NORMAL:
        behavior_conf_combobox->setCurrentText("Random-normal distribution");
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_CONF_RANDOM_UNIFORM:
        behavior_conf_combobox->setCurrentText("Random-uniform distribution");
        break;
      default:
        behavior_conf_combobox->setCurrentText("Default");
        break;
      }

      checkComboBoxConf();

      // — Skin (Gazebo only) —
      bool isGazebo = (simulator_combo_->currentText() == "Gazebo");
      if (agentYAML["skin"] && isGazebo)
      {
        int raw_skin = agentYAML["skin"].as<int>();
        skin_label_->setVisible(true);
        skin_combobox->setVisible(true);
        skin_combobox->setCurrentIndex(raw_skin);
      }

      // — Initial pose —
      if (agentYAML["init_pose"])
      {
        double ipx = agentYAML["init_pose"]["x"].as<double>();
        double ipy = agentYAML["init_pose"]["y"].as<double>();
        double ipz = agentYAML["init_pose"]["z"].as<double>();

        stored_pose.pose.position.x = ipx;
        stored_pose.pose.position.y = ipy;
        stored_pose.pose.position.z = ipz;
        initial_pose_set = true;

        // Enable “Save & Next” since we have a stored pose:
        save_button_->setEnabled(true);
      }
      else
      {
        initial_pose_set = false;
        save_button_->setEnabled(false);
      }
    }
    else
    {
      // ─────────────────── “CREATE_MODE” DEFAULTS ────────────────────────────
      agent_desired_vel->setText(QString::number(1.5, 'f', 1));
      behavior_type_combobox->setCurrentIndex(0);
      behavior_conf_combobox->setCurrentIndex(0);
      initial_pose_set = false;
      save_button_->setEnabled(false);

      skin_label_->setVisible(false);
      skin_combobox->setVisible(false);
    }

    bool last = false;
    if (panel_mode_ == CREATE_MODE)
    {
      last = (agent_count >= num_agents);
    }
    else
    {
      last = (current_edit_idx_ + 1 >= num_agents);
    }
    save_button_->setText(last ? tr("Finish") : tr("Next agent"));

    window->raise();
    window->adjustSize();
    window->show();
  }

  void ActorPanel::setInitialPose()
  {
    // On very first click, pop a one‐time tip:
    if (!initial_pose_tip_shown_)
    {
      initial_pose_tip_shown_ = true;
      QMessageBox::information(
          this, "Initial Pose Tip",
          "<b>Click and drag on the map</b> to set the agent’s initial position and orientation.");
    }

    // hook up the Pose tool so onInitialPose will fire
    if (initial_pose_connection_)
      disconnect(&GoalUpdater, 0, this, 0);
    initial_pose_connection_ = new QObject(this);
    connect(&GoalUpdater,
            SIGNAL(updateGoal(double, double, double, QString)),
            this,
            SLOT(onInitialPose(double, double, double, QString)));

    // switch RViz into HunavGoals tool
    if (auto *tm = getDisplayContext()->getToolManager())
    {
      for (int i = 0; i < tm->numTools(); ++i)
      {
        if (QString(tm->getTool(i)->getName()) == "HunavGoals")
        {
          tm->setCurrentTool(tm->getTool(i));
          break;
        }
      }
    }
  }

  void ActorPanel::onInitialPose(double x, double y, double theta, QString frame)
  {

    // figure out which agent this is
    int idx = (panel_mode_ == EDIT_MODE)
                  ? current_edit_idx_
                  : (agent_count - 1);
    if (idx < 0 || idx >= (int)loaded_initial_marker_ids_.size())
    {
      RCLCPP_ERROR(get_logger(),
                   "onInitialPose(): bogus agent index %d", idx);
      return;
    }
    // 1) If in EDIT_MODE, delete both the old mesh and its label for this agent:
    if (panel_mode_ == EDIT_MODE)
    {
      auto del = std::make_unique<visualization_msgs::msg::MarkerArray>();

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "/map";
      m.header.stamp = rclcpp::Clock().now();
      m.id = loaded_initial_marker_ids_[idx];
      m.action = visualization_msgs::msg::Marker::DELETE;

      // delete the mesh
      m.ns = "agent_initial";
      del->markers.push_back(m);

      // delete the floating text
      m.ns = "agent_id_text";
      del->markers.push_back(m);

      initial_pose_publisher->publish(std::move(del));
    }

    // 2) Read Z offset for the selected simulator
    double z_offset = simulator_combo_->currentData().toDouble();

    // 3) Allocate fresh ID
    int id = next_marker_id_++;
    loaded_initial_marker_ids_[idx] = id;

    // pick this agent’s unique color
    const QColor &c = agent_colors_[idx];
    std_msgs::msg::ColorRGBA agent_col;
    agent_col.r = c.redF();
    agent_col.g = c.greenF();
    agent_col.b = c.blueF();
    agent_col.a = 1.0f;

    // 4) Build the mesh marker
    visualization_msgs::msg::Marker mesh = createMarker(x, y, id, "person", "create");
    mesh.ns = "agent_initial";
    mesh.id = id;
    mesh.pose.position.z = z_offset;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    mesh.pose.orientation = tf2::toMsg(q);
    mesh.color = agent_col;

    // 5) Build the floating text label
    visualization_msgs::msg::Marker label =
        createAgentLabel(x, y, id, "/map");
    label.ns = "agent_id_text";
    label.id = id;
    label.pose.position.z = z_offset + 1.5; // float above the agent
    label.text = std::to_string(idx + 1);
    label.color = agent_col;

    // 6) Publish them together
    auto out = std::make_unique<visualization_msgs::msg::MarkerArray>();
    out->markers.push_back(mesh);
    out->markers.push_back(label);
    initial_pose_publisher->publish(std::move(out));

    // 7) Store the new pose + orientation
    initial_pose_set = true;
    stored_pose.header.frame_id = frame.toStdString();
    stored_pose.header.stamp = rclcpp::Clock().now();
    stored_pose.pose.position.x = x;
    stored_pose.pose.position.y = y;
    stored_pose.pose.position.z = z_offset;
    stored_pose.pose.orientation.x = q.x();
    stored_pose.pose.orientation.y = q.y();
    stored_pose.pose.orientation.z = q.z();
    stored_pose.pose.orientation.w = q.w();

    // 8) Tear down goal-updater connection
    disconnect(&GoalUpdater,
               SIGNAL(updateGoal(double, double, double, QString)),
               this,
               SLOT(onInitialPose(double, double, double, QString)));

    // 9) Re-enable panel’s controls so the user can press Next/Finish
    if (save_button_)
      save_button_->setEnabled(true);

    if (panel_mode_ == CREATE_MODE && idx == num_agents - 1)
      goal_group_->setEnabled(true);

    window->raise();
    window->activateWindow();
    window->show();
  }

  void ActorPanel::onSelectMap()
  {
    removeCurrentMarkers();
    // 1) ask user for a .yaml file in the right directory
    QString baseDir;
    QString shareDir;

    if (simulator_combo_->currentText() == "Gazebo")
    {
      try
      {
        shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory("hunav_gazebo_wrapper"));
      }
      catch (const std::exception &e)
      {
        // fallback to home‐installed wrapper if package not found
        QString homePath = QDir::homePath() + "/hunav_gazebo_wrapper";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_gazebo_wrapper";
        shareDir = QDir(dockerPath).exists() ? dockerPath : homePath;
      }
      baseDir = shareDir + "/maps";
    }
    else if (simulator_combo_->currentText() == "Isaac Sim")
    {
      QString homePath = QDir::homePath() + "/Hunav_isaac_wrapper/maps";
      QString dockerPath = "/workspace/hunav_isaac_ws/src/Hunav_isaac_wrapper/maps";
      baseDir = QDir(dockerPath).exists() ? dockerPath : homePath;
    }
    else
    {
      QString homePath = QDir::homePath() + "/hunav_webots_wrapper/maps";
      QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_webots_wrapper/maps";
      baseDir = QDir(dockerPath).exists() ? dockerPath : homePath;
    }

    QString yaml = QFileDialog::getOpenFileName(
        this,
        "Select map YAML",
        baseDir,
        "YAML files (*.yaml *.yml)");
    if (yaml.isEmpty())
    {
      return;
    }

    map_file_ = yaml;
    current_map_label_->setText(QFileInfo(yaml).fileName());

    // 2) call the map_server/load_map service
    auto client = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    if (!client->wait_for_service(2s))
    {
      QMessageBox::warning(this, "Map Server",
                           "Timed out waiting for /map_server/load_map\n\n Is map_server up?");
      return;
    }

    auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    req->map_url = yaml.toStdString();

    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) != rclcpp::FutureReturnCode::SUCCESS)
    {
      QMessageBox::critical(this, "Map Server",
                            "Failed to call /map_server/load_map on:\n" + yaml);
      return;
    }
    auto resp = future.get();
    QMessageBox::information(this, "Map Server",
                             "Successfully loaded map:\n" + QFileInfo(yaml).fileName());

    panel_mode_ = CREATE_MODE;
    actors->setEnabled(true);
    n_agents_label_->setEnabled(true);
  }

  /**
   * @brief Handle goal point clicked from RViz map
   *
   * This method is called when the user clicks on the map while in goal-picking mode.
   * It creates a new goal marker at the clicked location, assigns it a unique ID,
   * and adds it to both the visual representation and internal data structures.
   *
   * @param msg Point stamped message containing the clicked coordinates in map frame
   */

  void ActorPanel::onGoalPicked(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!goal_picking_mode_)
    {
      return;
    }

    // 1) If already “holding” a goal, this click is its new drop location:
    if (moving_goal_id_ >= 0)
    {
      int gid = moving_goal_id_;
      moving_goal_id_ = -1; // reset

      // update stored coords
      loaded_global_goals_[gid] = msg->point;

      // add new markers at the drop point
      visualization_msgs::msg::Marker sphere;
      sphere.header = msg->header;
      sphere.ns = "goal_points";
      sphere.id = gid * 2;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position = msg->point;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2;
      sphere.color.r = 0.0f;
      sphere.color.g = 0.7f;
      sphere.color.b = 0.7f;
      sphere.color.a = 1.0f;
      goal_markers_.markers.push_back(sphere);

      visualization_msgs::msg::Marker label = sphere;
      label.ns = "goal_numbers";
      label.id = gid * 2 + 1;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.y += 0.5;
      label.pose.position.z += 0.3;
      label.scale.z = 0.7;
      label.color.r = label.color.g = label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = std::to_string(gid);
      goal_markers_.markers.push_back(label);

      // refresh GUI
      goal_markers_pub_->publish(goal_markers_);
      rebuildGoalListWidget();

      if (auto *tm = getDisplayContext()->getToolManager())
      {
        for (int i = 0; i < tm->numTools(); ++i)
        {
          if (QString(tm->getTool(i)->getClassId()) == "rviz_default_plugins/PublishPoint")
          {
            tm->setCurrentTool(tm->getTool(i));
            break;
          }
        }
      }

      return;
    }

    // 2) Otherwise, see if this click is near an existing goal → pick it up
    int picked_gid = -1;
    double best_d = std::numeric_limits<double>::max();
    for (auto const &it : loaded_global_goals_)
    {
      int gid = it.first;
      auto &pos = it.second;
      double d = std::hypot(msg->point.x - pos.x,
                            msg->point.y - pos.y);
      if (d < 0.2 && d < best_d)
      {
        picked_gid = gid;
        best_d = d;
      }
    }

    if (picked_gid >= 0)
    {
      // pick up that goal:
      moving_goal_id_ = picked_gid;
      loaded_global_goals_.erase(picked_gid);

      // remove its two markers:
      goal_markers_.markers.erase(
          std::remove_if(
              goal_markers_.markers.begin(),
              goal_markers_.markers.end(),
              [&](auto &m)
              {
                return m.id == picked_gid * 2 || m.id == picked_gid * 2 + 1;
              }),
          goal_markers_.markers.end());

      // update GUI so user sees them vanish:
      goal_markers_pub_->publish(goal_markers_);
      rebuildGoalListWidget();

      // optionally nudge user:
      QMessageBox::information(
          this,
          tr("Edit Goal"),
          tr("Now click on the map to place Goal %1 at its new location.").arg(picked_gid));

      if (auto *tm = getDisplayContext()->getToolManager())
      {
        for (int i = 0; i < tm->numTools(); ++i)
        {
          if (QString(tm->getTool(i)->getClassId()) == "rviz_default_plugins/PublishPoint")
          {
            tm->setCurrentTool(tm->getTool(i));
            break;
          }
        }
      }

      return;
    }

    // 3) If clicked nowhere near an existing goal, fall back to “add new”:
    {
      // ─── 1) Store the new goal pose ───
      geometry_msgs::msg::Pose p;
      p.position = msg->point;
      p.orientation.w = 1.0;
      // goals_.push_back(p);

      // ─── 2) Compute a fresh 1-based goal ID ───
      //    (max key in loaded_global_goals_ + 1, or 1 if empty)
      // int newGID = goal_ids_.empty() ? 1 : (goal_ids_.back() + 1);
      int newGID = loaded_global_goals_.empty()
                       ? 1
                       : (loaded_global_goals_.rbegin()->first + 1);
      goal_ids_.push_back(newGID);
      // Determine next available goal ID from existing global goals
      // if (!loaded_global_goals_.empty())
      //   newGID = loaded_global_goals_.rbegin()->first + 1;
      loaded_global_goals_[newGID] = p.position;

      // ─── 3) Create & append the sphere marker ───
      visualization_msgs::msg::Marker sphere;
      sphere.header = msg->header; // reuse frame & stamp
      sphere.ns = "goal_points";
      sphere.id = newGID * 2; // even IDs
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = p;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2;
      sphere.color.r = 0.0f;
      sphere.color.g = 0.7f;
      sphere.color.b = 0.7f;
      sphere.color.a = 1.0f;
      goal_markers_.markers.push_back(sphere);

      // ─── 4) Create & append the text label marker ───
      visualization_msgs::msg::Marker label;
      label.header = msg->header;
      label.ns = "goal_numbers";
      label.id = newGID * 2 + 1; // odd IDs
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose = p;
      label.pose.position.y += 0.5;
      label.pose.position.z += 0.3;
      label.scale.z = 0.7;
      label.color.r = label.color.g = label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = std::to_string(newGID);
      goal_markers_.markers.push_back(label);

      // ─── 5) Publish all goal markers together ───
      goal_markers_pub_->publish(goal_markers_);

      // ─── 6) Update the list widget with the same ID & coords ───
      QString entry = QString("Goal %1: (%2, %3)")
                          .arg(newGID)
                          .arg(p.position.x, 0, 'f', 3)
                          .arg(p.position.y, 0, 'f', 3);
      goal_list_widget_->addItem(entry);

      // rebuildGoalListWidget();
    }

    if (auto *tm = getDisplayContext()->getToolManager())
    {
      for (int i = 0; i < tm->numTools(); ++i)
      {
        if (QString(tm->getTool(i)->getClassId()) == "rviz_default_plugins/PublishPoint")
        {
          tm->setCurrentTool(tm->getTool(i));
          break;
        }
      }
    }
  }

  void ActorPanel::rebuildGoalListWidget()
  {
    goal_list_widget_->clear();
    for (auto const &[gid, pos] : loaded_global_goals_)
    {
      goal_list_widget_->addItem(
          QString("Goal %1: (%2, %3)")
              .arg(gid)
              .arg(pos.x, 0, 'f', 3)
              .arg(pos.y, 0, 'f', 3));
    }
  }

  void ActorPanel::onEnterGoalPickingMode()
  {
    goal_picking_mode_ = !goal_picking_mode_;

    enter_goal_mode_btn_->setText(goal_picking_mode_
                                      ? "Exit Goal-Picking Mode"
                                      : "Enter Goal-Picking Mode");
    if (!goal_picking_mode_)
      enter_goal_mode_btn_->setDown(false);
    else
      enter_goal_mode_btn_->setDown(true);

    // only the first time, pop up instructions
    if (!first_goal_picking_info_shown_)
    {
      first_goal_picking_info_shown_ = true;

      QString msg = QString(
                        "<html>"
                        "<b>Click on the map to set navigation goals</b>.<br>"
                        "If you want to <b>modify</b> an already set goal, just <b>click on its marker</b>.<br><br>"
                        "<b>Please note</b>: Be sure to keep goals \"visible\" to each other (no obstacles in between) to avoid navigation issues.<br><br>"
                        "When you’re done, click <b><i>%1</i></b> and then <b><i>%2</i></b>."
                        "</html>")
                        .arg(enter_goal_mode_btn_->text())
                        .arg(assign_goals_btn_->text());

      QMessageBox::information(
          this,
          tr("Goal-Picking Mode"),
          msg);
    }

    // ── Automatically switch RViz into the PublishPoint tool ──
    if (auto *tm = getDisplayContext()->getToolManager())
    {
      const int count = tm->numTools();
      for (int i = 0; i < count; ++i)
      {
        rviz_common::Tool *tool = tm->getTool(i);
        if (QString(tool->getClassId()) == "rviz_default_plugins/PublishPoint")
        {
          tm->setCurrentTool(tool);
          break;
        }
      }
    }

    bool anyGoals = !goal_ids_.empty();
    assign_goals_btn_->setEnabled(!goal_picking_mode_ && anyGoals);
  }

  /**
   * @brief Open dialog for assigning goals to specific agents
   *
   * Creates an interactive dialog that allows users to assign picked goals to individual agents.
   * The dialog provides visual feedback with agent-specific colors and validates that all agents
   * have at least one goal assigned before completion.
   */
  void ActorPanel::onAssignGoalsClicked()
  {
    // 1) Build and position a tool-style dialog
    QDialog dlg(this);
    dlg.setWindowFlags(dlg.windowFlags() | Qt::Tool);
    QPoint top_left = this->mapToGlobal(QPoint(-30, 0));
    dlg.move(top_left);
    dlg.setWindowTitle("Assign goals to agents");
    dlg.resize(400, 300);
    assign_goals_btn_->setDown(true);

    if (!loaded_agent_goals_.empty())
    {
      agent_goals_ = loaded_agent_goals_;
      num_actors_ = (int)loaded_agent_names_.size();
      agent_colors_.clear();
      for (int i = 0; i < num_actors_; ++i)
      {
        QColor c;
        c.setHsvF(double(i) / num_actors_, 0.8, 0.9);
        agent_colors_.push_back(c);
      }
    }

    // 2) Main layout
    auto *main_layout = new QVBoxLayout(&dlg);

    // — Agent selector with per-item colors
    auto *agent_sel = new QComboBox;
    for (int i = 0; i < int(agent_goals_.size()); ++i)
    {
      QString label = QString("Agent %1").arg(i + 1);
      agent_sel->addItem(label);
    }
    main_layout->addWidget(new QLabel("Select agent:"));
    main_layout->addWidget(agent_sel);

    // — Available vs Assigned lists + buttons
    auto *titles = new QHBoxLayout;
    titles->addWidget(new QLabel("Available Goals"));
    titles->addStretch();
    titles->addWidget(new QLabel("Assigned Goals"));
    main_layout->addLayout(titles);

    auto *lists_layout = new QHBoxLayout;
    auto *avail_list = new QListWidget;
    auto *assigned_list = new QListWidget;
    auto *btn_layout = new QVBoxLayout;
    auto *add_btn = new QPushButton("▶");
    auto *remove_btn = new QPushButton("◀");
    btn_layout->addStretch();
    btn_layout->addWidget(add_btn);
    btn_layout->addWidget(remove_btn);
    btn_layout->addStretch();

    lists_layout->addWidget(avail_list);
    lists_layout->addLayout(btn_layout);
    lists_layout->addWidget(assigned_list);
    main_layout->addLayout(lists_layout);

    // — Lock Selection button
    auto *lock_btn = new QPushButton("Lock Selection");
    lock_btn->setEnabled(false);
    main_layout->addWidget(lock_btn);

    // — Summary area for locked agents
    auto *summary_area = new QVBoxLayout;
    main_layout->addLayout(summary_area);

    // keep track of which agents have been “locked”
    QVector<bool> lockedFlags(num_actors_, false);

    QDialogButtonBox *button_box = new QDialogButtonBox(&dlg);

    finishBtn_ = button_box->addButton(
        tr("Finish"),
        QDialogButtonBox::AcceptRole);
    button_box->addButton(QDialogButtonBox::Cancel);
    main_layout->addWidget(button_box);
    finishBtn_->setEnabled(false);

    finishBtn_->setEnabled(panel_mode_ == EDIT_MODE);

    connect(finishBtn_, &QAbstractButton::clicked, this, [this]()
            {
      resetGoalMarkerColors();
      goal_markers_pub_->publish(goal_markers_); });

    // — Refresh helper
    auto refresh = [&]()
    {
      avail_list->clear();
      assigned_list->clear();
      int a = agent_sel->currentIndex();
      QSet<int> assigned_set(agent_goals_[a].begin(), agent_goals_[a].end());
      for (int gid : goal_ids_)
      {
        if (!assigned_set.contains(gid))
        {
          avail_list->addItem(QString("Goal %1").arg(gid));
        }
      }
      for (int gid : agent_goals_[a])
      {
        assigned_list->addItem(QString("Goal %1").arg(gid)); // + 1
      }
      lock_btn->setEnabled(!agent_goals_[a].empty());
    };

    // — Switch agent handler
    connect(agent_sel, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [&](int new_agent_idx)
            {
            // 1) reset every goal marker back to default
            for (auto &m : goal_markers_.markers) {
              if (m.ns == "goal_points") {
                m.color.r = 0.0f;  m.color.g = 0.7f;  m.color.b = 0.7f;  m.color.a = 1.0f;
              }
              if (m.ns == "goal_numbers") {
                m.color.r = 1.0f;  m.color.g = 1.0f;  m.color.b = 1.0f;  m.color.a = 1.0f;
              }
            }

            // 2) color just the picked goals for this agent
            QColor c = agent_colors_[new_agent_idx];
            float rf = c.redF(),   gf = c.greenF(),   bf = c.blueF();
            for (int gid : agent_goals_[new_agent_idx]) {
              int sphere_id = gid*2, text_id = gid*2+1;
              for (auto &m : goal_markers_.markers) {
                if ((m.ns=="goal_points"  && m.id==sphere_id) ||
                    (m.ns=="goal_numbers" && m.id==text_id))
                {
                  m.color.r = rf;  m.color.g = gf;  m.color.b = bf;  m.color.a = 1.0f;
                }
              }
            }

            // 3) push the recolored markers out
            goal_markers_pub_->publish(goal_markers_);

            // 4) repaint the combo text in that agent’s color
            QPalette pal = agent_sel->palette();
            pal.setColor(QPalette::Text, c);
            agent_sel->setPalette(pal);

            // 5) rebuild the available/assigned lists
            refresh(); 
            
            bool hasGoals = !agent_goals_[new_agent_idx].empty();
            lock_btn->setEnabled(hasGoals && !lockedFlags[new_agent_idx]);
            if (panel_mode_ == CREATE_MODE)
              finishBtn_->setEnabled(false); });

    // — Add goal to agent
    connect(add_btn, &QPushButton::clicked, this, [&]()
            {
            int a = agent_sel->currentIndex();
            for (auto* it : avail_list->selectedItems()) {
            // int displayed = it->text().split(' ').last().toInt();
            // int gi = displayed - 1;                 
            // agent_goals_[a].push_back(gi);
            int gid = it->text().split(' ').last().toInt();
            agent_goals_[a].push_back(gid);
            // paint both the sphere and the text blue
            for (auto &m : goal_markers_.markers) {
                        if ((m.ns=="goal_points"  && m.id == gid*2) ||
                            (m.ns=="goal_numbers" && m.id == gid*2+1))
                {
                m.color.r = m.color.g = 0.0f;
                m.color.b = 1.0f;
                m.color.a = 1.0f;
                }
            }
            }
            goal_markers_pub_->publish(goal_markers_);
            refresh(); });

    // — Remove goal from agent
    connect(remove_btn, &QPushButton::clicked, this, [&]()
            {
    int a = agent_sel->currentIndex();
    auto &vec = agent_goals_[a];

    for (auto* it : assigned_list->selectedItems()) {
        // parse out the 1-based goal ID directly
        int gid = it->text().split(' ').last().toInt();

        // remove it from this agent’s assignments
        vec.erase(std::remove(vec.begin(), vec.end(), gid), vec.end());

        // reset *both* sphere and text markers for this goal back to default
        for (auto &m : goal_markers_.markers) {
            if (m.ns == "goal_points" && m.id == gid*2) {
                m.color.r = 0.0f;
                m.color.g = 0.7f;
                m.color.b = 0.7f;
                m.color.a = 1.0f;
            }
            if (m.ns == "goal_numbers" && m.id == gid*2+1) {
                m.color.r = 1.0f;
                m.color.g = 1.0f;
                m.color.b = 1.0f;
                m.color.a = 1.0f;
            }
        }
    }

    // push the color changes out to RViz
    goal_markers_pub_->publish(goal_markers_);

    // then rebuild the two lists
    refresh(); });

    // — Lock and summarize
    connect(lock_btn, &QPushButton::clicked, this, [&]()
            {
            int a = agent_sel->currentIndex();

            // ─── wipe out any previous summary row for this agent ───
            if (auto oldItem = summary_area->takeAt(a))
            {
              if (auto oldLayout = oldItem->layout())
              {
                // this will delete all widgets in that row
                QLayoutItem *child;
                while ((child = oldLayout->takeAt(0)) != nullptr)
                {
                  delete child->widget();
                  delete child;
                }
                delete oldLayout;
              }
            }

            lockedFlags[a] = true; 
            lock_btn->setEnabled(false);
            // summary row
            QStringList goal_strs;
            for (int gi : agent_goals_[a]) goal_strs << QString::number(gi);
            QString joined = goal_strs.join(", ");
            auto *row = new QHBoxLayout;
            auto *sq  = new QLabel;
            auto *lbl = new QLabel(QString("Agent %1 goals: [%2]").arg(a+1).arg(joined));
            QPixmap pix(16,16); pix.fill(agent_colors_[a]);
            sq->setPixmap(pix); sq->setFixedSize(16,16);
            row->addWidget(sq); row->addWidget(lbl); row->addStretch();
            summary_area->insertLayout(a, row);
            // recolor all that agent’s goals
            for (auto &m : goal_markers_.markers) {
            int gi = m.id / 2;
            if (std::find(agent_goals_[a].begin(),
                            agent_goals_[a].end(), gi)
                != agent_goals_[a].end()
                && (m.ns=="goal_numbers"))
            {
                QColor c = agent_colors_[a];
                m.color.r = c.redF();
                m.color.g = c.greenF();
                m.color.b = c.blueF();
                m.color.a = 1.0f;
            }
            }
            goal_markers_pub_->publish(goal_markers_); 
            bool allLocked = std::all_of(
              lockedFlags.begin(), lockedFlags.end(),
              [](bool v){ return v; });
            finishBtn_->setEnabled(panel_mode_ == EDIT_MODE || allLocked); });

    // — Dialog buttons
    connect(button_box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(button_box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    QString mapName = QFileInfo(map_file_).baseName();

    {
      int a = agent_sel->currentIndex();
      // reset all markers to default
      for (auto &m : goal_markers_.markers)
      {
        if (m.ns == "goal_numbers")
        {
          m.color.r = m.color.g = m.color.b = 1.0f;
          m.color.a = 1.0f;
        }
        else if (m.ns == "goal_points")
        {
          m.color.r = 0.0f;
          m.color.g = 0.7f;
          m.color.b = 0.7f;
          m.color.a = 1.0f;
        }
      }
      // now recolor just the assigned ones
      for (int gid : agent_goals_[a])
      {
        // each goal has two markers: sphere (gid*2) and text (gid*2+1)
        for (auto &m : goal_markers_.markers)
        {
          if ((m.ns == "goal_numbers" || m.ns == "goal_points") && (m.id == gid * 2 || m.id == gid * 2 + 1))
          {
            QColor c = agent_colors_[a];
            m.color.r = c.redF();
            m.color.g = c.greenF();
            m.color.b = c.blueF();
            m.color.a = 1.0f;
          }
        }
      }
      goal_markers_pub_->publish(goal_markers_);
    }

    // Initial populate & execute
    refresh();
    dlg.exec();

    if (dlg.result() != QDialog::Accepted)
      return;

    assign_goals_btn_->setDown(false);

    if (panel_mode_ == CREATE_MODE)
    {
      // Sanity checks
      if (goal_ids_.empty())
      {
        QMessageBox::warning(this, "No goals defined",
                             "Pick at least one goal before generating files.");
        return;
      }
      for (int i = 0; i < num_actors_; ++i)
      {
        if (agent_goals_[i].empty())
        {
          QMessageBox::warning(this, "Unassigned goal",
                               QString("Agent %1 has no goals assigned.").arg(i + 1));
          return;
        }
      }

      loaded_agent_goals_ = agent_goals_;
      save_bt_btn_->setEnabled(true);
      checkbox->setEnabled(true);
      QMessageBox::information(
          this,
          tr("Goals picked and assigned"),
          tr("\n"
             "<html>Now click <i><b>%1</b></i> to write out the agents YAML & BTs.</html>")
              .arg(save_bt_btn_->text()));
    }
    else
    {
      // EDIT: copy back from the dialog’s temporary arrays
      loaded_agent_goals_ = agent_goals_;
      save_bt_btn_->setEnabled(true);
      checkbox->setEnabled(true);
      QMessageBox::information(
          this,
          tr("Goals updated"),
          tr("<html>"
             "Your edited goals have been saved in memory.<br>"
             "Now click on <i><b>%1</b></i><br>to write out the updated YAML and behavior trees."
             "</html>")
              .arg(save_bt_btn_->text()));
    }
  }

  void ActorPanel::onResetLoadedGoals()
  {
    // 1) Clear the in‐memory maps/vectors
    loaded_global_goals_.clear();
    goal_ids_.clear();
    // Each agent’s own “loaded goals” also needs clearing:
    for (auto &vec : loaded_agent_goals_)
    {
      vec.clear();
    }

    // 2) Delete all RViz markers under “goal_points” and “goal_numbers”
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = "/map";
    delete_all.header.stamp = rclcpp::Clock().now();
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;

    {
      auto arr = std::make_unique<visualization_msgs::msg::MarkerArray>();
      delete_all.ns = "goal_points";
      arr->markers.push_back(delete_all);
      goal_markers_pub_->publish(std::move(arr));
    }
    {
      auto arr = std::make_unique<visualization_msgs::msg::MarkerArray>();
      delete_all.ns = "goal_numbers";
      arr->markers.push_back(delete_all);
      goal_markers_pub_->publish(std::move(arr));
    }

    // Also clear our local copy of “goal_markers_” so future onGoalPicked() starts clean:
    goal_markers_.markers.clear();

    // 3) Clear the QListWidget
    goal_list_widget_->clear();

    // 4) Disable “Assign goals”
    assign_goals_btn_->setEnabled(false);

    QMessageBox::information(this,
                             tr("Goals Reset"),
                             tr("All previously-loaded goals have been cleared.\n"
                                "You may now pick new goals on the map."));
  }

  void ActorPanel::onCreateOrEditAgents()
  {
    actor_button_->setDown(true);
    if (panel_mode_ == CREATE_MODE)
    {
      // 1) figure out how many agents
      num_actors_ = actors->text().toInt();
      // 2) initialize colors, goal‐lists, and per‐agent ID slots
      initAgentColors(num_actors_);
      agent_goals_.assign(num_actors_, {});
      loaded_initial_marker_ids_.assign(num_actors_, -1);
      next_marker_id_ = 0;
      addAgent();
      return;
    }
    // initial_pose_publisher->publish(std::move(marker_array_));

    clearNonAgentMarkers();
    publishAgentMarkers();

    current_edit_idx_ = 0;
    panel_mode_ = EDIT_MODE;

    addAgent();
  }

  /**
   * @brief Parse and load agents configuration from YAML file
   *
   * This method handles loading existing agent configurations from a YAML file.
   * It parses the file structure, loads agent data, goal information, and switches
   * the panel to edit mode for modifying existing configurations.
   */
  void ActorPanel::parseYaml()
  {
    // Remove any existing RViz markers
    removeCurrentMarkers();

    // Let the user pick exactly one YAML file, starting inside the correct dir
    QString simulatorName = simulator_combo_->currentText();
    QString configDir;

    if (simulatorName == "Gazebo")
    {
      QString shareDir;
      try
      {
        shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory("hunav_gazebo_wrapper"));
      }
      catch (const std::exception &e)
      {
        // fallback to home‐installed wrapper if package not found
        QString homePath = QDir::homePath() + "/hunav_gazebo_wrapper";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_gazebo_wrapper";
        shareDir = QDir(dockerPath).exists() ? dockerPath : homePath;
      }
      configDir = shareDir + "/scenarios";
    }
    else if (simulatorName == "Isaac Sim")
    {
      QString homePath = QDir::homePath() + "/Hunav_isaac_wrapper";
      QString dockerPath = "/workspace/hunav_isaac_ws/src/Hunav_isaac_wrapper";
      QString basePath = QDir(dockerPath).exists() ? dockerPath : homePath;
      configDir = basePath + "/scenarios";
    }
    else // Webots or other
    {
      QString homePath = QDir::homePath() + "/hunav_webots_wrapper";
      QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_webots_wrapper";
      QString basePath = QDir(dockerPath).exists() ? dockerPath : homePath;
      configDir = basePath + "/scenarios";
    }

    QString chosenFile = QFileDialog::getOpenFileName(
        this,
        "Load agents YAML",
        configDir,
        "YAML files (*.yaml *.yml)");
    if (chosenFile.isEmpty())
      return;

    pkg_shared_tree_dir_ = chosenFile.toStdString();
    orig_yaml_base_name_ = QFileInfo(chosenFile).baseName();
    yaml_file_label_->setText(orig_yaml_base_name_);
    yaml_file_label_->show();

    RCLCPP_INFO(this->get_logger(), "Loading YAML from: %s", pkg_shared_tree_dir_.c_str());

    YAML::Node yaml_file;
    try
    {
      yaml_file = YAML::LoadFile(pkg_shared_tree_dir_);
    }
    catch (const YAML::Exception &ex)
    {
      QMessageBox::critical(
          this,
          "YAML Load Error",
          QString("Failed to load YAML file:\n%1\n\n%2")
              .arg(QString::fromStdString(pkg_shared_tree_dir_))
              .arg(QString::fromStdString(ex.what())));
      return;
    }

    // Extract "hunav_loader/ros__parameters"
    if (!yaml_file["hunav_loader"] ||
        !yaml_file["hunav_loader"]["ros__parameters"])
    {
      QMessageBox::warning(
          this,
          "YAML Format Error",
          "The file does not contain 'hunav_loader/ros__parameters'.");
      return;
    }
    YAML::Node params = yaml_file["hunav_loader"]["ros__parameters"];
    params_ = params;

    if (params_["simulator"])
    {
      simulator_combo_->setCurrentText(
          QString::fromStdString(params_["simulator"].as<std::string>()));
    }

    // Immediately load the map named under params["map"]
    if (params["map"])
    {
      std::string mapName = params["map"].as<std::string>();
      QString mapBasename = QString::fromStdString(mapName) + ".yaml";

      // determine the base maps directory
      QString simulatorName = simulator_combo_->currentText();
      QString mapDir;
      if (simulatorName == "Gazebo")
      {
        QString shareDir;
        try
        {
          shareDir = QString::fromStdString(
              ament_index_cpp::get_package_share_directory("hunav_gazebo_wrapper"));
        }
        catch (const std::exception &e)
        {
          QString homePath = QDir::homePath() + "/hunav_gazebo_wrapper";
          QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_gazebo_wrapper";
          shareDir = QDir(dockerPath).exists() ? dockerPath : homePath;
        }
        mapDir = shareDir + "/maps";
      }
      else if (simulatorName == "Isaac Sim")
      {
        QString homePath = QDir::homePath() + "/Hunav_isaac_wrapper";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/Hunav_isaac_wrapper";
        QString basePath = QDir(dockerPath).exists() ? dockerPath : homePath;
        mapDir = basePath + "/maps";
      }
      else // Webots
      {
        QString homePath = QDir::homePath() + "/hunav_webots_wrapper";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_webots_wrapper";
        QString basePath = QDir(dockerPath).exists() ? dockerPath : homePath;
        mapDir = basePath + "/maps";
      }

      // try to locate it
      QString candidatePath = mapDir + "/" + mapBasename;
      if (!QFile::exists(candidatePath))
      {
        QMessageBox::critical(
            this,
            "Map Load Error",
            QString("Could not locate '%1' in:\n  %2")
                .arg(mapBasename)
                .arg(mapDir));
        return;
      }

      // 2) Call the map_server/load_map service
      auto client = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
      if (!client->wait_for_service(2s))
      {
        QMessageBox::warning(
            this,
            "Map Server",
            "Timed out waiting for /map_server/load_map. Is map_server running?");
        return;
      }
      auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
      req->map_url = candidatePath.toStdString();

      auto future = client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) != rclcpp::FutureReturnCode::SUCCESS)
      {
        QMessageBox::critical(
            this,
            "Map Server",
            "Failed to call /map_server/load_map on:\n" + candidatePath);
        return;
      }
    }
    else
    {
      QMessageBox::warning(
          this,
          "YAML Format Error",
          "Missing 'map' key under 'ros__parameters'.");
      return;
    }

    // 6) Build a goal_map from params["global_goals"]
    loaded_global_goals_.clear();
    if (params["global_goals"] && params["global_goals"].IsMap())
    {
      for (const auto &it : params["global_goals"].as<YAML::Node>())
      {
        int goal_id = it.first.as<int>();
        double gx = it.second["x"].as<double>();
        double gy = it.second["y"].as<double>();

        geometry_msgs::msg::Point pt;
        pt.x = gx;
        pt.y = gy;
        pt.z = 0.0;
        loaded_global_goals_[goal_id] = pt;
        goal_ids_.push_back(goal_id);
      }
    }
    else
    {
      QMessageBox::warning(
          this,
          "YAML Format Error",
          "Missing or invalid 'global_goals' section in agents.yaml.");
      return;
    }

    loaded_agent_names_.clear();
    loaded_agent_nodes_.clear();
    loaded_agent_goals_.clear();

    // 7) Read the list of agent‐names from params["agents"]
    std::vector<std::string> agents_list;
    if (params["agents"] && params["agents"].IsSequence())
    {
      for (auto const &entry : params["agents"])
      {
        agents_list.push_back(entry.as<std::string>());

        std::string agent_name = entry.as<std::string>();
        loaded_agent_names_.push_back(agent_name);

        YAML::Node sub = params[agent_name];
        loaded_agent_nodes_.push_back(sub);

        // pull out that agent’s “goals” array:
        std::vector<int> this_goals;
        if (sub["goals"] && sub["goals"].IsSequence())
        {
          for (auto const &g : sub["goals"])
            this_goals.push_back(g.as<int>());
        }
        loaded_agent_goals_.push_back(this_goals);
      }
    }
    else
    {
      QMessageBox::warning(
          this,
          "YAML Format Error",
          "Missing or invalid 'agents' sequence in agents.yaml.");
      return;
    }

    goal_markers_.markers.clear();
    int base_id = 0;
    for (auto const &it : loaded_global_goals_)
    {
      int gid = it.first;
      auto pt = it.second;

      // (a) Make a sphere marker
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "/map";
      sphere.header.stamp = rclcpp::Clock().now();
      sphere.ns = "goal_points";
      // sphere.id = base_id * 2; // even ID
      sphere.id = gid * 2;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position = pt;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2;
      sphere.color.r = 0.0f;
      sphere.color.g = 0.7f;
      sphere.color.b = 0.7f;
      sphere.color.a = 1.0f;
      goal_markers_.markers.push_back(sphere);

      // (b) Make a text label just above it
      visualization_msgs::msg::Marker text;
      text.header.frame_id = "/map";
      text.header.stamp = rclcpp::Clock().now();
      text.ns = "goal_numbers";
      // text.id = base_id * 2 + 1; // odd ID
      text.id = gid * 2 + 1;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = sphere.pose;
      text.pose.position.y += 0.5;
      text.pose.position.z += 0.3;
      text.scale.z = 0.7;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = std::to_string(gid);
      goal_markers_.markers.push_back(text);

      ++base_id;
    }

    goal_list_widget_->clear();
    for (auto const &it : loaded_global_goals_)
    {
      int gid = it.first;
      auto &pt = it.second;
      QString label = QString("Goal %1: (%2, %3)")
                          .arg(gid)
                          .arg(QString::number(pt.x, 'f', 3))
                          .arg(QString::number(pt.y, 'f', 3));
      goal_list_widget_->addItem(label);
    }

    // 8) Prepare a fresh MarkerArray for visualization
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

    // 9) Loop over agents, assign each a distinct color, draw initial‐pose + ID, then its goals + arrows
    const size_t num_agents = agents_list.size();
    loaded_initial_marker_ids_.assign(num_agents, -1);
    int id_counter = 0;

    for (size_t a = 0; a < num_agents; ++a)
    {
      const std::string &agent_name = agents_list[a];
      YAML::Node agent_node = params[agent_name];
      if (!agent_node || !agent_node["init_pose"] || !agent_node["goals"])
      {
        RCLCPP_WARN(
            this->get_logger(),
            "Skipping '%s' because it lacks 'init_pose' or 'goals'.",
            agent_name.c_str());
        continue;
      }

      // 9a) Agent initial‐pose (mesh or sphere)
      double ipx = agent_node["init_pose"]["x"].as<double>();
      double ipy = agent_node["init_pose"]["y"].as<double>();

      // pull out the saved yaw ("h") if present
      double yaw = 0.0;
      if (agent_node["init_pose"]["h"])
        yaw = agent_node["init_pose"]["h"].as<double>();

      // Choose skin if present:
      int raw_skin = 0;
      if (agent_node["skin"])
        raw_skin = agent_node["skin"].as<int>();
      checkParserSkin(raw_skin);

      int id = id_counter++;
      loaded_initial_marker_ids_[a] = id;

      visualization_msgs::msg::Marker agent_marker =
          createMarker(ipx, ipy, id, "person", "parser");
      agent_marker.ns = "agent_initial";
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      agent_marker.pose.orientation = tf2::toMsg(q);
      marker_array->markers.push_back(agent_marker);

      // 9b) Floating text above the agent to show its ID (1-based index)
      QColor qcol;
      qcol.setHsvF(double(a) / double(num_agents), 0.8, 0.9);
      visualization_msgs::msg::Marker id_text;
      id_text.header.frame_id = agent_marker.header.frame_id;
      id_text.header.stamp = rclcpp::Clock().now();
      id_text.ns = "agent_id_text";
      id_text.id = id;
      id_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      id_text.action = visualization_msgs::msg::Marker::ADD;
      id_text.pose.position.x = ipx;
      id_text.pose.position.y = ipy + 0.7;
      id_text.pose.position.z = 1.5; // float above the agent
      id_text.scale.z = 0.8;         // text height
      id_text.color.r = qcol.redF();
      id_text.color.g = qcol.greenF();
      id_text.color.b = qcol.blueF();
      id_text.color.a = 1.0f;
      id_text.text = std::to_string(int(a) + 1);
      marker_array->markers.push_back(id_text);

      // 9c) Read this agent’s assigned goal IDs
      std::vector<int> assigned_goals;
      for (const auto &gid_node : agent_node["goals"])
      {
        assigned_goals.push_back(gid_node.as<int>());
      }

      // 9d) Draw each goal cube + arrow from previous point
      geometry_msgs::msg::Point prev_pt;
      prev_pt.x = ipx;
      prev_pt.y = ipy;
      prev_pt.z = 0.0;

      for (size_t idx = 0; idx < assigned_goals.size(); ++idx)
      {
        int gid = assigned_goals[idx];
        auto it = loaded_global_goals_.find(gid);
        if (it == loaded_global_goals_.end())
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Goal ID %d not found in global_goals; skipping.", gid);
          continue;
        }

        geometry_msgs::msg::Point goal_pt = it->second;

        // small colored cube at goal_pt
        visualization_msgs::msg::Marker goal_marker =
            createMarker(goal_pt.x, goal_pt.y, id_counter++, "cube", "parser");
        goal_marker.ns = "agent_goal";
        goal_marker.color.r = qcol.redF();
        goal_marker.color.g = qcol.greenF();
        goal_marker.color.b = qcol.blueF();
        goal_marker.color.a = 1.0f;
        marker_array->markers.push_back(goal_marker);

        visualization_msgs::msg::Marker text_marker;
        text_marker.header = goal_marker.header; // same timestamp/frame
        text_marker.ns = "goal_labels";
        text_marker.id = id_counter++; // unique ID
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        // Copy the cube’s pose, but bump Z up a bit so the text floats above
        text_marker.pose = goal_marker.pose;
        text_marker.pose.position.z += 0.2;
        text_marker.pose.position.y += 0.7;
        // Text size (height)
        text_marker.scale.z = 0.7;

        // White (or any) color
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0f;

        // Show the goal’s integer ID
        text_marker.text = std::to_string(gid);

        marker_array->markers.push_back(text_marker);

        // (2) arrow from prev_pt → goal_pt (same color)
        visualization_msgs::msg::Marker arrow_marker =
            createArrowMarker(prev_pt.x, prev_pt.y, goal_pt.x, goal_pt.y, id_counter++);
        arrow_marker.ns = "agent_arrow";
        arrow_marker.color.r = qcol.redF();
        arrow_marker.color.g = qcol.greenF();
        arrow_marker.color.b = qcol.blueF();
        arrow_marker.color.a = 1.0f;
        marker_array->markers.push_back(arrow_marker);

        prev_pt = goal_pt;
      }

      // 9e) Finally, draw an arrow from the last goal back to the first goal
      if (!assigned_goals.empty())
      {
        // get the first goal’s coordinates
        int first_gid = assigned_goals.front();
        auto it_first = loaded_global_goals_.find(first_gid);
        if (it_first != loaded_global_goals_.end())
        {
          const auto &first_pt = it_first->second;
          visualization_msgs::msg::Marker closing_arrow =
              createArrowMarker(prev_pt.x, prev_pt.y,
                                first_pt.x, first_pt.y,
                                id_counter++);
          closing_arrow.ns = "agent_arrow";
          closing_arrow.color.r = qcol.redF();
          closing_arrow.color.g = qcol.greenF();
          closing_arrow.color.b = qcol.blueF();
          closing_arrow.color.a = 1.0f;
          marker_array->markers.push_back(closing_arrow);
        }
      }

    } // end for(each agent)

    next_marker_id_ = id_counter; // Update the next_marker_id_ to the last used ID

    // 10) Publish all markers at once
    initial_pose_publisher->publish(std::move(marker_array));
    // 11) If there are any agents, switch to EDIT_MODE
    if (!loaded_agent_nodes_.empty())
    {
      int n = static_cast<int>(loaded_agent_nodes_.size());
      initAgentColors(n);
      panel_mode_ = EDIT_MODE;
      current_edit_idx_ = 0;
      actors->hide();
      actor_button_->setText("Edit agents");
      actor_button_->setEnabled(true);
      edit_goals_button_->setVisible(true);
      n_agents_label_->hide();
      // save_bt_btn_->setVisible(true);
      map_group->setTitle("Edit agents or navigation goal:");
      map_group->setEnabled(false);
      map_group->setVisible(false);
      goal_group_->setTitle("");
      map_select_btn_->hide();
      current_map_label_->hide();
      map_select_btn_->setVisible(false);
      reset_goals_button_->show();
      reset_goals_button_->setEnabled(true);
      enter_goal_mode_btn_->hide();
    }
    QMessageBox::information(
        this,
        tr("Agents YAML Loaded"),
        tr("<html>"
           "Successfully loaded %1 agents from:<br><i><b>%2</b></i><br><br>"
           "You can now <b>edit their configuration and/or edit goals</b>."
           "</html>")
            .arg(loaded_agent_names_.size())
            .arg(orig_yaml_base_name_));
  }

  /**
   * @brief Save agents configuration and generate behavior trees
   *
   * This method saves the complete agent configuration to a YAML file and generates
   * corresponding behavior tree XML files for each agent. It handles both CREATE and
   * EDIT modes, preserving existing data while updating modified elements.
   */
  void ActorPanel::saveAndGenerateAll()
  {
    // 1) Ask for a filename
    bool ok = false;

    if (panel_mode_ == EDIT_MODE)
      defaultName_ = orig_yaml_base_name_;
    else
      defaultName_ = QFileInfo(map_file_).baseName() + "_agents_";

    QString base;

    // Create QInputDialog
    QInputDialog dlg(this);
    dlg.setWindowTitle("Output YAML Name");
    dlg.setLabelText("Enter a name for the agents YAML file:");
    dlg.setTextValue(defaultName_);
    dlg.setOption(QInputDialog::NoButtons, false);

    // Grab the embedded QLineEdit
    QLineEdit *le = dlg.findChild<QLineEdit *>();
    if (le)
    {
      QTimer::singleShot(0, this, [this, le]()
                         {
                           le->deselect();                              
                           le->setCursorPosition(defaultName_.length()); 
                         });
    }

    // Exec it
    if (dlg.exec() == QDialog::Accepted)
    {
      base = dlg.textValue();
      ok = true;
    }
    else
    {
      ok = false;
    }

    if (!ok || base.trimmed().isEmpty())
    {
      QMessageBox::warning(this, "Missing file name",
                           "Please enter a name for the output YAML file.");
      return;
    }
    yaml_base_name_ = base.trimmed();

    // 2) Build a fresh YAML::Node “root”:
    YAML::Node root;
    auto p = root["hunav_loader"]["ros__parameters"];

    //  2a) write yaml basename, simulator and map name:
    p["yaml_base_name"] = yaml_base_name_.toStdString();

    p["simulator"] = simulator_combo_->currentText().toStdString();
    std::string map_str;
    if (panel_mode_ == EDIT_MODE && params_["map"])
    {
      map_str = params_["map"].as<std::string>();
    }
    else if (!map_file_.isEmpty())
    {
      map_str = QFileInfo(map_file_).baseName().toStdString();
    }
    else
    {
      // fallback
      map_str = "warehouse";
    }
    p["map"] = map_str;

    p["publish_people"] = true;

    //  2b) write “global_goals”:
    p["global_goals"] = YAML::Node(YAML::NodeType::Map);
    for (auto const &it : loaded_global_goals_)
    {
      int gid = it.first;
      // create a sub‐map with x,y (rounded to 3 decimals)
      double x = it.second.x;
      double y = it.second.y;

      YAML::Node one;
      one["x"] = QString::number(x, 'f', 3).toStdString();
      one["y"] = QString::number(y, 'f', 3).toStdString();
      p["global_goals"][gid] = one;
    }

    if (panel_mode_ == CREATE_MODE)
    {
      loaded_agent_names_.clear();
      for (size_t i = 0; i < loaded_agent_nodes_.size(); ++i)
      {
        loaded_agent_names_.push_back("agent" + std::to_string(i + 1));
      }
    }

    //  2c) write “agents:” sequence
    p["agents"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto const &name : loaded_agent_names_)
      p["agents"].push_back(name);

    //  2d) For each agent, re‐insert its node but overwrite the “goals” block:
    for (size_t i = 0; i < loaded_agent_names_.size(); ++i)
    {
      std::string agent_name = loaded_agent_names_[i];
      YAML::Node agent_node = loaded_agent_nodes_[i];

      // Overwrite or create a “goals” sequence with its updated IDs:
      agent_node["goals"] = YAML::Node(YAML::NodeType::Sequence);
      for (int gid : loaded_agent_goals_[i])
        agent_node["goals"].push_back(gid);

      p[agent_name] = agent_node;
    }

    // 3) Write the YAML to disk:
    QString configDir;
    QString btDir;
    QString simulatorName = simulator_combo_->currentText();

    // Determine the simulator wrapper directories
    if (simulatorName == "Gazebo")
    {
      QString shareDir;
      try
      {
        shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory("hunav_gazebo_wrapper"));
      }
      catch (const std::exception &e)
      {
        // fallback to home‐installed wrapper if package not found
        QString homePath = QDir::homePath() + "/hunav_gazebo_wrapper";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_gazebo_wrapper";
        shareDir = QDir(dockerPath).exists() ? dockerPath : homePath;
      }
      configDir = shareDir + "/scenarios";
      btDir = shareDir + "/behavior_trees";
    }
    else if (simulatorName == "Isaac Sim")
    {
      QString homePath = QDir::homePath() + "/Hunav_isaac_wrapper";
      QString dockerPath = "/workspace/hunav_isaac_ws/src/Hunav_isaac_wrapper";
      QString basePath = QDir(dockerPath).exists() ? dockerPath : homePath;
      configDir = basePath + "/scenarios";
      btDir = basePath + "/behavior_trees";
    }
    else // Webots
    {
      QString homePath = QDir::homePath() + "/hunav_webots_wrapper";
      QString dockerPath = "/workspace/hunav_isaac_ws/src/hunav_webots_wrapper";
      QString basePath = QDir(dockerPath).exists() ? dockerPath : homePath;
      configDir = basePath + "/scenarios";
      btDir = basePath + "/behavior_trees";
    }
    QDir().mkpath(configDir);
    QDir().mkpath(btDir);
    QString outName = QString("%1.yaml").arg(yaml_base_name_);
    QString fullpath = configDir + "/" + outName;

    std::ofstream ofs(fullpath.toStdString());
    if (!ofs.is_open())
    {
      QMessageBox::critical(this, "Write Error",
                            "Cannot open file:\n" + fullpath);
      return;
    }
    ofs << root;
    ofs.close();

    RCLCPP_INFO(this->get_logger(),
                "Wrote updated agents.yaml to %s",
                fullpath.toStdString().c_str());

    // 4) Now regenerate each agent’s BT:
    QString mapName = QString::fromStdString(p["map"].as<std::string>());
    QString pkg = QString::fromStdString(
        ament_index_cpp::get_package_share_directory("hunav_agent_manager"));

    // reload TreeNodesModel.xml:
    QString modelPath = pkg + "/behavior_trees/TreeNodesModel.xml";
    QString modelXml = loadFile(modelPath);
    if (modelXml.isEmpty())
    {
      QMessageBox::critical(this, "Error",
                            "Failed to load TreeNodesModel from:\n" + modelPath);
      return;
    }

    // Find insertion point:
    const QString closingTag = "</TreeNodesModel>";
    int insertPos = modelXml.indexOf(closingTag);
    if (insertPos < 0)
    {
      QMessageBox::critical(this, "Error",
                            "Template missing </TreeNodesModel> tag!");
      return;
    }
    insertPos += closingTag.length();

    // For each agent i:
    for (int i = 0; i < (int)loaded_agent_names_.size(); ++i)
    {
      // 1) Select template upon behavior “type”
      std::string bt = loaded_agent_nodes_[i]["behavior"]["type"].as<std::string>();
      int beh_type = hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
      if (bt == "Regular")
        beh_type = hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
      else if (bt == "Impassive")
        beh_type = hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE;
      else if (bt == "Surprised")
        beh_type = hunav_msgs::msg::AgentBehavior::BEH_SURPRISED;
      else if (bt == "Scared")
        beh_type = hunav_msgs::msg::AgentBehavior::BEH_SCARED;
      else if (bt == "Curious")
        beh_type = hunav_msgs::msg::AgentBehavior::BEH_CURIOUS;
      else if (bt == "Threatening")
        beh_type = hunav_msgs::msg::AgentBehavior::BEH_THREATENING;

      // helper to pull in the Regular‐nav include
      auto includeCommon = [&]()
      {
        return QString("<include path=\"BTRegularNav.xml\" />\n\n");
      };
      auto buildGoalSequence = [&](int agent_idx)
      {
        auto const &indices = loaded_agent_goals_[agent_idx];
        QString seq;
        for (int j = 0; j + 1 < (int)indices.size(); ++j)
        {
          seq += QString(
                     "        <RunOnce>\n"
                     "          <SetGoal agent_id=\"{id}\" goal_id=\"%1\"/>\n"
                     "        </RunOnce>\n")
                     .arg(indices[j]);
        }
        if (!indices.empty())
        {
          seq += QString(
                     "        <Inverter>\n"
                     "          <RunOnce>\n"
                     "            <SetGoal agent_id=\"{id}\" goal_id=\"%1\"/>\n"
                     "          </RunOnce>\n"
                     "        </Inverter>\n")
                     .arg(indices.back());
        }
        return seq;
      };

      // Generate behavior tree for current agent based on its behavior type
      switch (beh_type)
      {
      case hunav_msgs::msg::AgentBehavior::BEH_SCARED:
      {
        double dist = loaded_agent_nodes_[i]["behavior"]["visibility_distance"].as<double>();
        double duration = loaded_agent_nodes_[i]["behavior"]["duration"].as<double>();
        bool once = loaded_agent_nodes_[i]["behavior"]["only_once"].as<bool>();
        double maxvel = loaded_agent_nodes_[i]["max_vel"].as<double>();
        double force = loaded_agent_nodes_[i]["behavior"]["scary_force_factor"].as<double>();

        btBlock_ = QString(R"(
%1
<BehaviorTree ID="ScaredNavTree">
  <Fallback name="ScaredFallback">
    <!-- Goal setting Sequence -->
    <Sequence name="SetGoals">
%2
    </Sequence>
    <Sequence name="ScaNav">
      <IsRobotVisible agent_id="{id}" distance="%3"/>
      <Inverter>
        <TimeExpiredCondition seconds="%4" ts="{dt}" only_once="%5"/>
      </Inverter>
      <ScaredNav agent_id="{id}" time_step="{dt}" runaway_vel="%6" scary_force_factor="%7"/>
    </Sequence>
    <!-- then fallback to regular nav -->
    <Sequence name="RegNav">
      <SetBlackboard output_key="agentid" value="{id}"/>
      <SetBlackboard output_key="timestep" value="{dt}"/>
      <SubTree ID="RegularNavTree" id="{agentid}" dt="{timestep}"/>
    </Sequence>
  </Fallback>
</BehaviorTree>
)")
                       .arg(includeCommon())
                       .arg(buildGoalSequence(i))
                       .arg(dist)
                       .arg(duration)
                       .arg(once ? "true" : "false")
                       .arg(maxvel)
                       .arg(force);
      }
      break;

      case hunav_msgs::msg::AgentBehavior::BEH_SURPRISED:
      {
        double dist = loaded_agent_nodes_[i]["behavior"]["visibility_distance"].as<double>();
        double duration = loaded_agent_nodes_[i]["behavior"]["duration"].as<double>();
        bool once = loaded_agent_nodes_[i]["behavior"]["only_once"].as<bool>();

        btBlock_ = QString(R"(
%1
<BehaviorTree ID="SurprisedNavTree">
  <Fallback name="SurprisedFallback">
    <!-- Goal setting Sequence -->
    <Sequence name="SetGoals">
%2
    </Sequence>
    <Sequence name="SurNav">
      <IsRobotVisible agent_id="{id}" distance="%3"/>
      <Inverter>
        <TimeExpiredCondition seconds="%4" ts="{dt}" only_once="%5"/>
      </Inverter>
      <SurprisedNav agent_id="{id}" time_step="{dt}"/>
    </Sequence>
    <Sequence name="RegNav">
      <SetBlackboard output_key="agentid" value="{id}"/>
      <SetBlackboard output_key="timestep" value="{dt}"/>
      <SubTree ID="RegularNavTree" id="{agentid}" dt="{timestep}"/>
    </Sequence>
  </Fallback>
</BehaviorTree>
)")
                       .arg(includeCommon())
                       .arg(buildGoalSequence(i))
                       .arg(dist)
                       .arg(duration)
                       .arg(once ? "true" : "false");
      }
      break;

      case hunav_msgs::msg::AgentBehavior::BEH_CURIOUS:
      {
        double dist = loaded_agent_nodes_[i]["behavior"]["visibility_distance"].as<double>();
        double duration = loaded_agent_nodes_[i]["behavior"]["duration"].as<double>();
        bool once = loaded_agent_nodes_[i]["behavior"]["only_once"].as<bool>();
        double stopdist = loaded_agent_nodes_[i]["behavior"]["stop_distance"].as<double>();
        double maxvel = loaded_agent_nodes_[i]["max_vel"].as<double>();

        btBlock_ = QString(R"(
%1
<BehaviorTree ID="CuriousNavTree">
  <Fallback name="CuriousFallback">
    <!-- Goal setting Sequence -->
    <Sequence name="SetGoals">
%2
    </Sequence>
    <Sequence name="CurNav">
      <IsRobotVisible agent_id="{id}" distance="%3"/>
      <Inverter>
        <TimeExpiredCondition seconds="%4" ts="{dt}" only_once="%5"/>
      </Inverter>
      <CuriousNav agent_id="{id}" time_step="{dt}" stop_distance="%6" agent_vel="%7"/>
    </Sequence>
    <Sequence name="RegNav">
      <SetBlackboard output_key="agentid" value="{id}"/>
      <SetBlackboard output_key="timestep" value="{dt}"/>
      <SubTree ID="RegularNavTree" id="{agentid}" dt="{timestep}"/>
    </Sequence>
  </Fallback>
</BehaviorTree>
)")
                       .arg(includeCommon())
                       .arg(buildGoalSequence(i))
                       .arg(dist)
                       .arg(duration)
                       .arg(once ? "true" : "false")
                       .arg(stopdist)
                       .arg(maxvel);
      }
      break;

      case hunav_msgs::msg::AgentBehavior::BEH_THREATENING:
      {
        double dist = loaded_agent_nodes_[i]["behavior"]["visibility_distance"].as<double>();
        double duration = loaded_agent_nodes_[i]["behavior"]["duration"].as<double>();
        bool once = loaded_agent_nodes_[i]["behavior"]["only_once"].as<bool>();
        double frontdist = loaded_agent_nodes_[i]["behavior"]["front_dist"].as<double>();

        btBlock_ = QString(R"(
%1
<BehaviorTree ID="ThreateningNavTree">
  <Fallback name="ThreateningFallback">
    <!-- Goal setting Sequence -->
    <Sequence name="SetGoals">
%2
    </Sequence>
    <Sequence name="ThreatNav">
      <IsRobotVisible agent_id="{id}" distance="%3"/>
      <Sequence name="ThreatTimerNav">
        <Inverter>
          <TimeExpiredCondition seconds="%4" ts="{dt}" only_once="%5"/>
        </Inverter>
        <ThreateningNav agent_id="{id}" time_step="{dt}" goal_dist="%6"/>
      </Sequence>
    </Sequence>
    <Sequence name="RegNav">
      <SetBlackboard output_key="agentid" value="{id}"/>
      <SetBlackboard output_key="timestep" value="{dt}"/>
      <SubTree ID="RegularNavTree" id="{agentid}" dt="{timestep}"/>
    </Sequence>
  </Fallback>
</BehaviorTree>
)")
                       .arg(includeCommon())
                       .arg(buildGoalSequence(i))
                       .arg(dist)
                       .arg(duration)
                       .arg(once ? "true" : "false")
                       .arg(frontdist);
      }
      break;

      default: // Regular & Impassive
        btBlock_ = QString(R"(
%1
<BehaviorTree ID="DefaultTree">
  <Fallback name="MainFallback">
    <!-- Goal setting Sequence -->
    <Sequence name="SetGoals">
%2
    </Sequence>
    <!-- Navigation loop -->
    <Sequence name="RegularNavigation">
      <Inverter>
        <IsGoalReached agent_id="{id}"/>
      </Inverter>
      <RegularNav agent_id="{id}" time_step="{dt}"/>
    </Sequence>
    <!-- Update Goal -->
    <UpdateGoal agent_id="{id}"/>
  </Fallback>
</BehaviorTree>
)")
                       .arg(includeCommon())
                       .arg(buildGoalSequence(i));
        break;
      }

      QString fullXml = modelXml.left(insertPos) + "\n" + btBlock_ + "\n" + modelXml.mid(insertPos);

      QString fname = btDir + QString("/%1__agent_%2_bt.xml")
                                  .arg(yaml_base_name_)
                                  .arg(i + 1);

      QFile out(fname);
      if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate))
      {
        QMessageBox::warning(this, "Write Error",
                             "Could not write file:\n" + fname);
        continue;
      }
      out.write(fullXml.toUtf8());
      out.close();
    }

    QString title;
    QString verbYaml;
    QString verbBT;
    if (panel_mode_ == CREATE_MODE)
    {
      title = tr("Agents YAML Saved and BTs Generated");
      verbYaml = tr("Wrote new agents YAML");
      verbBT = tr("generated");
    }
    else
    {
      title = tr("Agents YAML Updated and BTs Re-Generated");
      verbYaml = tr("Wrote updated agents YAML");
      verbBT = tr("re-generated");
    }
    QString msg = QString(
                      "<html>"
                      "%1:<br><b>%2</b><br><br>"
                      "and %3 <b>%4 BT files</b> in:<br>"
                      "%5"
                      "</html>")
                      .arg(verbYaml)
                      .arg(outName)
                      .arg(verbBT)
                      .arg(loaded_agent_names_.size())
                      .arg(btDir);

    QMessageBox::information(this, title, msg);
  }

  int ActorPanel::checkComboBox()
  {
    std::string aux = behavior_type_combobox->currentText().toStdString();

    if (aux.compare("Regular") == 0)
    {
      dur->setVisible(false);
      beh_duration->setVisible(false);
      once->setVisible(false);
      beh_once->setVisible(false);
      vel->setVisible(false);
      beh_vel->setVisible(false);
      dist->setVisible(false);
      beh_dist->setVisible(false);
      gff->setVisible(true);
      beh_gff->setVisible(true);
      off->setVisible(true);
      beh_off->setVisible(true);
      sff->setVisible(true);
      beh_sff->setVisible(true);
      other->setVisible(false);
      beh_otherff->setVisible(false);
      return hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
    }
    else if (aux.compare("Impassive") == 0)
    {
      dur->setVisible(false);
      beh_duration->setVisible(false);
      once->setVisible(false);
      beh_once->setVisible(false);
      vel->setVisible(false);
      beh_vel->setVisible(false);
      dist->setVisible(false);
      beh_dist->setVisible(false);
      gff->setVisible(true);
      beh_gff->setVisible(true);
      off->setVisible(true);
      beh_off->setVisible(true);
      sff->setVisible(true);
      beh_sff->setVisible(true);
      other->setVisible(false);
      beh_otherff->setVisible(false);
      return hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE;
    }
    else if (aux.compare("Surprised") == 0)
    {
      dur->setVisible(true);
      beh_duration->setVisible(true);
      once->setVisible(true);
      beh_once->setVisible(true);
      vel->setVisible(false);
      beh_vel->setVisible(false);
      dist->setVisible(false);
      beh_dist->setVisible(false);
      gff->setVisible(true);
      beh_gff->setVisible(true);
      off->setVisible(true);
      beh_off->setVisible(true);
      sff->setVisible(true);
      beh_sff->setVisible(true);
      other->setVisible(false);
      beh_otherff->setVisible(false);
      return hunav_msgs::msg::AgentBehavior::BEH_SURPRISED;
    }
    else if (aux.compare("Scared") == 0)
    {
      dur->setVisible(true);
      beh_duration->setVisible(true);
      once->setVisible(true);
      beh_once->setVisible(true);
      vel->setVisible(true);
      beh_vel->setVisible(true);
      dist->setVisible(false);
      beh_dist->setVisible(false);
      gff->setVisible(true);
      beh_gff->setVisible(true);
      off->setVisible(true);
      beh_off->setVisible(true);
      sff->setVisible(true);
      beh_sff->setVisible(true);
      other->setVisible(true);
      beh_otherff->setVisible(true);
      return hunav_msgs::msg::AgentBehavior::BEH_SCARED;
    }
    else if (aux.compare("Curious") == 0)
    {
      dur->setVisible(true);
      beh_duration->setVisible(true);
      once->setVisible(true);
      beh_once->setVisible(true);
      vel->setVisible(true);
      beh_vel->setVisible(true);
      dist->setVisible(true);
      beh_dist->setVisible(true);
      gff->setVisible(true);
      beh_gff->setVisible(true);
      off->setVisible(true);
      beh_off->setVisible(true);
      sff->setVisible(true);
      beh_sff->setVisible(true);
      other->setVisible(false);
      beh_otherff->setVisible(false);
      return hunav_msgs::msg::AgentBehavior::BEH_CURIOUS;
    }
    else
    {
      dur->setVisible(true);
      beh_duration->setVisible(true);
      once->setVisible(true);
      beh_once->setVisible(true);
      vel->setVisible(false);
      beh_vel->setVisible(false);
      dist->setVisible(true);
      beh_dist->setVisible(true);
      gff->setVisible(true);
      beh_gff->setVisible(true);
      off->setVisible(true);
      beh_off->setVisible(true);
      sff->setVisible(true);
      beh_sff->setVisible(true);
      other->setVisible(false);
      beh_otherff->setVisible(false);
      return hunav_msgs::msg::AgentBehavior::BEH_THREATENING;
    }
  }

  void ActorPanel::checkComboBoxConf()
  {
    int beh = checkComboBox();
    std::string conf = behavior_conf_combobox->currentText().toStdString();

    if (conf == "Default")
    {
      beh_duration->setText(QString::number(40.0));
      beh_duration->setEnabled(false);
      beh_once->setText(QString("true"));
      beh_once->setEnabled(false);
      beh_gff->setText(QString::number(2.0));
      beh_gff->setEnabled(false);
      beh_off->setText(QString::number(10.0));
      beh_off->setEnabled(false);
      beh_sff->setText(QString::number(5.0));
      beh_sff->setEnabled(false);
      beh_otherff->setText(QString::number(20.0));
      beh_otherff->setEnabled(false);
      beh_vel->setText(QString::number(1.0));
      beh_vel->setEnabled(false);
      beh_dist->setText(QString::number(10.0));
      beh_dist->setEnabled(false);

      if (beh == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED) // surprised
      {
        beh_duration->setText(QString::number(30.0));
        beh_dist->setText(QString::number(4.0));
      }
      if (beh == hunav_msgs::msg::AgentBehavior::BEH_SCARED) // scared
      {
        beh_vel->setText(QString::number(0.6));
        beh_dist->setText(QString::number(3.0));
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS) // curious
      {
        beh_vel->setText(QString::number(1.0));
        beh_dist->setText(QString::number(1.5));
      }
      else
      { // threatening
        beh_dist->setText(QString::number(1.4));
      }
    }
    else if (conf == "Custom")
    {
      beh_duration->setEnabled(true);
      beh_duration->setText("");
      beh_duration->setPlaceholderText("[10.0 - 80.0]");
      beh_duration->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

      beh_once->setEnabled(true);
      beh_once->setText("");
      beh_once->setPlaceholderText("[true or false]");
      beh_once->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

      beh_gff->setEnabled(true);
      beh_gff->setText("");
      beh_gff->setPlaceholderText("[2.0 - 5.0]");
      beh_gff->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

      beh_off->setEnabled(true);
      beh_off->setText("");
      beh_off->setPlaceholderText("[2.0 - 50.0]");
      beh_off->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

      beh_sff->setEnabled(true);
      beh_sff->setText("");
      beh_sff->setPlaceholderText("[5.0 - 20.0]");
      beh_sff->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

      beh_otherff->setEnabled(true);
      beh_otherff->setText("");
      beh_otherff->setPlaceholderText("[0.0 - 25.0]");
      beh_otherff->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

      beh_vel->setEnabled(true);
      beh_vel->setText("");
      beh_vel->setPlaceholderText("[0.4 - 1.8]");
      beh_vel->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

      beh_dist->setEnabled(true);
      beh_dist->setText("");
      beh_dist->setPlaceholderText("[0.5 - 15.0]");
      beh_dist->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");
    }

    else if (conf == "Random-normal distribution")
    {
      // Generate random values (normal distribution)
      std::random_device rd;
      std::mt19937 gen(rd());
      std::normal_distribution<> dis_gff{2.0, 1.5};
      double facGoal = dis_gff(gen);
      facGoal = (facGoal < 0.5) ? 0.5 : facGoal;
      beh_gff->setText(QString::number(facGoal));
      beh_gff->setEnabled(false);
      std::normal_distribution<> dis_off{10.0, 4.0};
      double facObstacle = dis_off(gen);
      facObstacle = (facObstacle < 0.5) ? 0.5 : facObstacle;
      beh_off->setText(QString::number(facObstacle));
      beh_off->setEnabled(false);
      std::normal_distribution<> dis_sff{4.0, 3.5};
      double facSocial = dis_sff(gen);
      facSocial = (facSocial < 3.0) ? 3.0 : facSocial;
      beh_sff->setText(QString::number(facSocial));
      beh_sff->setEnabled(false);

      std::normal_distribution<> dis_dur(40.0, 15.0);       // duration
      std::normal_distribution<> dis_vel(0.8, 0.35);        // agent max vel
      std::normal_distribution<> dis_detect_dist(4.5, 2.5); // distance to detect the robot
      double duration = dis_dur(gen);
      double vel = dis_vel(gen);

      if (beh == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS) // curious
      {
        duration = dis_dur(gen);
        beh_duration->setText(QString::number(duration));
        beh_duration->setEnabled(false);
        vel = (vel < 0.4) ? 0.4 : vel;
        beh_vel->setText(QString::number(vel));
        beh_vel->setEnabled(false);
        std::normal_distribution<> dis_dist(1.5, 0.3); // distance to get close to the robot
        beh_dist->setText(QString::number(dis_dist(gen)));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED) // surprised
      {
        duration = dis_dur(gen);
        beh_duration->setText(QString::number(duration));
        beh_duration->setEnabled(false);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_SCARED) // scared
      {
        duration = dis_dur(gen);
        beh_duration->setText(QString::number(duration));
        beh_duration->setEnabled(false);
        vel = dis_vel(gen);
        vel = (vel < 0.4) ? 0.4 : vel;
        beh_vel->setText(QString::number(vel));
        beh_vel->setEnabled(false);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
        std::normal_distribution<> dis_force(20.0, 6.0); // repulsive factor from the robot
        beh_otherff->setText(QString::number(dis_force(gen)));
        beh_otherff->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_THREATENING) // threatening
      {
        duration = dis_dur(gen);
        beh_duration->setText(QString::number(duration));
        beh_duration->setEnabled(false);
        // distance in front of the robot to put the robot goal
        std::normal_distribution<> dis_goal_dist(1.4, 0.3);
        double dist = dis_goal_dist(gen);
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
      }
    }
    else
    {
      // Generate random values (uniform distribution)
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis_gff(2.0, 5.0);
      double facGoal = dis_gff(gen);
      facGoal = (facGoal < 2.0) ? 2.0 : facGoal;
      beh_gff->setText(QString::number(facGoal));
      beh_gff->setEnabled(false);

      std::uniform_real_distribution<> dis_off(2.0, 50.0);
      double facObstacle = dis_off(gen);
      facObstacle = (facObstacle < 02.0) ? 2.0 : facObstacle;
      beh_off->setText(QString::number(facObstacle));
      beh_off->setEnabled(false);

      std::uniform_real_distribution<> dis_sff(4.0, 20.0);
      double facSocial = dis_sff(gen);
      facSocial = (facSocial < 4.0) ? 4.0 : facSocial;
      beh_sff->setText(QString::number(facSocial));
      beh_sff->setEnabled(false);

      std::uniform_real_distribution<> dis_dur(25.0, 60.0);       // duration
      std::uniform_real_distribution<> dis_vel(0.6, 1.2);         // agent max vel
      std::uniform_real_distribution<> dis_detect_dist(2.0, 6.0); // distance to detect the robot

      if (beh == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS) // curious
      {
        beh_duration->setText(QString::number(dis_dur(gen)));
        beh_duration->setEnabled(false);
        double vel = dis_vel(gen);
        vel = (vel < 0.4) ? 0.4 : vel;
        beh_vel->setText(QString::number(vel));
        beh_vel->setEnabled(false);
        std::uniform_real_distribution<> dis_dist(1.0, 2.5); // distance to get close to the robot
        beh_dist->setText(QString::number(dis_dist(gen)));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED)
      {
        beh_duration->setText(QString::number(dis_dur(gen)));
        beh_duration->setEnabled(false);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
      {
        beh_duration->setText(QString::number(dis_dur(gen)));
        beh_duration->setEnabled(false);
        double vel = dis_vel(gen);
        vel = (vel < 0.4) ? 0.4 : vel;
        beh_vel->setText(QString::number(vel));
        beh_vel->setEnabled(false);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
        std::uniform_real_distribution<> dis_force(10.0, 25.0); // repulsive factor from the robot
        beh_otherff->setText(QString::number(dis_force(gen)));
        beh_otherff->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
      {
        beh_duration->setText(QString::number(dis_dur(gen)));
        beh_duration->setEnabled(false);
        // distance in front of the robot to put the robot goal
        std::uniform_real_distribution<> dis_goal_dist(0.8, 1.9);
        beh_dist->setText(QString::number(dis_goal_dist(gen)));
        beh_dist->setEnabled(false);
      }
    }
  }

  int ActorPanel::checkComboBoxSkin()
  {
    std::string aux = skin_combobox->currentText().toStdString();

    if (simulator_combo_->currentText() != "Gazebo")
    {
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
      return 0;
    }

    if (aux.compare("Elegant man") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
      return 0;
    }
    else if (aux.compare("Casual man") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/casual_man.dae";
      return 1;
    }
    else if (aux.compare("Elegant woman") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_woman.dae";
      return 2;
    }
    else if (aux.compare("Regular man") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/regular_man.dae";
      return 3;
    }
    else if (aux.compare("Worker man") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/worker_man.dae";
      return 4;
    }
    else if (aux.compare("Blue jeans") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
      return 5;
    }
    else if (aux.compare("Green t-shirt") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
      return 6;
    }
    else if (aux.compare("Blue t-shirt") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
      return 7;
    }
    else if (aux.compare("Red t-shirt") == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
      return 8;
    }
    else
    {
      return 0;
    }
  }

  void ActorPanel::checkParserSkin(int skin)
  {
    if (skin == 0)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
    }
    else if (skin == 1)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/casual_man.dae";
    }
    else if (skin == 2)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_woman.dae";
    }
    else if (skin == 3)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/regular_man.dae";
    }
    else if (skin == 4)
    {
      person_skin = "package://hunav_rviz2_panel/meshes/worker_man.dae";
    }
    else
    {
      person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
    }
  }

  void ActorPanel::initAgentColors(int num_agents)
  {
    agent_colors_.clear();
    agent_colors_.reserve(num_agents);
    for (int i = 0; i < num_agents; ++i)
    {
      QColor c;
      c.setHsvF(double(i) / double(num_agents), 0.8, 0.9);
      agent_colors_.push_back(c);
    }
  }

  visualization_msgs::msg::Marker ActorPanel::createAgentLabel(
      double x, double y, int id, const std::string &frame_id)
  {
    visualization_msgs::msg::Marker text;
    text.header.frame_id = frame_id;
    text.header.stamp = rclcpp::Clock().now();
    text.ns = "agent_labels";
    text.id = id; // same ID as the mesh
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = x;
    text.pose.position.y = y + 0.7;
    text.pose.position.z = 1.5; // float it above the agent
    text.scale.z = 0.8;         // text height
    text.color.r = 0.0f;
    text.color.g = 0.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    text.text = std::to_string(id);
    return text;
  }

  visualization_msgs::msg::Marker ActorPanel::createMarker(double point1_x, double point1_y, double ids,
                                                           std::string marker_shape, std::string create_or_parser)
  {
    visualization_msgs::msg::Marker marker;
    uint32_t shape;
    float scale;

    if (marker_shape.compare("person") == 0)
    {
      // The variable create_or_parser is used to know from where we are calling the createMarker function
      // If the function is being called from the creation of agents, we need to check which skin is selected in the
      // combobox If the function is being called from the parser, we already know which skin it has by reading the yaml
      // file.
      if (create_or_parser.compare("create") == 0)
      {
        // Check which skin is selected
        checkComboBoxSkin();
      }

      shape = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.mesh_resource = person_skin;
      scale = 1;
      marker.pose.position.z = 0.0;
    }
    else
    {
      scale = 0.3;
      shape = visualization_msgs::msg::Marker::CUBE;
      // marker.mesh_resource = "package://hunav_rviz2_panel/meshes/ring.dae";
      marker.color.r = rgb[red];
      marker.color.g = rgb[green];
      marker.color.b = rgb[blue];
      marker.color.a = 1.0; // alpha has to be non-zero
      marker.pose.position.z = 0.5;
    }

    marker.header.frame_id = "/map";
    // marker.header.stamp = rclcpp::Node::now();
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

  visualization_msgs::msg::Marker ActorPanel::createArrowMarker(double point1_x, double point1_y, double point2_x,
                                                                double point2_y, double ids)
  {
    visualization_msgs::msg::Marker arrow_marker;

    arrow_marker.header.frame_id = "/map";
    // arrow_marker.header.stamp = rclcpp::Node::now();
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

    arrow_marker.lifetime = rclcpp::Duration(0, 0);
    arrow_marker.frame_locked = false;

    markers_array_to_remove.push_back(arrow_marker);

    return arrow_marker;
  }

  void ActorPanel::randomRGB()
  {
    red = rand() % 256;
    green = rand() % 256;
    blue = rand() % 256;
  }

  void ActorPanel::resetGoalMarkerColors()
  {
    for (auto &m : goal_markers_.markers)
    {
      if (m.ns == "goal_points")
      {
        m.color.r = 0.0f;
        m.color.g = 0.7f;
        m.color.b = 0.7f;
        m.color.a = 1.0f;
      }
      if (m.ns == "goal_numbers")
      {
        m.color.r = 1.0f;
        m.color.g = 1.0f;
        m.color.b = 1.0f;
        m.color.a = 1.0f;
      }
    }
  }

  void ActorPanel::publishAgentMarkers()
  {
    // 1) First, clear everything out
    removeCurrentMarkers();

    // 2) Build a MarkerArray on the stack
    auto arr1 = std::make_unique<visualization_msgs::msg::MarkerArray>();
    int next_id = 0;

    // 3) For each loaded agent…
    const size_t N = loaded_agent_nodes_.size();
    for (size_t i = 0; i < N; ++i)
    {
      // read its stored init‐pose
      const YAML::Node &node = loaded_agent_nodes_[i];
      double ipx = node["init_pose"]["x"].as<double>();
      double ipy = node["init_pose"]["y"].as<double>();

      // pull out the saved yaw ("h") if present
      double yaw = 0.0;
      if (node["init_pose"]["h"])
        yaw = node["init_pose"]["h"].as<double>();

      // pick a single id for *both* mesh+text
      int this_id = next_id++;
      loaded_initial_marker_ids_[i] = this_id;

      // stamp
      std_msgs::msg::Header hdr;
      hdr.frame_id = "/map";
      hdr.stamp = this->now();

      // pick its color
      const QColor &c = agent_colors_[i];
      std_msgs::msg::ColorRGBA col;
      col.r = c.redF();
      col.g = c.greenF();
      col.b = c.blueF();
      col.a = 1.0f;

      // — mesh/person marker —
      auto mesh = createMarker(ipx, ipy, this_id, "person", "parser");
      mesh.header = hdr;
      mesh.ns = "agent_initial";
      mesh.id = this_id;
      mesh.color = col;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      mesh.pose.orientation = tf2::toMsg(q);

      arr1->markers.push_back(mesh);

      // — floating text label —
      auto text = mesh; // copy header, pose, etc
      text.ns = "agent_id_text";
      text.id = this_id;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.text = std::to_string(int(i + 1));
      text.scale.z = 0.8;
      text.pose.position.y += 0.7;
      text.pose.position.z += 1.5;
      text.color = col;
      arr1->markers.push_back(text);
    }
    next_marker_id_ = next_id;

    // 4) Publish all at once
    initial_pose_publisher->publish(std::move(arr1));
  }

  void ActorPanel::clearDisplayedMap()
  {
    nav_msgs::msg::OccupancyGrid blank;
    blank.header.frame_id = "map";
    blank.header.stamp = this->now();

    // a single unknown cell
    blank.info.resolution = 1.0;
    blank.info.width = 1;
    blank.info.height = 1;
    // origin somewhere offscreen
    blank.info.origin.position.x = 9999.0;
    blank.info.origin.position.y = 9999.0;
    blank.info.origin.orientation.w = 1.0;

    // mark it unknown
    blank.data.assign(1, -1);

    map_pub_->publish(blank);
  }

  void ActorPanel::clearNonAgentMarkers()
  {
    static const std::vector<std::string> bad_ns = {
        "goal_points",
        "goal_numbers",
        "agent_goal",
        "agent_arrow",
        "goal_cubes",
        "goal_labels"};

    for (const auto &ns : bad_ns)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "/map";
      m.header.stamp = rclcpp::Clock().now();
      m.ns = ns;
      m.action = visualization_msgs::msg::Marker::DELETEALL;

      auto arr = std::make_unique<visualization_msgs::msg::MarkerArray>();
      arr->markers.push_back(m);

      // publish on both topics
      initial_pose_publisher->publish(std::move(arr));

      auto arr2 = std::make_unique<visualization_msgs::msg::MarkerArray>();
      arr2->markers.push_back(m);
      goal_markers_pub_->publish(std::move(arr2));
    }
  }

  void ActorPanel::removeCurrentMarkers()
  {
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
    goal_markers_pub_->publish(std::move(marker_array1));
  }

  std::string ActorPanel::openFileExplorer(bool file)
  {
    QString fileName;

    if (file)
    {
      fileName = QFileDialog::getOpenFileName(this, tr("Open file"), "/home", tr("YAML Files (*.yaml)"));
      show_file_selector_once = true;
      checkbox->setChecked(true);
    }
    else
    {
      fileName = QFileDialog::getExistingDirectory(this, tr("Open folder"), "/home", QFileDialog::ShowDirsOnly);
      window->activateWindow();
    }

    dir = fileName.toStdString();
    return dir;
  }

  void ActorPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("Topic", output_topic_);
  }

  void ActorPanel::resetPanel()
  {
    // 1) Clear all RViz markers
    removeCurrentMarkers();
    clearDisplayedMap();
    goal_markers_.markers.clear();
    initial_pose_marker_array.markers.clear();

    // 1a) Create/Load buttons
    create_button_->setChecked(false);
    open_button_->setChecked(false);

    // 2) Clear loaded data
    loaded_agent_nodes_.clear();
    loaded_agent_names_.clear();
    loaded_agent_goals_.clear();
    loaded_global_goals_.clear();
    loaded_initial_marker_ids_.clear();
    goal_list_widget_->clear();
    goal_ids_.clear();
    agent_goals_.clear();
    actors_info_.clear();
    next_marker_id_ = 0;

    // 3) Reset mode & indices
    panel_mode_ = CREATE_MODE;
    current_edit_idx_ = 0;
    iterate_actors_ = 1;
    agent_count = 1;

    // 4) Reset UI back to “fresh” state
    map_group->setEnabled(false);
    actors->clear();
    actors->show();
    actor_button_->setText(tr("Generate agents"));
    actor_button_->setEnabled(false);
    n_agents_label_->setEnabled(false);

    edit_goals_button_->hide();
    save_bt_btn_->hide();
    reset_goals_button_->hide();

    yaml_file_label_->hide();
    simulator_combo_->setCurrentIndex(-1);
    map_select_btn_->show();
    map_select_btn_->setVisible(true);
    map_select_btn_->setEnabled(false);
    current_map_label_->clear();
    goal_group_->setTitle("Define agents goals");
    goal_group_->setEnabled(false);
    checkbox->setEnabled(false);

    QMessageBox::information(this, tr("Reset"),
                             tr("All settings cleared.\nPanel is back to its initial state."));
  }

  /**
   * @brief Load configuration data for this panel from the given Config object
   *
   * This method is called by RViz2 when loading saved configurations.
   * Currently loads base panel configuration but custom panel settings
   * can be added here as needed.
   *
   * @param config RViz2 configuration object containing saved settings
   */
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

} // namespace hunav_rviz2_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hunav_rviz2_panel::ActorPanel, rviz_common::Panel)
