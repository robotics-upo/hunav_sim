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
#include <QProgressDialog>
#include <QMessageBox>
#include <QProcess>
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
#include "headers/BTConfigDialog.h"

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
    // ─── Main Layout ───
    QVBoxLayout *main_layout = new QVBoxLayout;
    main_layout->setContentsMargins(16, 16, 16, 16);

    // ─── Header Section ───
    QLabel *header_label = new QLabel("<h2>Agent Configuration Manager</h2>");
    header_label->setAlignment(Qt::AlignCenter);
    header_label->setStyleSheet(
        "QLabel {"
        "  color: #2c3e50;"
        "  padding: 1px;"
        "  background-color: #ecf0f1;"
        "  border-radius: 6px;"
        "}");
    main_layout->addWidget(header_label);

    QLabel *subtitle = new QLabel("Create new agent configurations or edit existing ones");
    subtitle->setAlignment(Qt::AlignCenter);
    subtitle->setStyleSheet(
        "QLabel {"
        "  color: #7f8c8d;"
        "  font-style: italic;"
        "}");
    main_layout->addWidget(subtitle);

    // ─── Mode Selection Group ───
    QGroupBox *mode_group = new QGroupBox("Configuration Mode");
    mode_group->setStyleSheet(
        "QGroupBox {"
        "  font-size: 16px;"
        "  font-weight: bold;"
        "  color:rgb(0, 0, 0);"
        "  margin-top: 8px;"
        "  padding-top: 4px;"
        "  border: 2px solid #bdc3c7;"
        "  border-color: rgb(34, 103, 167);"
        "  border-radius: 8px;"
        "  background-color:rgb(230, 239, 248);"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  subcontrol-position: top left;"
        "  padding: 0 4px;"
        "  background-color:rgb(176, 231, 245);"
        "  border: 1px rgb(74, 132, 170);"
        "  border-width: 1px;"
        "  border-style: solid;"
        "  border-color: rgb(74, 132, 170);"
        "  border-radius: 4px;"
        "}");

    QVBoxLayout *mode_layout = new QVBoxLayout;
    mode_layout->setSpacing(5);

    create_button_ = new QPushButton("Create New Configuration", this);
    open_button_ = new QPushButton("Edit Existing Configuration", this);

    // Style the mode buttons
    QString button_style =
        "QPushButton {"
        "  padding: 3px 3px;"
        "  border: 2px solid #3498db;"
        "  border-radius: 6px;"
        "  background-color: #ecf0f1;"
        // "  color: #2c3e50;"
        "  font-weight: bold;"
        "  text-align: center;"
        "}"
        "QPushButton:hover {"
        "  background-color: #d5eaf8;"
        // "  color: #2980b9;"
        "  border: 2px solid #2980b9;"
        "  border-color: #2980b9;"
        "}"
        "QPushButton:checked {"
        "  background-color: #3498db;"
        "  color: white;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #2980b9;"
        "}";

    create_button_->setStyleSheet(button_style);
    open_button_->setStyleSheet(button_style);
    create_button_->setCheckable(true);
    open_button_->setCheckable(true);

    mode_layout->addWidget(create_button_);
    mode_layout->addWidget(open_button_);

    yaml_file_label_ = new QLabel("", this);
    yaml_file_label_->setStyleSheet(
        "QLabel {"
        "  font-style: italic;"
        "  color: #27ae60;"
        "  background-color: #d5f4e6;"
        "  padding: 4px;"
        "  border-radius: 4px;"
        "  border: 1px solid #a9dfbf;"
        "}");
    yaml_file_label_->setAlignment(Qt::AlignCenter);
    yaml_file_label_->hide();
    mode_layout->addWidget(yaml_file_label_);

    mode_group->setLayout(mode_layout);
    main_layout->addWidget(mode_group);

    // ─── Simulator + Map Selection ───

    map_group = new QGroupBox("Simulation Environment", this);
    map_group->setStyleSheet(
        "QGroupBox {"
        "  font-size: 16px;"
        "  font-weight: bold;"
        "  color:rgb(0, 0, 0);"
        "  margin-top: 8px;"
        "  padding-top: 4px;"
        "  border: 2px solid #e74c3c;"
        "  border-radius: 8px;"
        "  background-color: #fdf2f2;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  subcontrol-position: top left;"
        "  padding: 0 4px;"
        "  background-color: #fadbd8;"
        "  border: 1px solid #e74c3c;"
        "  border-radius: 4px;"
        "}");

    QVBoxLayout *map_layout_v = new QVBoxLayout;
    map_layout_v->setSpacing(2);

    // Simulator selection
    QLabel *sim_label = new QLabel("Select Simulator:");
    sim_label->setStyleSheet("font-weight: bold; color: #2c3e50;");
    map_layout_v->addWidget(sim_label);

    simulator_combo_ = new QComboBox;
    simulator_combo_->addItem("Gazebo Classic", 1.25);
    simulator_combo_->addItem("Gazebo Fortress", 1.25);
    simulator_combo_->addItem("Isaac Sim", 0.0);
    simulator_combo_->addItem("Webots", 0.01);
    simulator_combo_->setCurrentIndex(-1);
    simulator_combo_->setStyleSheet(
        "QComboBox {"
        "  padding: 4px 8px;"
        "  border: 2px solid #bdc3c7;"
        "  border-radius: 6px;"
        "  font-size: 14px;"
        "}"
        "QComboBox:hover {"
        "  background-color: rgba(247, 214, 204, 0.8);"
        "  border: 2px solid #e74c3c;"
        "  border-color: #e74c3c;"
        "}");
    map_layout_v->addWidget(simulator_combo_);

    // Map selection
    QLabel *map_label = new QLabel("Select Map:");
    map_label->setStyleSheet("font-weight: bold; color: #2c3e50; margin-top: 8px;");
    map_layout_v->addWidget(map_label);

    QHBoxLayout *map_select_layout = new QHBoxLayout;
    map_select_btn_ = new QPushButton("Browse Maps", this);
    map_select_btn_->setEnabled(false);
    map_select_btn_->setStyleSheet(
        "QPushButton {"
        "  padding: 4px 8px;"
        "  border: 2px solid #e74c3c;"
        "  border-radius: 6px;"
        "  background-color:rgb(254, 236, 231);"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color:rgba(247, 214, 204, 0.8);"
        "}"
        "QPushButton:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "  border-color: #d5dbdb;"
        "}");

    current_map_label_ = new QLabel("No map selected", this);
    current_map_label_->setStyleSheet(
        "QLabel {"
        "  padding: 4px 8px;"
        "  border: 1px solid #d5dbdb;"
        "  border-radius: 4px;"
        "  background-color: #f8f9fa;"
        "  color: #5d6d7e;"
        "  font-style: italic;"
        "}");
    current_map_label_->setMinimumWidth(100);

    map_select_layout->addWidget(map_select_btn_);
    map_select_layout->addWidget(current_map_label_, 1);
    map_layout_v->addLayout(map_select_layout);

    map_group->setLayout(map_layout_v);
    map_group->setEnabled(false);
    main_layout->addWidget(map_group);

    // Only enable “Select map” once a simulator is picked:
    connect(simulator_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx)
            { map_select_btn_->setEnabled(idx >= 0);
              if (panel_mode_ == EDIT_MODE)
                actor_button_->setEnabled(true); });

    connect(map_select_btn_, &QPushButton::clicked,
            this, &ActorPanel::onSelectMap);

    // ─── Agent Configuration Section ───
    QGroupBox *agent_config_group = new QGroupBox("Agent Configuration");
    agent_config_group->setStyleSheet(
        "QGroupBox {"
        "  font-weight: bold;"
        "  font-size: 16px;"
        "  color:rgb(0, 0, 0);"
        "  margin-top: 8px;"
        "  padding-top: 4px;"
        "  border: 2px solid #8e44ad;"
        "  border-radius: 8px;"
        "  background-color: #f4f1f8;"
        "  align-items: center;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  subcontrol-position: top left;"
        "  padding: 0 4px;"
        "  background-color: #e8daef;"
        "  border: 1px solid #8e44ad;"
        "  border-radius: 4px;"
        "}");

    QVBoxLayout *agent_config_layout = new QVBoxLayout;
    agent_config_layout->setSpacing(2);

    // Number of agents (CREATE mode only)
    n_agents_label_ = new QLabel("Number of agents to generate:");
    n_agents_label_->setStyleSheet("font-weight: bold; color: #2c3e50;");
    n_agents_label_->setEnabled(false);
    agent_config_layout->addWidget(n_agents_label_);

    actors = new QLineEdit(this);
    actors->setEnabled(false);
    actors->setPlaceholderText("Enter number of agents (e.g., 5)");
    actors->setStyleSheet(
        "QLineEdit {"
        "  padding: 4px 8px;"
        "  border: 2px solid #bdc3c7;"
        "  border-radius: 6px;"
        "  background-color: white;"
        "  font-size: 14px;"
        "}"
        "QLineEdit:focus {"
        "  border-color: #3498db;"
        "}"
        "QLineEdit:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "}");

    actors->setToolTip(
        "<html><b>Number of Agents</b><br>"
        "Total number of human agents to create for this scenario.<br>"
        "<b>Range:</b> 1 - 50 agents<br>"
        "<b>Note:</b> More agents increase computational load.<br>"
        "<b>Recommended:</b> Start with 3-10 agents for testing</html>");
    agent_config_layout->addWidget(actors);

    // Action buttons
    create_mode_layout_ = new QVBoxLayout;
    edit_mode_layout_ = new QHBoxLayout;

    // QHBoxLayout *action_buttons_layout = new QHBoxLayout;
    // action_buttons_layout->setSpacing(2);

    actor_button_ = new QPushButton("Generate Agents", this);
    actor_button_->setCheckable(true);
    actor_button_->setEnabled(false);

    add_agent_button_ = new QPushButton("Add New Agent", this);
    add_agent_button_->setEnabled(false);
    add_agent_button_->setCheckable(true);
    add_agent_button_->hide();

    edit_goals_button_ = new QPushButton("Edit Goals", this);
    edit_goals_button_->setCheckable(true);
    edit_goals_button_->hide();

    QString action_button_style =
        "QPushButton {"
        "  padding: 5px 8px;"
        "  border: 2px solid #27ae60;"
        "  border-radius: 6px;"
        "  background-color: #d5f4e6;"
        "  color: #1e8449;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color: #a9dfbf;"
        "  color: #1e8449;"
        "  border: 2px solid #1e8449;"
        "  border-color: #1e8449;"
        "}"
        "QPushButton:checked {"
        "  background-color: #27ae60;"
        "  color: white;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "  border-color: #d5dbdb;"
        "}";

    QString actor_create_button_style =
        "QPushButton {"
        "  padding: 5px 8px;"
        "  border: 2px solid #8e44ad;"
        "  border-radius: 6px;"
        "  background-color:rgb(238, 231, 248);"
        "  color:rgb(97, 46, 119);"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover::enabled {"
        "  background-color:rgb(238, 222, 248);"
        "  color:rgb(97, 46, 119);"
        "  border-color: #8e44ad;"
        "}"
        "QPushButton:pressed {"
        "  background-color:rgb(100, 48, 122);"
        "  color: white;"
        "}"
        "QPushButton:checked {"
        "  background-color: #8e44ad;"
        "  color: white;"
        "  font-weight: bold;"
        "  font-size: 13px;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "  border-color: #d5dbdb;"
        "}";

    // Apply initial styles
    actor_button_->setStyleSheet(actor_create_button_style);
    add_agent_button_->setStyleSheet(actor_create_button_style);
    edit_goals_button_->setStyleSheet(actor_create_button_style);

    create_mode_layout_->addWidget(actor_button_);

    // action_buttons_layout->addWidget(actor_button_);
    // action_buttons_layout->addWidget(add_agent_button_);
    // action_buttons_layout->addWidget(edit_goals_button_);
    // action_buttons_layout->addStretch();

    edit_mode_layout_->addWidget(actor_button_);
    edit_mode_layout_->addWidget(add_agent_button_);
    edit_mode_layout_->addWidget(edit_goals_button_);
    edit_mode_layout_->setSpacing(9);

    // agent_config_layout->addLayout(action_buttons_layout);
    agent_config_layout->addLayout(create_mode_layout_);

    QWidget *edit_mode_widget = new QWidget;
    edit_mode_widget->setLayout(edit_mode_layout_);
    edit_mode_widget->hide();
    agent_config_layout->addWidget(edit_mode_widget);

    edit_mode_widget_ = edit_mode_widget;

    agent_config_group->setLayout(agent_config_layout);
    main_layout->addWidget(agent_config_group);

    connect(add_agent_button_, &QPushButton::clicked, this, [this](bool checked)
            {
        // Toggle the button state
        add_agent_button_->setDown(checked);
        
        // Call the existing onAddAgent method
        onAddAgent(); });

    connect(edit_goals_button_, &QPushButton::clicked, this, [this](bool checked)
            {
        // Toggle the button state
        edit_goals_button_->setDown(checked);

        // toggle pick‐mode flag
        goal_picking_mode_ = !goal_picking_mode_;

        // reset markers & enable them
        publishAgentMarkers();
        goal_markers_pub_->publish(goal_markers_);

        goal_group_->setEnabled(true);

        if (goal_picking_mode_)
        {
          // Entering goal picking mode - disable assign goals
          assign_goals_btn_->setEnabled(false);
        }
        else
        {
          // Exiting goal picking mode - enable assign goals if goals exist
          assign_goals_btn_->setEnabled(!loaded_global_goals_.empty());
        }

        // the very first time only: show HTML instructions
        if (goal_picking_mode_ && !first_goal_picking_info_shown_) {
          first_goal_picking_info_shown_ = true;
          QString msg = QString(R"(
            <html>
              Click on the map to <b><i>add or edit</i></b> navigation goals.<br>
              To <b>edit</b>, just <b>click on the goal marker</b> you wish to modify.<br><br>
              When you’re done, click on <b><i>%1</i></b> again to exit goal-picking mode,<br>
              then click on <b><i>%2</i></b> to assign your changes or <b><i>%3</i></b> to save the file.
            </html>
          )")
            .arg(edit_goals_button_->text())
            .arg(assign_goals_btn_->text())
            .arg(save_bt_btn_->text());
          QMessageBox::information(this, tr("Add/Edit Goals"), msg);
        }

        // ── Tool management - switch tools based on mode ──
        if (auto *tm = getDisplayContext()->getToolManager()) 
          {
            if (goal_picking_mode_) 
            {
              // Entering goal picking mode - switch to PublishPoint tool
              for (int i = 0; i < tm->numTools(); ++i) {
                auto *tool = tm->getTool(i);
                if (QString(tool->getClassId()) == "rviz_default_plugins/PublishPoint") {
                  tm->setCurrentTool(tool);
                  break;
                }
              }
              // edit_goals_button_->setDown(true);
            } 
            else 
            {
              // exiting goal picking mode - switch to Interact tool
              for (int i = 0; i < tm->numTools(); ++i) {
                auto *tool = tm->getTool(i);
                if (QString(tool->getClassId()) == "rviz_default_plugins/Interact") {
                  tm->setCurrentTool(tool);
                  break;
                }
              }
              // edit_goals_button_->setDown(false);
              save_bt_btn_->setEnabled(true);
            }
          } });

    // checkbox = new QCheckBox("Use default directory", this);
    // checkbox->setChecked(true);
    // checkbox->setEnabled(false);

    connect(create_button_, &QPushButton::clicked, this, [this]()
            {
        create_button_->setDown(true);
        open_button_->setDown(false);
        create_button_->setChecked(true);
        open_button_->setChecked(false);

        switchButtonLayout(CREATE_MODE);

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
        actors->show();                  // the “# of agents” line-edit
        n_agents_label_->show();
        actor_button_->setText("Generate Agents");
        actor_button_->setEnabled(false);
        simulator_combo_->setCurrentIndex(-1);

        // Hide all the EDIT_MODE widgets
        edit_goals_button_->hide();
        reset_goals_button_->hide();
        yaml_file_label_->hide();

        // Put map/group boxes back to the CREATE titles & states
        // map_group->setTitle("Select simulator and map:");
        map_group->setEnabled(true);
        map_group->setVisible(true);
        map_select_btn_->show();
        map_select_btn_->setVisible(true);
        current_map_label_->show();
        // goal_group_->setTitle("Define agents goals");
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
    connect(actor_button_, &QPushButton::clicked, this, [this](bool checked)
            {
              // Toggle the button state
              actor_button_->setDown(checked);

              onCreateOrEditAgents(); });

    connect(open_button_, &QPushButton::clicked, this, [this]()
            {
              // 1) Ask which simulator we’re editing for
              QStringList sims = { "Gazebo Classic", "Gazebo Fortress", "Isaac Sim", "Webots" };
              QInputDialog dlg(this);
              dlg.setWindowTitle(tr("Select Simulator"));
              dlg.setLabelText(tr("Which simulator are you using?"));
              dlg.setComboBoxItems(sims);
              dlg.setOption(QInputDialog::UseListViewForComboBoxItems);  
              dlg.setTextValue(simulator_combo_->currentText());
              
              dlg.setStyleSheet(R"(
                QInputDialog {
                  background-color: #f1f4f8ff;
                  border: 2px solid #445cadff;
                  border-radius: 8px;
                }
                QInputDialog QLabel {
                  color: #2c3e50;
                  font-size: 14px;
                }
                QComboBox {
                  padding: 6px 10px;
                  border: 2px solid #4457adff;
                  border-radius: 6px;
                  background-color: white;
                  font-size: 14px;
                }
                QComboBox::drop-down {
                  subcontrol-origin: padding;
                  subcontrol-position: top right;
                  width: 20px;
                  border-left: none;
                }
                QComboBox QAbstractItemView {
                  background: #ffffff;
                  selection-background-color: #daddefff;
                  font-size: 13px;
                }
                QDialogButtonBox QPushButton {
                  padding: 6px 12px;
                  border: 2px solid #445cadff;
                  border-radius: 6px;
                  background-color: #dae2efff;
                  font-weight: bold;
                }
                QDialogButtonBox QPushButton:hover {
                  background-color: #c3c9e8ff;
                }
                QDialogButtonBox QPushButton:disabled {
                  background-color: #f8f9fa;
                  color: #aeb6bf;
                  border-color: #d5dbdb;
                }
              )");
              
              if (dlg.exec() == QDialog::Accepted) {
                QString sim = dlg.textValue();
                simulator_combo_->setCurrentText(sim);
                open_button_->setDown(true);
                create_button_->setDown(false);
                open_button_->setChecked(true);
                create_button_->setChecked(false);
                parseYaml();
                bt_group_->setEnabled(true);
              } });

    // ─── Goal Management Section ───
    goal_group_ = new QGroupBox("Navigation Goals");
    goal_group_->setStyleSheet(
        "QGroupBox {"
        "  font-size: 16px;"
        "  font-weight: bold;"
        "  color:rgb(0, 0, 0);"
        "  margin-top: 8px;"
        "  padding-top: 4px;"
        "  border: 2px solid #e67e22;"
        "  border-radius: 8px;"
        "  background-color: #fdf2e6;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  subcontrol-position: top left;"
        "  padding: 0 4px;"
        "  background-color: #f8c471;"
        "  border: 1px solid #e67e22;"
        "  border-radius: 4px;"
        "}");
    goal_group_->setEnabled(panel_mode_ == EDIT_MODE);

    QVBoxLayout *goal_layout = new QVBoxLayout;
    goal_layout->setSpacing(2);

    // Goal picking mode button
    enter_goal_mode_btn_ = new QPushButton("Enter Goal-Picking Mode");
    enter_goal_mode_btn_->setCheckable(true);
    enter_goal_mode_btn_->setStyleSheet(
        "QPushButton {"
        "  padding: 5px 8px;"
        "  border: 2px solid #e67e22;"
        "  border-radius: 6px;"
        "  background-color: #fef5e7;"
        // "  color: #d68910;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color:rgb(243, 222, 207);"
        "  border-color: #d68910;"
        "}"
        "QPushButton:checked {"
        "  background-color: #e67e22;"
        "  color: white;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "  border-color: #d5dbdb;"
        "}");
    goal_layout->addWidget(enter_goal_mode_btn_);

    connect(enter_goal_mode_btn_, &QPushButton::clicked,
            this, &ActorPanel::onEnterGoalPickingMode);
    // Reset goals button (EDIT mode only)
    reset_goals_button_ = new QPushButton("Reset All Goals", this);
    reset_goals_button_->setEnabled(true);
    reset_goals_button_->hide();
    reset_goals_button_->setStyleSheet(
        "QPushButton {"
        "  padding: 4px 8px;"
        "  border: 2px solid #e74c3c;"
        "  border-radius: 6px;"
        "  background-color: #fdf2f2;"
        "  color: #c0392b;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #fcebea;"
        "  color: #c0392b;"
        "  border-color: #c0392b;"
        "}");
    goal_layout->addWidget(reset_goals_button_);

    connect(reset_goals_button_, &QPushButton::clicked,
            this, &ActorPanel::onResetLoadedGoals);

    // Goals list with better styling
    QLabel *goals_list_label = new QLabel("Current Goals:");
    goals_list_label->setStyleSheet("font-weight: bold; color: #2c3e50; margin-top: 8px;");
    goal_layout->addWidget(goals_list_label);

    goal_list_widget_ = new QListWidget;
    goal_list_widget_->setStyleSheet(
        "QListWidget {"
        "  border: 2px solid #bdc3c7;"
        "  border-radius: 6px;"
        "  background-color: white;"
        "  padding: 4px;"
        "  font-size: 13px;"
        "}"
        "QListWidget::item {"
        "  padding: 6px 8px;"
        "  border-bottom: 1px solid #ecf0f1;"
        "}"
        "QListWidget::item:hover {"
        "}"
        "QListWidget::item:selected {"
        "  background-color: #3498db;"
        "  color: white;"
        "}");
    goal_list_widget_->setMaximumHeight(80);
    goal_layout->addWidget(goal_list_widget_);

    // Assign goals button
    assign_goals_btn_ = new QPushButton("Assign Goals to Agents");
    assign_goals_btn_->setEnabled(false);
    assign_goals_btn_->setCheckable(true);
    assign_goals_btn_->setStyleSheet(
        "QPushButton {"
        "  padding: 5px 8px;"
        "  border: 2px solid #e67e22;"
        "  border-radius: 6px;"
        "  background-color: #fef5e7;"
        // "  color: #1f4e79;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color:rgb(243, 222, 207);"
        "  border-color: #d68910;"
        "}"
        "QPushButton:checked {"
        "  background-color: #e67e22;"
        "  color: white;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "  border-color: #d5dbdb;"
        "}");
    goal_layout->addWidget(assign_goals_btn_);

    connect(assign_goals_btn_, &QPushButton::clicked, this, [this](bool checked)
            {
      // Toggle the button state
      assign_goals_btn_->setDown(checked);
      onAssignGoalsClicked(); 
      bt_group_->setEnabled(true);});

    // Summary area for assigned goals
    summary_area_ = new QVBoxLayout;
    goal_layout->addLayout(summary_area_);

    goal_group_->setLayout(goal_layout);
    main_layout->addWidget(goal_group_);

    QGroupBox *output_group = new QGroupBox("Save/Export");
    output_group->setStyleSheet(
        "QGroupBox {"
        "  font-size: 16px;"
        "  font-weight: bold;"
        "  color:rgb(0, 0, 0);"
        "  margin-top: 8px;"
        "  padding-top: 4px;"
        "  border: 2px solid #16a085;"
        "  border-radius: 8px;"
        "  background-color: #e8f8f5;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  subcontrol-position: top left;"
        "  padding: 0 4px;"
        "  background-color: #a3e4d7;"
        "  border: 1px solid #16a085;"
        "  border-radius: 4px;"
        "}");

    QVBoxLayout *output_layout = new QVBoxLayout;
    output_layout->setSpacing(2);

    // Default directory checkbox with better styling
    checkbox = new QCheckBox("Use default output directory", this);
    checkbox->setChecked(true);
    checkbox->setEnabled(false);
    checkbox->setStyleSheet(
        "QCheckBox {"
        "  color: #2c3e50;"
        "  font-weight: bold;"
        "}"
        "QCheckBox::indicator {"
        "  width: 18px;"
        "  height: 18px;"
        "  border: 2px solid #bdc3c7;"
        "  border-radius: 4px;"
        "  background-color: white;"
        "}"
        "QCheckBox::indicator:checked {"
        "  background-color: #27ae60;"
        "  border-color: #27ae60;"
        "}"
        "QCheckBox::indicator:disabled {"
        "  background-color: #f8f9fa;"
        "  border-color: #d5dbdb;"
        "}");
    output_layout->addWidget(checkbox);

    // Save button with enhanced styling
    save_bt_btn_ = new QPushButton("Save Configuration - Generate BT Files");
    save_bt_btn_->setEnabled(false);
    save_bt_btn_->setStyleSheet(
        "QPushButton {"
        "  padding: 6px 10px;"
        "  border: 2px solid #27ae60;"
        "  border-radius: 8px;"
        "  background-color: #d5f4e6;"
        // "  color: #1e8449;"
        "  font-weight: bold;"
        "  font-size: 15px;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color: #a9dfbf;"
        "  color: #1e8449;"
        "  border-color: #1e8449;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #27ae60;"
        "  color: white;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "  border-color: #d5dbdb;"
        "}");
    output_layout->addWidget(save_bt_btn_);

    connect(save_bt_btn_, &QPushButton::clicked, this, [this](bool checked)
            {
        // Toggle the button state
        save_bt_btn_->setDown(checked);
        saveAndGenerateAll(); });

    output_group->setLayout(output_layout);
    main_layout->addWidget(output_group);

    // ─── Behavior Tree Management ───
    bt_group_ = new QGroupBox("Behavior Tree Management");
    bt_group_->setStyleSheet(
        "QGroupBox {"
        "  font-size: 16px;"
        "  font-weight: bold;"
        "  color:rgb(0, 0, 0);"
        "  margin-top: 8px;"
        "  padding-top: 4px;"
        "  border: 2px solid #9b59b6;"
        "  border-radius: 8px;"
        "  background-color: #f4f1f8;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  subcontrol-position: top left;"
        "  padding: 0 4px;"
        "  background-color: #e8daef;"
        "  border: 1px solid #9b59b6;"
        "  border-radius: 4px;"
        "}");
    bt_group_->setEnabled(false);

    QVBoxLayout *bt_layout = new QVBoxLayout;
    bt_layout->setSpacing(8);

    // Small hint for the user to hover over buttons
    statusLabel_ = new QLabel("Hover over buttons for more info", bt_group_);
    statusLabel_->setStyleSheet(
        "QLabel {"
        "  color: #2c3e50;"
        "  font-style: italic;"
        "  font-size: 12px;"
        "  padding: 4px;"
        "  background-color: #ecf0f1;"
        "  border-radius: 4px;"
        "  border: 1px solid #bdc3c7;"
        "}");
    bt_layout->addWidget(statusLabel_); 

    currentXmlPath_ = "BTRegularNav.xml";
    agentBTConfig_.clear();

    // Wizard configuration button
    auto cfg_bt_btn = new QPushButton("Configure Agent Behaviors");
    cfg_bt_btn->setToolTip(
        "<html><b>Behavior Tree Configurator</b><br>"
        "Launch a step-by-step configurator to customize agent behavior trees with predefined behavior blocks.<br>"
        "This helps you set up behaviors without needing to edit XML files directly.</html>");
    cfg_bt_btn->setStyleSheet(
        "QPushButton {"
        "  padding: 5px 8px;"
        "  border: 2px solid #9b59b6;"
        "  border-radius: 6px;"
        "  background-color: #f4f1f8;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  border-color: #7d3c98;"
        "  background-color: #e8daef;"
        "  color: #6c3483;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #9b59b6;"
        "  color: white;"
        "}");

    connect(cfg_bt_btn, &QPushButton::clicked, this, &ActorPanel::launchBTWizard);

    bt_layout->addWidget(cfg_bt_btn);

    // Reset customizations button
    auto resetBTBtn = new QPushButton("Reset All Customizations");
    resetBTBtn->setStyleSheet(
        "QPushButton {"
        "  padding: 4px 8px;"
        "  border: 2px solid #e74c3c;"
        "  border-radius: 6px;"
        "  background-color: #fdf2f2;"
        // "  color: #c0392b;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #fcebea;"
        "  border-color: #c0392b;"
        "}");

    resetBTBtn->setToolTip(
        "<html><b>Reset Behavior Tree Customizations</b><br>"
        "This will revert all agents to their default behavior templates.<br>"
        "<b>Note:</b> This action cannot be undone.</html>");

    connect(resetBTBtn, &QPushButton::clicked, this, [this]()
            {
        // Check if agents are loaded
        if (loaded_agent_names_.empty()) {
            QMessageBox::information(this, "No Agents", 
                "No agents are currently loaded. Please load or create agents first.");
            return;
        }

        // Check if any BT files exist for the current scenario
        QString scenarioName = panel_mode_ == EDIT_MODE ? orig_yaml_base_name_ : yaml_base_name_;
        QString simulator = simulator_combo_->currentText();
        
        if (scenarioName.isEmpty()) {
            QMessageBox::information(this, "No Configuration", 
                "No scenario configuration is loaded. Save your configuration first.");
            return;
        }
        
        // Generate BT file paths to check if they exist
        QStringList btPaths = BTConfigDialog::generateBTPathsForScenario(
            scenarioName, simulator, loaded_agent_names_.size());
        
        // Check if at least one BT file exists
        bool anyBTExists = false;
        for (const QString& path : btPaths) {
            if (QFile::exists(path)) {
                anyBTExists = true;
                break;
            }
        }
        
        if (!anyBTExists) {
            QMessageBox::information(this, "No BT Files", 
                "No behavior tree files found for this scenario.\n\n"
                "BT files are created when you save the configuration or use the BT wizard.");
            return;
        }


        resetBTConfiguration(btPaths);
        RCLCPP_INFO(this->get_logger(), "Reset all BT customizations");
        });


    // Info label
    QLabel *bt_info = new QLabel("Use the option above for guided BT editing, or \nlaunch Groot2 for visual editing");
    bt_info->setStyleSheet(
        "QLabel {"
        "  color: #5d6d7e;"
        "  font-style: italic;"
        "  padding: 4px;"
        "  background-color: #f8f9fa;"
        "  border-radius: 4px;"
        "  border: 1px solid #d5dbdb;"
        "}");
    // bt_layout->addWidget(bt_info); // (currently not used)

    // Add button to launch LLM-based BT generator
    auto llm_bt_btn_ = new QPushButton("Generate Behaviors with LLM");
    llm_bt_btn_->setToolTip(
        "<html><b>LLM-based Behavior Tree Generator</b><br>"
        "Leverage a Large Language Model to automatically generate behavior trees based on high-level descriptions.</html>");
    llm_bt_btn_->setStyleSheet(
        "QPushButton {"
        "  padding: 5px 8px;"
        "  border: 2px solid #9b59b6;"
        "  border-radius: 6px;"
        "  background-color: #f4f1f8;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  border-color: #7d3c98;"
        "  background-color: #e8daef;"
        "  color: #6c3483;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #9b59b6;"
        "  color: white;"
        "}"); 
    bt_layout->addWidget(llm_bt_btn_);
    connect(llm_bt_btn_, &QPushButton::clicked, this, &ActorPanel::launchLLMBTGenerator);

    bt_layout->addWidget(resetBTBtn);

    edit_bt_btn_ = new QPushButton("Launch Groot2 Visual BT Editor");
    // edit_bt_btn_->setEnabled(true);
    edit_bt_btn_->setToolTip(
        "<html><b>Launch Groot2</b><br>"
        "Open the Behavior Tree configuration in Groot2 for visual editing.<br>"
        "<b>Note:</b> Ensure Groot2 is installed and accessible from your system PATH.</html>");
    edit_bt_btn_->setStyleSheet(
        "QPushButton {"
        "  padding: 5px 8px;"
        "  border: 2px solid #9b59b6;"
        "  border-radius: 6px;"
        "  background-color: #f4f1f8;"
        // "  color: #7d3c98;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  border-color: #7d3c98;"
        "  background-color: #e8daef;"
        "  color: #6c3483;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #9b59b6;"
        "  color: white;"
        "}");
    bt_layout->addWidget(edit_bt_btn_);

    connect(edit_bt_btn_, &QPushButton::clicked, this, [this](bool checked)
            {
        // Toggle the button state
        edit_bt_btn_->setDown(checked);
        onEditAllInGroot(); });

    bt_group_->setLayout(bt_layout);
    main_layout->addWidget(bt_group_);

    // main_layout->addWidget(output_group);

    // ─── Reset Button ───
    reset_button_ = new QPushButton("Reset Panel", this);
    main_layout->addStretch();
    reset_button_->setToolTip("Clear everything and return to initial panel state");
    reset_button_->setStyleSheet(
        "QPushButton {"
        "  padding: 5px 8px;"
        "  border: 2px solid #e74c3c;"
        "  border-radius: 6px;"
        "  background-color: #fdf2f2;"
        // "  color: #c0392b;"
        "  font-weight: bold;"
        "  margin-top: 8px;"
        "}"
        "QPushButton:hover {"
        "  border-color: #c0392b;"
        "  background-color:rgb(250, 205, 203);"
        "  color: #c0392b;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #e74c3c;"
        "  color: white;"
        "}");
    main_layout->addWidget(reset_button_);

    connect(reset_button_, &QPushButton::clicked, this, &ActorPanel::resetPanel);

    main_layout->addStretch();

    auto container = new QWidget;
    container->setLayout(main_layout);

    auto scroll = new QScrollArea;
    scroll->setWidget(container);
    scroll->setWidgetResizable(true);

    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    auto topLayout = new QVBoxLayout(this);
    topLayout->setContentsMargins(0, 0, 0, 0);
    topLayout->addWidget(scroll);

    setLayout(topLayout);

    // Set the main layout
    setLayout(main_layout);

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

  void ActorPanel::switchButtonLayout(PanelMode mode)
  {
    if (mode == CREATE_MODE)
    {
      edit_mode_widget_->hide();
      // Remove from edit layout and add to create layout for full width
      edit_mode_layout_->removeWidget(actor_button_);
      create_mode_layout_->addWidget(actor_button_);

      // Hide edit mode buttons
      add_agent_button_->hide();
      add_agent_button_->setDown(false);
      add_agent_button_->setChecked(false);
      edit_goals_button_->hide();
      edit_goals_button_->setDown(false);
      edit_goals_button_->setChecked(false);

      // Update button text and style
      actor_button_->setText("Generate Agents");
      // actor_button_->setStyleSheet(button_style);
    }
    else // EDIT_MODE
    {
      // Remove from create layout and add to edit layout
      create_mode_layout_->removeWidget(actor_button_);
      edit_mode_layout_->insertWidget(0, actor_button_);

      // Show the edit mode container
      edit_mode_widget_->show();

      // Show edit mode buttons
      add_agent_button_->show();
      add_agent_button_->setEnabled(true);
      add_agent_button_->setDown(false);
      add_agent_button_->setChecked(false);
      edit_goals_button_->show();
      edit_goals_button_->setVisible(true);
      edit_goals_button_->setDown(false);
      edit_goals_button_->setChecked(false);

      // Update button text and style
      actor_button_->setText("Edit Agents");
      // actor_button_->setStyleSheet(button_style);
    }
  }



  // std::string ActorPanel::share_to_src_path(const std::string& share_path) 
  // {
  //   // Example:
  //   // input: /home/hunav_gz_classic_ws/install/hunav_gazebo_wrapper/share/hunav_gazebo_wrapper
  //   // output: /home/hunav_gz_classic_ws/src/hunav_gazebo_wrapper

  //   std::vector<std::string> parts;
  //   std::stringstream ss(share_path);
  //   std::string item;
  //   while (std::getline(ss, item, '/')) {
  //       if (!item.empty()) parts.push_back(item);
  //   }

  //   auto it = std::find(parts.begin(), parts.end(), "install");
  //   if (it == parts.end() || (it + 1) == parts.end()) {
  //       throw std::runtime_error("The path does not have the expected structure  ../install/share/package_name");
  //   }
  //   size_t install_idx = std::distance(parts.begin(), it);
  //   std::string pkg_name = parts[install_idx + 1];

  //   std::ostringstream src_path;
  //   for (size_t i = 0; i < install_idx; ++i) {
  //       src_path << "/" << parts[i];
  //   }

  //   src_path << "/src/" << pkg_name;
    
  //   // Check if this is hunav_agent_manager or hunav_rviz2_panel (nested in hunav_sim) (local only)
  //   // if (pkg_name == "hunav_agent_manager" || pkg_name == "hunav_rviz2_panel") {
  //   //     src_path << "/src/hunav_sim/" << pkg_name << "/";
  //   // } else {
  //   //     // For wrapper packages (Isaac, Gazebo, etc.) - use direct src path
  //   //     src_path << "/src/";
  //   // }

  //   RCLCPP_INFO(this->get_logger(), "Path to store the scenario file: %s!!!",
  //                 src_path.str().c_str());

  //   return src_path.str();
  // }

  



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

        // closing arrow back to start (if any goals were defined and cyclic goals == true)
        bool is_cyclic = node["cyclic_goals"].as<bool>();
        if (node["goals"].size() > 0 && is_cyclic)
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

    // — 2) Determine behavior trees directory —
    QString btDir;
    QString simulatorName = simulator_combo_->currentText();

    // Map simulator names to their corresponding ROS2 package names
    QString packageName;
    if (simulatorName == "Gazebo Classic")
    {
      packageName = "hunav_gazebo_wrapper";
    }
    else if (simulatorName == "Gazebo Fortress")
    {
      packageName = "hunav_gazebo_fortress_wrapper";
    }
    else if (simulatorName == "Isaac Sim")
    {
      packageName = "hunav_isaac_wrapper";
    }
    else
    { // Webots
      packageName = "hunav_webots_wrapper";
    }

    // Try ROS2 package discovery
    try
    {
      QString shareDir = QString::fromStdString(
          ament_index_cpp::get_package_share_directory(packageName.toStdString()));
      std::string srcDir = this->share_to_src_path(shareDir.toStdString());
      btDir = QString::fromStdString(srcDir + "/behavior_trees");
      RCLCPP_INFO(get_logger(), "Found ROS2 package '%s', behavior trees at: %s",
                  packageName.toStdString().c_str(), btDir.toStdString().c_str());
    }
    catch (const std::exception &e)
    {
      // Fallback to development paths only if package not found
      RCLCPP_WARN(get_logger(), "ROS2 package '%s' not found, falling back to development paths",
                  packageName.toStdString().c_str());

      QString homePath = QDir::homePath() + "/" + packageName + "/behavior_trees";
      QString dockerPath = "/workspace/hunav_isaac_ws/src/" + packageName + "/behavior_trees";

      btDir = QDir(dockerPath).exists() ? dockerPath : homePath;
      RCLCPP_INFO(get_logger(), "Using fallback path: %s", btDir.toStdString().c_str());
    }

    // — 3) Agent Selection Dialog —
    QString btFileToOpen;
    QString yamlBaseName;

    if (panel_mode_ == EDIT_MODE && !orig_yaml_base_name_.isEmpty())
    {
      yamlBaseName = orig_yaml_base_name_;
    }
    else if (panel_mode_ == CREATE_MODE && !yaml_base_name_.isEmpty())
    {
      yamlBaseName = yaml_base_name_;
    }

    // Only show agent selection if we have agents and a valid scenario
    if (!loaded_agent_names_.empty() && !yamlBaseName.isEmpty())
    {
      // Create agent selection dialog
      QDialog agentDialog(this);
      agentDialog.setWindowTitle("Select Agent for Behavior Tree Editing");
      agentDialog.setMinimumWidth(400);
      agentDialog.setStyleSheet(
          "QDialog {"
          "  border: 2px solid #9b59b6;"
          "  border-radius: 8px;"
          "}"
          "QLabel {"
          "  color: #2c3e50;"
          "  font-weight: bold;"
          "  font-size: 14px;"
          "}"
          "QComboBox {"
          "  padding: 8px 12px;"
          "  border: 2px solid #bdc3c7;"
          "  border-radius: 6px;"
          "  font-size: 14px;"
          "  min-height: 20px;"
          "}"
          "QComboBox:focus {"
          "  border-color: #9b59b6;"
          "}"
          "QPushButton {"
          "  padding: 8px 16px;"
          "  border: 2px solid #9b59b6;"
          "  border-radius: 6px;"
          "  background-color: #f4f1f8;"
          "  color: #7d3c98;"
          "  font-weight: bold;"
          "  font-size: 13px;"
          "  min-width: 80px;"
          "}"
          "QPushButton:hover:enabled {"
          "  border-color: #7d3c98;"
          "}"
          "QPushButton:pressed {"
          "  background-color: #9b59b6;"
          "  color: white;"
          "}");

      QVBoxLayout *dialogLayout = new QVBoxLayout(&agentDialog);
      dialogLayout->setSpacing(16);
      dialogLayout->setContentsMargins(20, 20, 20, 20);

      // Header
      QLabel *headerLabel = new QLabel("Choose Agent for BT Editing");
      headerLabel->setStyleSheet(
          "QLabel {"
          "  font-size: 16px;"
          "  font-weight: bold;"
          "  color: #9b59b6;"
          "  background-color: #f4f1f8;"
          "  padding: 8px;"
          "  border-radius: 6px;"
          "  border: 1px solid #9b59b6;"
          "}");
      headerLabel->setAlignment(Qt::AlignCenter);
      dialogLayout->addWidget(headerLabel);

      // Agent selection
      QLabel *selectLabel = new QLabel("Select agent to edit behavior tree:");
      dialogLayout->addWidget(selectLabel);

      QComboBox *agentSelector = new QComboBox;

      // Populate with agent information
      for (size_t i = 0; i < loaded_agent_names_.size(); ++i)
      {
        QString agentName = QString::fromStdString(loaded_agent_names_[i]);
        QString behaviorType = "Regular"; // Default

        if (i < loaded_agent_nodes_.size() &&
            loaded_agent_nodes_[i]["behavior"] &&
            loaded_agent_nodes_[i]["behavior"]["type"])
        {
          behaviorType = QString::fromStdString(
              loaded_agent_nodes_[i]["behavior"]["type"].as<std::string>());
        }

        QString displayText = QString("Agent %1 - %2 Behavior")
                                  .arg(i + 1)
                                  .arg(behaviorType);

        agentSelector->addItem(displayText, static_cast<int>(i + 1));

        QString btFileName = QString("%1__agent_%2_bt.xml").arg(yamlBaseName).arg(i + 1);
        QString fullBtPath = btDir + "/" + btFileName;
        agentBtPaths_.push_back(fullBtPath);
      }

      agentSelector->setCurrentIndex(0);
      dialogLayout->addWidget(agentSelector);

      // Information label
      QLabel *infoLabel = new QLabel(
          "<html><i>The selected agent's behavior tree file will be opened in Groot2 for editing.<br>"
          "File format: <b>" +
          yamlBaseName + "__agent_X_bt.xml</b></i></html>");
      infoLabel->setStyleSheet(
          "QLabel {"
          "  color: #7f8c8d;"
          "  font-size: 12px;"
          "  background-color: #ecf0f1;"
          "  padding: 8px;"
          "  border-radius: 4px;"
          "  border-left: 4px solid #9b59b6;"
          "}");
      dialogLayout->addWidget(infoLabel);

      // Buttons
      QHBoxLayout *buttonLayout = new QHBoxLayout;
      QPushButton *openButton = new QPushButton("Open in Groot2");
      QPushButton *cancelButton = new QPushButton("Cancel");

      openButton->setStyleSheet(openButton->styleSheet() +
                                "QPushButton { border-color: #27ae60; background-color: #d5f4e6; }"
                                "QPushButton:hover { background-color: #a9dfbf; }");
      cancelButton->setStyleSheet(cancelButton->styleSheet() +
                                  "QPushButton { border-color: #e74c3c; background-color: #fdf2f2; }"
                                  "QPushButton:hover { background-color: #fcebea; }");

      buttonLayout->addStretch();
      buttonLayout->addWidget(openButton);
      buttonLayout->addWidget(cancelButton);
      dialogLayout->addLayout(buttonLayout);

      // Connect buttons
      connect(openButton, &QPushButton::clicked, &agentDialog, &QDialog::accept);
      connect(cancelButton, &QPushButton::clicked, &agentDialog, &QDialog::reject);

      // Show dialog and get result
      if (agentDialog.exec() == QDialog::Accepted)
      {
        int selectedAgentId = agentSelector->currentData().toInt();
        QString selectedAgentName = QString::fromStdString(loaded_agent_names_[selectedAgentId - 1]);
        QString selectedBehavior = "Regular";

        if (selectedAgentId - 1 < static_cast<int>(loaded_agent_nodes_.size()) &&
            loaded_agent_nodes_[selectedAgentId - 1]["behavior"] &&
            loaded_agent_nodes_[selectedAgentId - 1]["behavior"]["type"])
        {
          selectedBehavior = QString::fromStdString(
              loaded_agent_nodes_[selectedAgentId - 1]["behavior"]["type"].as<std::string>());
        }

        // Construct the BT file path for the selected agent
        QString btFileName = QString("%1__agent_%2_bt.xml").arg(yamlBaseName).arg(selectedAgentId);
        QString fullBtPath = btDir + "/" + btFileName;

        if (QFile::exists(fullBtPath))
        {
          btFileToOpen = fullBtPath;
          RCLCPP_INFO(get_logger(), "Opening BT file for Agent %d (%s - %s): %s",
                      selectedAgentId,
                      selectedAgentName.toStdString().c_str(),
                      selectedBehavior.toStdString().c_str(),
                      btFileToOpen.toStdString().c_str());
        }
        else
        {
          RCLCPP_WARN(get_logger(), "BT file not found for Agent %d: %s",
                      selectedAgentId, fullBtPath.toStdString().c_str());

          QMessageBox::warning(this, "File Not Found",
                               QString("<html><h3>Behavior Tree File Not Found</h3>"
                                       "<p><b>Agent:</b> %1 (%2)</p>"
                                       "<p><b>Expected file:</b> %3</p>"
                                       "<p><b>Location:</b> %4</p>"
                                       "<br><p>Please ensure the scenario has been saved and behavior trees generated.</p></html>")
                                   .arg(selectedAgentId)
                                   .arg(selectedAgentName)
                                   .arg(btFileName)
                                   .arg(btDir));
          return;
        }
      }
      else
      {
        // User cancelled
        return;
      }
    }
    else if (!yamlBaseName.isEmpty())
    {
      // Fallback to first agent if no agent selection is possible
      QString btFileName = QString("%1__agent_1_bt.xml").arg(yamlBaseName);
      QString fullBtPath = btDir + "/" + btFileName;

      if (QFile::exists(fullBtPath))
      {
        btFileToOpen = fullBtPath;
        RCLCPP_INFO(get_logger(), "Opening default BT file (Agent 1): %s", btFileToOpen.toStdString().c_str());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Default BT file not found: %s", fullBtPath.toStdString().c_str());
      }
    }

    // Show information about the BT directory and selected file
    QString infoMessage;
    if (!btFileToOpen.isEmpty())
    {
      QFileInfo fileInfo(btFileToOpen);
      infoMessage = QString(
                        "<html>All generated BehaviorTree XML files are located in:<br><br>"
                        "%1<br><br>"
                        "<b>Groot2 will launch with the following agent's behavior tree file:</b> %2<br></html>")
                        .arg(btDir)
                        .arg(fileInfo.fileName());
    }
    else
    {
      infoMessage = QString(
                        "<html>All generated BehaviorTree XML files are located in:<br><br>"
                        "%1<br><br>"
                        "<b>Groot2 will now launch.</b></html>")
                        .arg(btDir);
    }

    QMessageBox::information(this, "Behavior Trees Location", infoMessage);

    // — 3) Launch Groot2 —

    QStringList grootCandidates = {
        // Docker environment path
        "/opt/Groot2/squashfs/AppRun",

        // Native environment paths
        QDir::home().filePath("Groot2/bin/groot2"),
        QDir::home().filePath("groot2/bin/groot2")};

    QString grootExe;
    bool grootFound = false;

    // Search for Groot2 executable in candidate paths
    for (const QString &candidate : grootCandidates)
    {
      QFileInfo fileInfo(candidate);
      if (fileInfo.exists() && fileInfo.isExecutable())
      {
        grootExe = candidate;
        grootFound = true;
        RCLCPP_INFO(get_logger(), "Found Groot2 executable at: %s",
                    grootExe.toStdString().c_str());
        break;
      }
    }

    // If no executable found, try using 'which' command or environment detection
    if (!grootFound)
    {
      // Try to find groot2 using system PATH
      QProcess whichProcess;
      whichProcess.start("which", QStringList() << "groot2");
      whichProcess.waitForFinished(3000); // 3 second timeout

      if (whichProcess.exitCode() == 0)
      {
        grootExe = QString::fromUtf8(whichProcess.readAllStandardOutput()).trimmed();
        if (!grootExe.isEmpty())
        {
          grootFound = true;
          RCLCPP_INFO(get_logger(), "Found Groot2 via PATH: %s",
                      grootExe.toStdString().c_str());
        }
      }
    }

    if (grootFound && !grootExe.isEmpty())
    {
      // Launch Groot2 with enhanced error handling and direct file opening
      QStringList arguments;

      // Determine which BT file to open
      QString btFileToOpen;

      if (panel_mode_ == EDIT_MODE && !orig_yaml_base_name_.isEmpty())
      {
        // In EDIT mode, try to open the BT file for the first agent
        QString btFileName = QString("%1__agent_1_bt.xml").arg(orig_yaml_base_name_);
        QString fullBtPath = btDir + "/" + btFileName;

        if (QFile::exists(fullBtPath))
        {
          btFileToOpen = fullBtPath;
          RCLCPP_INFO(get_logger(), "Opening existing BT file: %s", btFileToOpen.toStdString().c_str());
        }
        else
        {
          RCLCPP_WARN(get_logger(), "BT file not found: %s", fullBtPath.toStdString().c_str());
        }
      }
      else if (panel_mode_ == CREATE_MODE && !yaml_base_name_.isEmpty())
      {
        // In CREATE mode after saving, try to open the BT file for the first agent
        QString btFileName = QString("%1__agent_1_bt.xml").arg(yaml_base_name_);
        QString fullBtPath = btDir + "/" + btFileName;

        if (QFile::exists(fullBtPath))
        {
          btFileToOpen = fullBtPath;
          RCLCPP_INFO(get_logger(), "Opening newly created BT file: %s", btFileToOpen.toStdString().c_str());
        }
        else
        {
          RCLCPP_WARN(get_logger(), "BT file not found: %s", fullBtPath.toStdString().c_str());
        }
      }

      // Add file argument if we found a valid BT file
      if (!btFileToOpen.isEmpty())
      {
        arguments << "--file" << btFileToOpen;
      }
      else
      {
        // Fallback: just open Groot2 in the behavior trees directory
        RCLCPP_INFO(get_logger(), "No specific BT file found, opening Groot2 in default directory");
      }

      if (QProcess::startDetached(grootExe, arguments))
      {
        QString successMsg = btFileToOpen.isEmpty()
                                 ? QString("Successfully launched Groot2 from: %1").arg(grootExe)
                                 : QString("Successfully launched Groot2 with file: %1").arg(QFileInfo(btFileToOpen).fileName());

        RCLCPP_INFO(get_logger(), "%s", successMsg.toStdString().c_str());
      }
      else
      {
        // Enhanced error message with troubleshooting info
        QMessageBox::warning(this, "Launch Error",
                             QString("<html><head><style>"
                                     "body { font-family: 'Segoe UI', Arial, sans-serif; margin: 8px; }"
                                     ".error { color: #e74c3c; font-weight: bold; }"
                                     ".path { background-color: #f8f9fa; padding: 2px 4px; border-radius: 4px; "
                                     "        font-family: 'Consolas', monospace; }"
                                     ".suggestion { margin-top: 8px; padding: 4px; background-color: #fff3cd; "
                                     "             border-radius: 4px; border-left: 4px solid #ffc107; }"
                                     "</style></head><body>"

                                     "<div class='error'>Failed to launch Groot2</div><br>"

                                     "<strong>Executable found at:</strong><br>"
                                     "<span class='path'>%1</span><br><br>"

                                     "%2"

                                     "<strong>Possible causes:</strong><br>"
                                     "• Groot2 executable lacks proper permissions<br>"
                                     "• Missing required libraries or dependencies<br>"
                                     "• Display/X11 forwarding issues (in Docker)<br>"
                                     "• Invalid file path or format<br><br>"

                                     "<div class='suggestion'>"
                                     "<strong>Troubleshooting:</strong><br>"
                                     "• Check file permissions: <code>ls -la %1</code><br>"
                                     "• Verify dependencies: <code>ldd %1</code><br>"
                                     "• For Docker: Ensure X11 forwarding is enabled<br>"
                                     "• Try launching Groot2 manually to test"
                                     "</div>"

                                     "</body></html>")
                                 .arg(grootExe)
                                 .arg(btFileToOpen.isEmpty()
                                          ? ""
                                          : QString("<strong>Attempted to open file:</strong><br><span class='path'>%1</span><br><br>")
                                                .arg(btFileToOpen)));
      }
    }
  }

  // Utility: load file to string
  // static QString loadFile(const QString &path)
  // {
  //   QFile f(path);
  //   if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
  //     return {};
  //   return f.readAll();
  // }

  QString ActorPanel::loadFile(const QString &filepath)
  {
    QString fullPath = filepath;

    if (!QFile::exists(fullPath))
    {
      try
      {
        auto shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory("hunav_agent_manager"));
        std::string srcDir = this->share_to_src_path(shareDir.toStdString());
        fullPath = QString::fromStdString(srcDir) + "/behavior_trees/" + filepath;
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Could not find share dir for hunav_agent_manager: %s",
                    e.what());
      }
    }

    QFile file(fullPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot open BT template file: %s",
                  fullPath.toStdString().c_str());
      return QString();
    }

    QTextStream in(&file);
    return in.readAll();
  }



  void ActorPanel::addAgent()
  {
    // ─────────────────────────── DETERMINE TOTAL AGENTS ────────────────────────
    if (panel_mode_ == CREATE_MODE)
    {
      num_agents = actors->text().toInt();

      // Make sure all arrays are sized correctly for CREATE mode
      while (loaded_agent_nodes_.size() < static_cast<size_t>(num_agents))
      {
        loaded_agent_nodes_.push_back(YAML::Node());
      }
      while (loaded_agent_names_.size() < static_cast<size_t>(num_agents))
      {
        loaded_agent_names_.push_back("");
      }
      while (loaded_agent_goals_.size() < static_cast<size_t>(num_agents))
      {
        loaded_agent_goals_.push_back(std::vector<int>());
      }
      while (loaded_initial_marker_ids_.size() < static_cast<size_t>(num_agents))
      {
        loaded_initial_marker_ids_.push_back(-1);
      }
    }
    else if (panel_mode_ == EDIT_MODE)
    {
      num_agents = static_cast<int>(loaded_agent_nodes_.size());

      // For adding new agents in EDIT mode, we might need to expand
      if (adding_new_agent_)
      {
        num_agents = static_cast<int>(loaded_agent_names_.size()) + 1;
      }
    }

    // Validate current_edit_idx_
    if (!adding_new_agent_ &&
        (current_edit_idx_ < 0 || current_edit_idx_ >= static_cast<int>(loaded_agent_names_.size())))
    {
      RCLCPP_ERROR(get_logger(),
                   "Invalid current_edit_idx_ %d (should be 0-%zu)",
                   current_edit_idx_, loaded_agent_names_.size() - 1);
      current_edit_idx_ = 0;
    }
    initAgentColors(num_agents);

    // ─────────────────────── POPUP WINDOW SETUP ─────────────────────────────────
    if (!window)
    {
      window = new QWidget;
      window->setWindowFlag(Qt::WindowStaysOnTopHint);
      window->setStyleSheet(
          "QGroupBox {"
          "  font-weight: bold;"
          "  font-size: 16px;"
          "  color:rgb(0, 0, 0);"
          "  margin-top: 8px;"
          "  padding-top: 4px;"
          "  border: 2px solid #3498db;"
          "  border-radius: 6px;"
          "  background-color: rgb(229, 236, 247);"
          "}"
          "QGroupBox::title {"
          "  font-size: 16px;"
          "  subcontrol-origin: margin;"
          "  subcontrol-position: top left;"
          "  padding: 0 4px;"
          "  border: 1px solid #3498db;"
          "  background-color: rgb(150, 198, 253);"
          "  border-radius: 4px;"
          "}"
          "QLabel {"
          "  font-weight: bold;"
          "}"
          "QLineEdit {"
          "  padding: 3px 5px;"
          "  border: 2px solid #bdc3c7;"
          "  border-radius: 4px;"
          "}"
          "QComboBox {"
          "  padding: 3px 5px;"
          "}"
          "QLineEdit:focus, QComboBox:focus {"
          "  border-color: #3498db;"
          "}"
          "QPushButton {"
          "  padding: 4px 8px;"
          "  border: 2px solid #27ae60;"
          "  border-radius: 6px;"
          "  background-color: #d5f4e6;"
          "  font-weight: bold;"
          "}"
          "QPushButton:hover {"
          "  border-color: #1e8449;"
          "  background-color: #a9dfbf;"
          "}"
          "QPushButton:disabled {"
          "  background-color: #f8f9fa;"
          "  color: #aeb6bf;"
          "  border-color: #d5dbdb;"
          "}");

      QTimer::singleShot(0, this, [this]()
                         {
          window->adjustSize();
          QPoint panelTL = this->mapToGlobal(QPoint(0,0));
          int x = panelTL.x() + (this->width() - window->width()) / 2;

          window->move(x, panelTL.y()); });
    }

    // Enhanced window title
    QString window_title;
    if (adding_new_agent_)
    {
      window_title = QString("Add New Agent (#%1)").arg(current_edit_idx_ + 1);
    }
    else if (panel_mode_ == CREATE_MODE)
    {
      window_title = QString("Agent %1 of %2").arg(agent_count).arg(num_agents);
    }
    else
    {
      window_title = QString("Edit Agent %1 of %2").arg(current_edit_idx_ + 1).arg(num_agents);
    }
    window->setWindowTitle(window_title);

    // Clear existing layout
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

    // ─── Header with Agent Info ───
    QLabel *header = new QLabel(window_title, window);
    header->setStyleSheet(
        "QLabel {"
        "  font-size: 18px;"
        "  font-weight: bold;"
        // "  color: #2c3e50;"
        "  background-color:rgb(120, 179, 247);"
        "  padding: 4px;"
        "  border-radius: 6px;"
        "  border: 4px #3498db;"
        "}");
    header->setAlignment(Qt::AlignCenter);
    topic_layout->addWidget(header);

    // ─── Navigation Buttons for Edit Mode ───
    if (panel_mode_ == EDIT_MODE && !adding_new_agent_)
    {
      QHBoxLayout *nav_layout = new QHBoxLayout;

      QPushButton *prev_btn = new QPushButton("Previous", window);
      QPushButton *next_btn = new QPushButton("Next", window);

      prev_btn->setEnabled(current_edit_idx_ > 0);
      next_btn->setEnabled(current_edit_idx_ + 1 < num_agents);

      nav_layout->addStretch();
      nav_layout->addWidget(prev_btn);
      nav_layout->addWidget(next_btn);
      nav_layout->addStretch();

      topic_layout->addLayout(nav_layout);

      // Connect navigation
      connect(prev_btn, &QPushButton::clicked, this, [this]()
              {
        if (current_edit_idx_ > 0) {
          current_edit_idx_--;
          window->close();
          addAgent();
        } });

      connect(next_btn, &QPushButton::clicked, this, [this]()
              {
        if (current_edit_idx_ + 1 < num_agents) {
          current_edit_idx_++;
          window->close();
          addAgent();
        } });
    }
    // ─────────────────────────── AGENT CONFIGURATION FIELDS ─────────────────────

    // ─── Basic Configuration Group ───
    QGroupBox *basic_group = new QGroupBox("Basic Configuration");
    QVBoxLayout *basic_layout = new QVBoxLayout;

    // Desired velocity
    basic_layout->addWidget(new QLabel("Maximum velocity (m/s):"));
    agent_desired_vel = new QLineEdit(window);
    agent_desired_vel->setText(QString::number(1.5, 'f', 1));
    agent_desired_vel->setPlaceholderText("0.0 - 1.5 m/s");
    basic_layout->addWidget(agent_desired_vel);

    // Cyclic goals
    cyclic_goals_checkbox = new QCheckBox("Cyclic navigation", window);
    cyclic_goals_checkbox->setChecked(true);
    cyclic_goals_checkbox->setToolTip("Agent will return to the first goal after reaching the last one and start navigation over.");
    basic_layout->addWidget(cyclic_goals_checkbox);

    basic_group->setLayout(basic_layout);
    topic_layout->addWidget(basic_group);

    // ─── Behavior Configuration Group ───
    QGroupBox *behavior_group = new QGroupBox("Behavior Configuration");
    QVBoxLayout *behavior_layout = new QVBoxLayout;

    // Behavior type
    behavior_layout->addWidget(new QLabel("Behavior type:"));
    behavior_type_combobox = new QComboBox(window);
    behavior_type_combobox->addItems({"Regular", "Impassive", "Surprised", "Scared", "Curious", "Threatening"});
    behavior_layout->addWidget(behavior_type_combobox);
    connect(behavior_type_combobox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ActorPanel::checkComboBoxConf);

    // Behavior configuration
    behavior_layout->addWidget(new QLabel("Configuration preset:"));
    behavior_conf_combobox = new QComboBox(window);
    behavior_conf_combobox->addItems({"Default", "Custom", "Random-normal distribution", "Random-uniform distribution"});
    behavior_layout->addWidget(behavior_conf_combobox);

    dur = new QLabel("Duration:", window);
    dur->setVisible(false);
    beh_duration = new QLineEdit(window);
    beh_duration->setVisible(false);
    once = new QLabel("Only once:", window);
    once->setVisible(false);
    beh_once = new QLineEdit(window);
    beh_once->setVisible(false);
    dist = new QLabel("Visibility distance:", window);
    dist->setVisible(false);
    beh_dist = new QLineEdit(window);
    beh_dist->setVisible(false);
    vel = new QLabel("Behavior velocity:", window);
    vel->setVisible(false);
    beh_vel = new QLineEdit(window);
    beh_vel->setVisible(false);
    other = new QLabel("Repulsive force factor:", window);
    other->setVisible(false);
    beh_otherff = new QLineEdit(window);
    beh_otherff->setVisible(false);
    front = new QLabel("Front distance:", window);
    front->setVisible(false);
    beh_front_dist = new QLineEdit(window);
    beh_front_dist->setVisible(false);
    safe_dist = new QLabel("Safe distance:", window);
    safe_dist->setVisible(false);
    beh_safe_distance = new QLineEdit(window);
    beh_safe_distance->setVisible(false);
    stop = new QLabel("Stop distance:", window);
    stop->setVisible(false);
    beh_stop_dist = new QLineEdit(window);
    beh_stop_dist->setVisible(false);

    // Agent desired velocity tooltip
    agent_desired_vel->setToolTip(
        "<html><b>Maximum Velocity</b><br>"
        "Sets the agent's maximum walking speed in meters per second.<br>"
        "<b>Range:</b> 0.1 - 1.5 m/s</html>");

    // Behavior duration tooltip
    beh_duration->setToolTip(
        "<html><b>Duration</b><br>"
        "How long the special behavior lasts (in seconds) when triggered.<br>"
        "<b>Note:</b> After this time, agent returns to regular navigation.</html>");

    // Behavior velocity tooltip
    beh_vel->setToolTip(
        "<html><b>Behavior Velocity</b><br>"
        "Agent's velocity during the special behavior state.<br>"
        "<b>Note:</b> Usually different from normal walking speed.</html>");

    // Behavior once tooltip
    beh_once->setToolTip(
        "<html><b>Only Once</b><br>"
        "Whether the behavior triggers only once per simulation.<br>"
        "<b>Values:</b><br>"
        "• <b>true:</b> Behavior happens only the first time<br>"
        "• <b>false:</b> Behavior can happen multiple times</html>");

    // Visibility distance tooltip
    beh_dist->setToolTip(
        "<html><b>Visibility Distance</b><br>"
        "Maximum distance at which the agent can detect the robot.<br>"
        "<b>Range:</b> 1.0 - 15.0 meters<br>"
        "<b>Note:</b> Larger values make agents more reactive to distant robots.</html>");

    // Stop distance tooltip (for Curious behavior)
    beh_stop_dist->setToolTip(
        "<html><b>Stop Distance</b><br>"
        "Distance from robot where curious agent stops approaching.<br>"
        "<b>Range:</b> 0.5 - 3.0 meters<br>"
        "<b>Note:</b> Agent maintains this distance when observing robot.</html>");

    // Front distance tooltip (for Threatening behavior)
    beh_front_dist->setToolTip(
        "<html><b>Front Distance</b><br>"
        "How far in front of the robot the agent positions itself.<br>"
        "<b>Range:</b> 0.5 - 2.0 meters<br>"
        "<b>Note:</b> Used for blocking or confrontational behavior.</html>");

    // Safe distance tooltip (for Scared behavior)
    beh_safe_distance->setToolTip(
        "<html><b>Safe Distance</b><br>"
        "Minimum distance the scared agent tries to maintain from the robot.<br>"
        "<b>Range:</b> 2.0 - 10.0 meters<br>"
        "<b>Note:</b> Agent moves away if robot gets closer than this distance.</html>");

    behavior_layout->addWidget(dur);
    behavior_layout->addWidget(beh_duration);
    behavior_layout->addWidget(vel);
    behavior_layout->addWidget(beh_vel);
    behavior_layout->addWidget(once);
    behavior_layout->addWidget(beh_once);
    behavior_layout->addWidget(dist);
    behavior_layout->addWidget(beh_dist);
    behavior_layout->addWidget(stop);
    behavior_layout->addWidget(beh_stop_dist);
    behavior_layout->addWidget(front);
    behavior_layout->addWidget(beh_front_dist);
    behavior_layout->addWidget(safe_dist);
    behavior_layout->addWidget(beh_safe_distance);
    behavior_layout->addWidget(other);
    behavior_layout->addWidget(beh_otherff);

    behavior_group->setLayout(behavior_layout);
    topic_layout->addWidget(behavior_group);

    // ─── Simulator-Specific Configuration Group ───
    QGroupBox *sim_group = new QGroupBox("Simulator Options");
    QVBoxLayout *sim_layout = new QVBoxLayout;

    // Skin selection (simulator-specific)
    skin_label_ = new QLabel("Agent appearance:", window);
    skin_combobox = new QComboBox(window);
    skin_combobox->setToolTip(
        "<html><b>Agent Appearance</b><br>"
        "Visual representation of the agent in simulation.</html>");

    // Populate combobox based on current simulator
    QString currentSim = simulator_combo_->currentText();

    skin_combobox->clear();

    if (currentSim.contains("Gazebo", Qt::CaseInsensitive) ||
        currentSim == "Gazebo Classic" ||
        currentSim == "Gazebo Fortress")
    {
      skin_combobox->addItems({"Elegant man",
                               "Casual man",
                               "Elegant woman",
                               "Regular man",
                               "Worker man",
                               "Blue jeans",
                               "Green t-shirt",
                               "Blue t-shirt",
                               "Red t-shirt"});
    }
    else if (currentSim.contains("Isaac", Qt::CaseInsensitive) || currentSim == "Isaac Sim")
    {
      skin_combobox->addItems({"Random",
                               "F_Business_02",
                               "F_Medical_01",
                               "M_Medical_01",
                               "male_adult_construction_01",
                               "male_adult_construction_05",
                               "female_adult_police_01",
                               "female_adult_police_02",
                               "female_adult_police_03",
                               "male_adult_police_04",
                               "female_adult_business_02",
                               "female_adult_medical_01"});

      // Set "Random" as default
      skin_combobox->setCurrentIndex(0);
    }
    else if (currentSim.contains("Webots", Qt::CaseInsensitive) || currentSim == "Webots")
    {
      skin_combobox->addItems({
          "Default Character", // 0 - placeholder
          "Character Type 1",  // 1 - placeholder
          "Character Type 2",  // 2 - placeholder
          "Character Type 3"   // 3 - placeholder
      });
    }

    skin_combobox->update();
    skin_combobox->repaint();
    skin_label_->update();
    skin_label_->repaint();

    sim_layout->addWidget(skin_label_);
    sim_layout->addWidget(skin_combobox);

    sim_group->setLayout(sim_layout);
    sim_group->setVisible(true);
    sim_group->show();

    topic_layout->addWidget(sim_group);

    // Store the current connection to avoid duplicates
    static QMetaObject::Connection simulator_connection;

    // Disconnect any existing connection
    if (simulator_connection)
    {
      disconnect(simulator_connection);
    }

    // Update skin options when simulator changes in the main panel
    simulator_connection = connect(simulator_combo_,
                                   QOverload<int>::of(&QComboBox::currentIndexChanged),
                                   this,
                                   [this]()
                                   {
                                     // Only update if the dialog is open and skin_combobox exists
                                     if (!skin_combobox || !window || !window->isVisible())
                                       return;

                                     QString newSim = simulator_combo_->currentText();
                                     RCLCPP_INFO(get_logger(), "Simulator changed to: '%s' while agent window is open", newSim.toStdString().c_str());

                                     // Clear existing items
                                     skin_combobox->clear();

                                     if (newSim.contains("Gazebo", Qt::CaseInsensitive) ||
                                         newSim == "Gazebo Classic" ||
                                         newSim == "Gazebo Fortress")
                                     {
                                       skin_combobox->addItems({"Elegant man",
                                                                "Casual man",
                                                                "Elegant woman",
                                                                "Regular man",
                                                                "Worker man",
                                                                "Blue jeans",
                                                                "Green t-shirt",
                                                                "Blue t-shirt",
                                                                "Red t-shirt"});
                                       skin_combobox->setCurrentIndex(0);
                                       skin_label_->setVisible(true);
                                       skin_combobox->setVisible(true);
                                     }
                                     else if (newSim.contains("Isaac", Qt::CaseInsensitive) || newSim == "Isaac Sim")
                                     {
                                       skin_combobox->addItems({"Random",
                                                                "F_Business_02",
                                                                "F_Medical_01",
                                                                "M_Medical_01",
                                                                "male_adult_construction_01",
                                                                "male_adult_construction_05",
                                                                "female_adult_police_01",
                                                                "female_adult_police_02",
                                                                "female_adult_police_03",
                                                                "male_adult_police_04",
                                                                "female_adult_business_02",
                                                                "female_adult_medical_01"});
                                       skin_combobox->setCurrentIndex(0);
                                       skin_label_->setVisible(true);
                                       skin_combobox->setVisible(true);
                                     }
                                     else if (newSim.contains("Webots", Qt::CaseInsensitive) || newSim == "Webots")
                                     {
                                       skin_combobox->addItems({"Default Character",
                                                                "Character Type 1",
                                                                "Character Type 2",
                                                                "Character Type 3"});
                                       skin_combobox->setCurrentIndex(0);
                                       skin_label_->setVisible(true);
                                       skin_combobox->setVisible(true);
                                     }
                                     else
                                     {
                                       skin_combobox->addItems({"Default Character",
                                                                "Character Type 1",
                                                                "Character Type 2"});
                                       skin_combobox->setCurrentIndex(0);
                                       skin_label_->setVisible(true);
                                       skin_combobox->setVisible(true);
                                     }

                                     // Force widget updates
                                     skin_combobox->update();
                                     skin_label_->update();
                                   });

    // ─── Advanced Behavior Parameters ───
    QGroupBox *advanced_group = new QGroupBox("SFM parameters");
    QVBoxLayout *advanced_layout = new QVBoxLayout;

    // (8) Hidden “GFF / OFF / SFF / …” fields:
    // vel = new QLabel("Behavior agent vel:", window);
    // vel->setVisible(false);
    // behavior_layout->addWidget(vel);

    // beh_vel = new QLineEdit(window);
    // beh_vel->setText("1.0");
    // beh_vel->setVisible(false);
    // beh_vel->setEnabled(false);
    // advanced_layout->addWidget(beh_vel);

    gff = new QLabel("Goal Force Factor:", window);
    advanced_layout->addWidget(gff);
    beh_gff = new QLineEdit(window);
    beh_gff->setText(QString::number(2.0, 'f', 1));
    beh_gff->setEnabled(false);
    beh_gff->setToolTip(
        "<html><b>Goal Force Factor</b><br>"
        "Strength of attraction towards navigation goals.<br>"
        "<b>Higher values:</b> More direct path to goals</html>");
    advanced_layout->addWidget(beh_gff);

    off = new QLabel("Obstacle Force Factor:", window);
    advanced_layout->addWidget(off);
    beh_off = new QLineEdit(window);
    beh_off->setText(QString::number(10.0, 'f', 1));
    beh_off->setEnabled(false);
    beh_off->setToolTip(
        "<html><b>Obstacle Force Factor</b><br>"
        "Strength of repulsion from obstacles and walls.<br>"
        "<b>Higher values:</b> Better obstacle avoidance</html>");
    advanced_layout->addWidget(beh_off);

    sff = new QLabel("Social Force Factor:", window);
    advanced_layout->addWidget(sff);
    beh_sff = new QLineEdit(window);
    beh_sff->setText(QString::number(5.0, 'f', 1));
    beh_sff->setEnabled(false);
    beh_sff->setToolTip(
        "<html><b>Social Force Factor</b><br>"
        "Strength of repulsion from other people.<br>"
        "<b>Higher values:</b> Agents keep more distance from each other</html>");
    advanced_layout->addWidget(beh_sff);

    other = new QLabel("Robot Repulsive Force Factor:", window);
    other->setVisible(false);
    advanced_layout->addWidget(other);
    beh_otherff = new QLineEdit(window);
    beh_otherff->setText(QString::number(20.0, 'f', 1));
    beh_otherff->setEnabled(false);
    beh_otherff->setVisible(false);
    beh_otherff->setToolTip(
        "<html><b>Robot Repulsive Force Factor</b><br>"
        "Strength of repulsion specifically from robots.<br>"
        "<b>Scared behavior:</b> Higher values make agents flee faster</html>");
    advanced_layout->addWidget(beh_otherff);

    connect(behavior_conf_combobox,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &ActorPanel::checkComboBoxConf);

    checkComboBoxConf();

    advanced_group->setLayout(advanced_layout);
    topic_layout->addWidget(advanced_group);

    // ─── Position Configuration ───
    initial_pose_button = new QPushButton("Set Initial Pose on Map", window);
    if (panel_mode_ == EDIT_MODE)
      initial_pose_button->setText("Edit Initial Pose");
    initial_pose_button->setCheckable(true);
    initial_pose_button->setStyleSheet(
        "QPushButton {"
        "  padding: 6px 8px;"
        "  border: 2px solid #e67e22;"
        "  border-radius: 6px;"
        "  background-color: #fef5e7;"
        // "  color: #d68910;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #f5b041;"
        "  color: #ffffff;"
        "  border-color: #d35400;"
        "}"
        "QPushButton:checked {"
        "  background-color: #e67e22;"
        "  color: white;"
        "}");

    topic_layout->addWidget(initial_pose_button);

    connect(initial_pose_button, &QPushButton::clicked,
            this, &ActorPanel::setInitialPose);

    save_button_ = new QPushButton("Save & Continue", window);
    save_button_->setEnabled(false);
    save_button_->setStyleSheet(
        "QPushButton {"
        "  padding: 6px 10px;"
        "  border: 2px solid #27ae60;"
        "  border-radius: 6px;"
        "  background-color: #d5f4e6;"
        // "  color: #1e8449;"
        "  font-weight: bold;"
        "  font-size: 14px;"
        "}"
        "QPushButton:hover:enabled {"
        "  border-color: #1e8449;"
        "  background-color: #a9dfbf;"
        "  color: #1e8449;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "  border-color: #d5dbdb;"
        "}");

    topic_layout->addWidget(save_button_);

    connect(save_button_, &QPushButton::clicked, [this]()
            {
    // ─────────────────────────── SAVE CURRENT FIELDS ─────────────────────────
    YAML::Node new_node;

    // (a) id (1-based) and group_id
    int agent_id;
    if (adding_new_agent_)
    {
      // For new agents, use the next available ID (current list size + 1)
      agent_id = static_cast<int>(loaded_agent_names_.size()) + 1;
    }
    else if (panel_mode_ == EDIT_MODE)
    {
      agent_id = current_edit_idx_ + 1;
    }
    else // CREATE_MODE
    {
      agent_id = agent_count;
    }

    new_node["id"] = agent_id;
    new_node["group_id"] = agent_id; //-1;

    // (b) skin
    new_node["skin"] = skin_combobox->currentIndex();

    // (c) max_vel
    new_node["max_vel"] = agent_desired_vel->text().toDouble();

    // (d) radius, goal_radius, cyclic_goals
    new_node["radius"] = "0.4";
    new_node["goal_radius"] = "0.3";
    new_node["cyclic_goals"] = cyclic_goals_checkbox->isChecked();

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
        new_node["behavior"]["other_force_factor"] = QString::number(beh_otherff->text().toDouble(), 'f', 1).toStdString();
        new_node["behavior"]["safe_distance"] = QString::number(beh_safe_distance->text().toDouble(), 'f', 1).toStdString();
        break;

      case hunav_msgs::msg::AgentBehavior::BEH_CURIOUS:
        new_node["behavior"]["dist"]   = QString::number(beh_dist->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["duration"]              = QString::number(beh_duration->text().toDouble(), 'f', 1).toStdString();
        new_node["behavior"]["once"]             = (beh_once->text().toLower() == "true");
        new_node["behavior"]["vel"]             = QString::number(beh_vel->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["stop_dist"]         = QString::number(beh_stop_dist->text().toDouble(), 'f', 2).toStdString();
        break;

      case hunav_msgs::msg::AgentBehavior::BEH_THREATENING:
        new_node["behavior"]["dist"]   = QString::number(beh_dist->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["duration"]              = QString::number(beh_duration->text().toDouble(), 'f', 2).toStdString();
        new_node["behavior"]["once"]             = (beh_once->text().toLower() == "true");
        new_node["behavior"]["front_dist"]   = QString::number(beh_front_dist->text().toDouble(), 'f', 2).toStdString();
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

    // ────────────────────── SAVE LOGIC ────────────────────────

    if (adding_new_agent_)
    {
      // ═══════════════ NOW ADD THE NEW AGENT DATA ═══════════════
      
      // Generate agent name
      std::string new_name = "agent" + std::to_string(agent_id);
      std::vector<int> empty_goals;
      
      // Add to all lists simultaneously
      loaded_agent_names_.push_back(new_name);
      loaded_agent_nodes_.push_back(new_node);
      loaded_agent_goals_.push_back(empty_goals);
      
      // Update current_edit_idx to point to the newly added agent
      current_edit_idx_ = static_cast<int>(loaded_agent_names_.size()) - 1;
      
      // Reset the flag
      adding_new_agent_ = false;
      window->close();
      
      QMessageBox::information(this,
        "Agent Added",
        QString("New agent '%1' (ID: %2) has been added successfully.\n"
                "You can now edit it or add goals to it.")
          .arg(QString::fromStdString(loaded_agent_names_[current_edit_idx_]))
          .arg(agent_id));
      
      // Update the UI to show the new agent
      publishAgentMarkers();
      add_agent_button_->setEnabled(true);
      assign_goals_btn_->setEnabled(!loaded_global_goals_.empty());

      return;
    }
    else
    {
      // ═══════════════ REGULAR EDIT/CREATE MODE ═══════════════
      // loaded_agent_nodes_[current_edit_idx_] = new_node;
      // Ensure the index is within bounds for all arrays
      if (current_edit_idx_ >= 0 && 
          current_edit_idx_ < static_cast<int>(loaded_agent_nodes_.size()) &&
          current_edit_idx_ < static_cast<int>(loaded_agent_names_.size()) &&
          current_edit_idx_ < static_cast<int>(loaded_agent_goals_.size()))
      {
        // Safe to update the existing agent
        loaded_agent_nodes_[current_edit_idx_] = new_node;
        
        // Update the agent name if needed (for consistency in CREATE mode)
        if (panel_mode_ == CREATE_MODE)
        {
          std::string agent_name = "agent" + std::to_string(agent_id);
          loaded_agent_names_[current_edit_idx_] = agent_name;
        }
      }
      else
      {
        // Index out of bounds - this shouldn't happen, but handle gracefully
        RCLCPP_ERROR(get_logger(), 
                     "Agent index %d is out of bounds (loaded_agent_nodes_.size()=%zu)",
                     current_edit_idx_, loaded_agent_nodes_.size());
        window->close();
        actor_button_->setDown(false);
        actor_button_->setChecked(false);
        QMessageBox::critical(this, "Error", 
                              "Internal error: Agent index out of bounds. Please restart the panel.");
        return;
      }
    }
    
    // ────────────────────── NORMAL NAVIGATION ────────────────────────
    if (panel_mode_ == EDIT_MODE && current_edit_idx_ + 1 < static_cast<int>(loaded_agent_names_.size()))
    {
      // Move to next agent in edit mode
      current_edit_idx_++;
      window->close();
      addAgent();
      return;
    }
    else if (panel_mode_ == CREATE_MODE && agent_count < num_agents)
    {
      // Move to next agent in create mode
      
      // Ensure we have enough space in all arrays for CREATE mode
      while (loaded_agent_nodes_.size() < static_cast<size_t>(num_agents))
      {
        loaded_agent_nodes_.push_back(YAML::Node());
      }
      while (loaded_agent_names_.size() < static_cast<size_t>(num_agents))
      {
        loaded_agent_names_.push_back("");
      }
      while (loaded_agent_goals_.size() < static_cast<size_t>(num_agents))
      {
        loaded_agent_goals_.push_back(std::vector<int>());
      }
      
      agent_count++;
      current_edit_idx_ = agent_count - 1; // Update current_edit_idx_ to match agent_count
      window->close();
      addAgent();
      return;
    }

    // ─────────────────────── ALL AGENTS DONE ────────────────────────────────
    if (simulator_connection) {
      disconnect(simulator_connection);
      simulator_connection = QMetaObject::Connection();
    }
    window->close();
    actor_button_->setDown(false);
    actor_button_->setChecked(false);
    if (panel_mode_ == EDIT_MODE)
    {
      assign_goals_btn_->setEnabled(!loaded_global_goals_.empty());
    }
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

    // ──────────────────────── Cancel button ──────────────────────────
    auto *dialog_cancel_button = new QPushButton("Cancel", window);
    dialog_cancel_button->setStyleSheet(
        "QPushButton {"
        "  padding: 6px 10px;"
        "  border: 2px solid #e74c3c;"
        "  border-radius: 6px;"
        "  background-color: #fdf2f2;"
        // "  color: #c0392b;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #f5b7b1;"
        "  color: #ffffff;"
        "  border-color: #c0392b;"
        "}");
    topic_layout->addWidget(dialog_cancel_button);

    connect(dialog_cancel_button, &QPushButton::clicked, [this]()
            {
                if (adding_new_agent_)
                {
                  // Just reset the flag - no data was added to clean up
                  adding_new_agent_ = false;
                  
                  // Reset counts to original values
                  num_agents = static_cast<int>(loaded_agent_names_.size());
                  if (num_agents > 0)
                  {
                    current_edit_idx_ = num_agents - 1;
                  }
                  else
                  {
                    current_edit_idx_ = 0;
                  }
                  
                  // Resize marker IDs array back
                  loaded_initial_marker_ids_.resize(num_agents, -1);
                }

                static QMetaObject::Connection simulator_connection;
                if (simulator_connection) {
                    disconnect(simulator_connection);
                    simulator_connection = QMetaObject::Connection();
                }
                
                window->close();
                actor_button_->setDown(false);
                actor_button_->setChecked(false);
                add_agent_button_->setEnabled(true);
                add_agent_button_->setDown(false);
                add_agent_button_->setChecked(false); });

    // ────────────────────── POPULATE FORM FIELDS ─────────────────────────────────

    if (adding_new_agent_)
    {
      // ═══════════════════ NEW AGENT - USE DEFAULTS ═══════════════════
      agent_desired_vel->setText("1.5");
      behavior_type_combobox->setCurrentText("Regular");
      behavior_conf_combobox->setCurrentText("Default");
      beh_gff->setText("2.0");
      beh_off->setText("10.0");
      beh_sff->setText("5.0");
      beh_otherff->setText("20.0");
      beh_dist->setText("3.0");
      beh_duration->setText("5.0");
      beh_once->setText("false");
      beh_vel->setText("1.0");
      beh_stop_dist->setText("1.5");
      beh_front_dist->setText("1.0");
      beh_safe_distance->setText("5.0");

      cyclic_goals_checkbox->setChecked(true);

      skin_combobox->setCurrentIndex(0);

      initial_pose_set = false;
      save_button_->setEnabled(false);
    }
    else if (panel_mode_ == EDIT_MODE &&
             current_edit_idx_ >= 0 &&
             current_edit_idx_ < static_cast<int>(loaded_agent_nodes_.size()))
    {
      // ═══════════════════ EXISTING AGENT - READ FROM YAML ═══════════════════
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

      // Load custom behavior parameters if configuration is Custom
      if (conf == hunav_msgs::msg::AgentBehavior::BEH_CONF_CUSTOM && 
          agentYAML["behavior"])
      {
        const YAML::Node &behYAML = agentYAML["behavior"];
        
        if (behYAML["duration"])
          beh_duration->setText(QString::number(behYAML["duration"].as<double>(), 'f', 1));
        if (behYAML["once"])
          beh_once->setText(behYAML["once"].as<bool>() ? "true" : "false");
        if (behYAML["vel"])
          beh_vel->setText(QString::number(behYAML["vel"].as<double>(), 'f', 3));
        if (behYAML["dist"])
          beh_dist->setText(QString::number(behYAML["dist"].as<double>(), 'f', 2));
        if (behYAML["goal_force_factor"])
          beh_gff->setText(QString::number(behYAML["goal_force_factor"].as<double>(), 'f', 1));
        if (behYAML["obstacle_force_factor"])
          beh_off->setText(QString::number(behYAML["obstacle_force_factor"].as<double>(), 'f', 1));
        if (behYAML["social_force_factor"])
          beh_sff->setText(QString::number(behYAML["social_force_factor"].as<double>(), 'f', 1));
        if (behYAML["other_force_factor"])
          beh_otherff->setText(QString::number(behYAML["other_force_factor"].as<double>(), 'f', 1));
        if (behYAML["stop_dist"])
          beh_stop_dist->setText(QString::number(behYAML["stop_dist"].as<double>(), 'f', 2));
        if (behYAML["front_dist"])
          beh_front_dist->setText(QString::number(behYAML["front_dist"].as<double>(), 'f', 2));
        if (behYAML["safe_distance"])
          beh_safe_distance->setText(QString::number(behYAML["safe_distance"].as<double>(), 'f', 1));
      }

      // — Skin —
      if (agentYAML["skin"])
      {
        try
        {
          int skinIndex = agentYAML["skin"].as<int>();
          skin_combobox->setCurrentIndex(skinIndex);
        }
        catch (...)
        {
          // Default to first option if parsing fails
          skin_combobox->setCurrentIndex(0);
          RCLCPP_WARN(get_logger(), "Failed to parse skin value, defaulting to index 0");
        }

        skin_label_->setVisible(true);
        skin_combobox->setVisible(true);
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

      // — Cyclic goals —
      if (agentYAML["cyclic_goals"])
      {
        bool is_cyclic = agentYAML["cyclic_goals"].as<bool>();
        cyclic_goals_checkbox->setChecked(is_cyclic);
      }
      else
      {
        cyclic_goals_checkbox->setChecked(true); // Default to true if not specified
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

      skin_label_->setVisible(true);
      skin_combobox->setVisible(true);

      cyclic_goals_checkbox->setChecked(true);
    }

    if (adding_new_agent_)
    {
      save_button_->setText(tr("Add agent"));
    }
    else if (panel_mode_ == CREATE_MODE)
    {
      bool last = (agent_count >= num_agents);
      save_button_->setText(last ? tr("Finish") : tr("Next agent"));
    }
    else // EDIT_MODE
    {
      bool last = (current_edit_idx_ + 1 >= num_agents);
      save_button_->setText(last ? tr("Finish") : tr("Next agent"));
    }

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
    int idx;
    if (panel_mode_ == EDIT_MODE || adding_new_agent_)
    {
      idx = current_edit_idx_;
    }
    else // CREATE_MODE
    {
      idx = agent_count - 1;
    }

    if (idx < 0 || idx >= (int)loaded_initial_marker_ids_.size())
    {
      RCLCPP_ERROR(get_logger(),
                   "onInitialPose(): bogus agent index %d", idx);
      return;
    }

    // 1) If in EDIT_MODE, delete both the old mesh and its label for this agent:
    if (panel_mode_ == EDIT_MODE && !adding_new_agent_)
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
    if (adding_new_agent_)
    {
      // For new agents, show the ID that will be assigned (current list size + 1)
      label.text = std::to_string(static_cast<int>(loaded_agent_names_.size()) + 1);
    }
    else
    {
      // For existing agents, show their actual 1-based index
      label.text = std::to_string(idx + 1);
    }
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

    QString simulatorName = simulator_combo_->currentText();
    QString packageName;

    if (simulatorName == "Gazebo Classic")
    {
      packageName = "hunav_gazebo_wrapper";
    }
    else if (simulatorName == "Gazebo Fortress")
    {
      packageName = "hunav_gazebo_fortress_wrapper";
    }
    else if (simulatorName == "Isaac Sim")
    {
      packageName = "hunav_isaac_wrapper";
    }
    else
    { // Webots
      packageName = "hunav_webots_wrapper";
    }

      QString baseDir;
      try
      {
        QString shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory(packageName.toStdString()));
        std::string srcDir = this->share_to_src_path(shareDir.toStdString());
        baseDir = QString::fromStdString(srcDir + "/maps");
      }
      catch (const std::exception &e)
      {
        QString homePath = QDir::homePath() + "/" + packageName + "/maps";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/" + packageName + "/maps";
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
    if (resp && resp->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS)
    {
      // Enhanced success message
      QMessageBox success_msg(this);
      success_msg.setWindowTitle("Map Loaded Successfully");
      success_msg.setText(QString(
                              "<html>"
                              "<h3>Map loaded successfully!</h3>"
                              "<p><b>File:</b> %1</p>"
                              "<p><b>Status:</b> Ready to configure agents</p>"
                              "</html>")
                              .arg(QFileInfo(yaml).fileName()));
      success_msg.setIcon(QMessageBox::Information);
      success_msg.setStyleSheet(
          "QMessageBox {"
          "  background-color: #d5f4e6;"
          "}"
          "QMessageBox QLabel {"
          // "  color: #1e8449;"
          "}");
      success_msg.exec();

      // Update UI state
      panel_mode_ = CREATE_MODE;
      actors->setEnabled(true);
      n_agents_label_->setEnabled(true);
      current_map_label_->setText(QFileInfo(yaml).fileName());
      current_map_label_->setStyleSheet(
          "QLabel {"
          "  padding: 4px 6px;"
          "  border: 1px solid #27ae60;"
          "  border-radius: 4px;"
          "  color: #1e8449;"
          "  font-weight: bold;"
          "}");
    }
    else
    {
      // Enhanced error message
      QMessageBox error_msg(this);
      error_msg.setWindowTitle("Map Loading Failed");
      error_msg.setText(QString(
                            "<html>"
                            "<h3>Failed to load map</h3>"
                            "<p><b>File:</b> %1</p>"
                            "<p><b>Error:</b> Map server request failed</p>"
                            "<p>Please check that the map_server is running and try again.</p>"
                            "</html>")
                            .arg(yaml));
      error_msg.setIcon(QMessageBox::Critical);
      error_msg.setStyleSheet(
          "QMessageBox {"
          "  background-color: #fdf2f2;"
          "}"
          "QMessageBox QLabel {"
          "  color: #c0392b;"
          "}");
      error_msg.exec();
    }
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
                        "When you’re done, click <b><i>%1</i></b> and then <b><i>%2</i></b>."
                        "</html>")
                        .arg(enter_goal_mode_btn_->text())
                        .arg(assign_goals_btn_->text());

      QMessageBox::information(
          this,
          tr("Goal-Picking Mode"),
          msg);
    }

    // ── Tool management - switch tools based on mode ──
    if (auto *tm = getDisplayContext()->getToolManager())
    {
      if (goal_picking_mode_)
      {
        // Entering goal picking mode - switch to PublishPoint tool
        for (int i = 0; i < tm->numTools(); ++i)
        {
          rviz_common::Tool *tool = tm->getTool(i);
          if (QString(tool->getClassId()) == "rviz_default_plugins/PublishPoint")
          {
            tm->setCurrentTool(tool);
            break;
          }
        }
      }
      else
      {
        // exiting goal picking mode - switch to Interact tool
        for (int i = 0; i < tm->numTools(); ++i)
        {
          rviz_common::Tool *tool = tm->getTool(i);
          if (QString(tool->getClassId()) == "rviz_default_plugins/Interact")
          {
            tm->setCurrentTool(tool);
            break;
          }
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
    // 1) Build and position a tool-style dialog with enhanced styling
    QString msg = QString(
        "<html><head><style>"
        "body { font-family: 'Segoe UI', Arial, sans-serif; margin: 8px; }"
        ".highlight { color: #3498db; font-weight: bold; }"
        ".action { color: #27ae60; font-weight: bold; }"
        ".warning { color: #e67e22; font-weight: bold; }"
        "</style></head><body>"

        "<h3 style='color: #2c3e50; margin-bottom: 8px;'>Goal Assignment Guide</h3>"

        "<p>You are about to <span class='highlight'>assign navigation goals</span> to your agents.</p>"

        "<p><strong>Steps:</strong></p>"
        "<ol style='margin-left: 16px;'>"
        "<li>Select an agent from the dropdown</li>"
        "<li>Pick goals from the <span class='action'>Available Goals</span> list</li>"
        "<li>Use <span class='action'>▶</span> to assign or <span class='action'>◀</span> to remove goals</li>"
        "<li>Click <span class='action'>Lock Selection</span> to confirm each agent's route</li>"
        "<li>Click <span class='action'>Finish</span> when all assignments are complete</li>"
        "</ol>"

        "<p><span class='warning'>⚠️ Navigation Tip:</span> Ensure goal connections have clear paths (no obstacles) for optimal navigation.</p>"

        "</body></html>");

    QMessageBox info_box(this);
    info_box.setWindowTitle("Goal Assignment");
    info_box.setText(msg);
    info_box.setIcon(QMessageBox::Information);
    info_box.setStyleSheet(
        "QMessageBox {"
        "  background-color: #f8f9fa;"
        "  border: 2px solid #3498db;"
        "  border-radius: 8px;"
        "}"
        "QMessageBox QLabel {"
        "  color: #2c3e50;"
        "  padding: 4px;"
        "}");
    info_box.exec();

    // Create the main dialog with enhanced styling
    QDialog dlg(this);
    int panelWidth = this->width();
    dlg.setFixedWidth(panelWidth);
    dlg.setWindowFlags(dlg.windowFlags() | Qt::Tool);
    dlg.setWindowTitle("Assign Goals to Agents");
    dlg.setStyleSheet(
        "QDialog {"
        // "  background-color: #f8f9fa;"
        "  border: 2px solid #3498db;"
        "  border-radius: 8px;"
        "}"
        "QLabel {"
        "  color: #2c3e50;"
        "  font-weight: bold;"
        "  font-size: 13px;"
        "}"
        "QComboBox {"
        "  padding: 6px 10px;"
        "  border: 2px solid #bdc3c7;"
        "  border-radius: 6px;"
        // "  background-color: white;"
        "  font-size: 14px;"
        "  font-weight: bold;"
        // "  min-height: 20px;"
        "}"
        "QComboBox:focus {"
        "  border-color: #3498db;"
        "}"
        // "QComboBox::drop-down {"
        // // "  width: 20px;"
        // "  border-left: 2px solid #bdc3c7;"
        // "  border-radius: 0 6px 6px 0;"
        // "  background-color: #ecf0f1;"
        // "  color:rgb(0, 0, 0);"
        // "}"
        // "QComboBox::down-arrow {"
        // "  image: url(:/icons/down-arrow.svg);"
        // "  border-left: 5px solid transparent;"
        // "  border-right: 5px solid transparent;"
        // "  border-top: 5px solid #7f8c8d;"
        // "  margin-right: 5px;"
        // "}"
        "QListWidget {"
        "  border: 2px solid #bdc3c7;"
        "  border-radius: 6px;"
        "  background-color: white;"
        "  padding: 4px;"
        "  font-size: 13px;"
        "  selection-background-color: #3498db;"
        "}"
        "QListWidget::item {"
        "  padding: 6px 8px;"
        "  border-bottom: 1px solid #ecf0f1;"
        "  border-radius: 4px;"
        "  margin: 1px;"
        "  color: #2c3e50;"
        "}"
        "QListWidget::item:hover {"
        "  background-color: #ebf3fd;"
        "}"
        "QListWidget::item:selected {"
        "  background-color: #3498db;"
        "  color: white;"
        "  font-weight: bold;"
        "}"
        "QPushButton {"
        "  padding: 8px 12px;"
        "  border: 2px solid #27ae60;"
        "  border-radius: 6px;"
        "  background-color: #d5f4e6;"
        // "  color: #1e8449;"
        "  font-weight: bold;"
        "  font-size: 13px;"
        // "  min-width: 80px;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color: #a9dfbf;"
        "  border-color: #1e8449;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #27ae60;"
        "  color: white;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #f8f9fa;"
        "  color: #aeb6bf;"
        "  border-color: #d5dbdb;"
        "}"
        "QCheckBox {"
        "  color: #2c3e50;"
        "  font-weight: bold;"
        "  font-size: 13px;"
        "}"
        "QCheckBox::indicator {"
        // "  width: 18px;"
        "  height: 18px;"
        "  border: 2px solid #bdc3c7;"
        "  border-radius: 4px;"
        "  background-color: white;"
        "}"
        "QCheckBox::indicator:checked {"
        "  background-color: #3498db;"
        "  border-color: #3498db;"
        "}"
        "QCheckBox::indicator:checked::after {"
        "  content: '✓';"
        "  color: white;"
        "  font-weight: bold;"
        "}"
        "QGroupBox {"
        "  font-weight: bold;"
        "  color:rgb(0, 0, 0);"
        "  margin-top: 8px;"
        "  padding-top: 4px;"
        "  border: 2px solid #3498db;"
        "  border-radius: 6px;"
        "  background-color: #ebf3fd;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  subcontrol-position: top left;"
        "  padding: 0 4px;"
        "  background-color: #d6eaf8;"
        "  border: 1px solid #3498db;"
        "  border-radius: 4px;"
        "}");

    if (auto *lay = dlg.layout())
    {
      lay->setSizeConstraint(QLayout::SetMinimumSize);
    }

    // assign_goals_btn_->setDown(true);

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

    // 2) Main layout with enhanced spacing
    auto *main_layout = new QVBoxLayout(&dlg);
    // main_layout->setSpacing(4);
    // main_layout->setContentsMargins(8, 8, 8, 8);
    dlg.setLayout(main_layout);
    main_layout->setSizeConstraint(QLayout::SetMinimumSize);

    // Header section with styled title
    QLabel *header_label = new QLabel("Goal Assignment Manager");
    header_label->setStyleSheet(
        "QLabel {"
        "  font-size: 16px;"
        "  font-weight: bold;"
        "  color:rgb(0, 0, 0);"
        "  background-color: #ebf3fd;"
        "  padding: 8px;"
        "  border-radius: 6px;"
        "  border: 2px solid #3498db;"
        "}");
    header_label->setAlignment(Qt::AlignCenter);
    main_layout->addWidget(header_label);

    // Agent selector section in a group box
    QGroupBox *agent_group = new QGroupBox("Agent Selection");
    QVBoxLayout *agent_layout = new QVBoxLayout;

    QLabel *agent_label = new QLabel("Select agent to configure:");
    auto *agent_sel = new QComboBox;
    // agent_sel->setStyleSheet("QComboBox { font-size: 14px; font-weight: bold; }");

    for (int i = 0; i < int(agent_goals_.size()); ++i)
    {
      QString label = QString("Agent %1").arg(i + 1);
      agent_sel->addItem(label);
    }

    agent_layout->addWidget(agent_label);
    agent_layout->addWidget(agent_sel);
    agent_group->setLayout(agent_layout);
    main_layout->addWidget(agent_group);

    // Goals management section
    QGroupBox *goals_group = new QGroupBox("Goal Management");
    QVBoxLayout *goals_layout = new QVBoxLayout;

    // — Available vs Assigned lists + buttons with enhanced titles
    auto *titles = new QHBoxLayout;
    QLabel *avail_title = new QLabel("Available Goals");
    QLabel *assigned_title = new QLabel("Assigned Goals");
    avail_title->setStyleSheet("QLabel { color: #e67e22; font-size: 14px; }");
    assigned_title->setStyleSheet("QLabel { color: #27ae60; font-size: 14px; }");

    titles->addWidget(avail_title);
    titles->addStretch();
    titles->addWidget(assigned_title);
    goals_layout->addLayout(titles);

    auto *lists_layout = new QHBoxLayout;
    auto *avail_list = new QListWidget;
    auto *assigned_list = new QListWidget;

    // Enhanced button styling for arrow buttons
    auto *btn_layout = new QVBoxLayout;
    auto *add_btn = new QPushButton("▶ Assign");
    auto *remove_btn = new QPushButton("◀ Remove");

    add_btn->setStyleSheet(
        "QPushButton {"
        "  background-color: #d5f4e6;"
        "  border-color: #27ae60;"
        "  color: #1e8449;"
        "  font-size: 14px;"
        "  padding: 10px 16px;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color: #a9dfbf;"
        "  border-color: #1e8449;"
        "}");
    remove_btn->setStyleSheet(
        "QPushButton {"
        "  background-color: #fdf2f2;"
        "  border-color: #e74c3c;"
        "  color: #c0392b;"
        "  font-size: 14px;"
        "  padding: 10px 16px;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color:rgb(233, 176, 166);"
        "  border-color:rgb(228, 64, 46);"
        "}");

    btn_layout->addStretch();
    btn_layout->addWidget(add_btn);
    btn_layout->addSpacing(8);
    btn_layout->addWidget(remove_btn);
    btn_layout->addStretch();

    lists_layout->addWidget(avail_list);
    lists_layout->addLayout(btn_layout);
    lists_layout->addWidget(assigned_list);
    goals_layout->addLayout(lists_layout);
    goals_group->setLayout(goals_layout);
    main_layout->addWidget(goals_group);

    // Control buttons section
    QGroupBox *control_group = new QGroupBox("Actions");
    QVBoxLayout *control_layout = new QVBoxLayout;

    // — Lock Selection button with enhanced styling
    auto *lock_btn = new QPushButton("Lock Agent Configuration");
    lock_btn->setEnabled(false);
    lock_btn->setStyleSheet(
        "QPushButton {"
        "  background-color: #fff3cd;"
        "  border-color: #ffc107;"
        // "  color: #856404;"
        "  font-size: 14px;"
        "  padding: 8px 16px;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color: #ffeaa7;"
        "  border-color: #f39c12;"
        "}");

    // — Show/Hide Arrows checkbox with enhanced styling
    auto *show_arrows_checkbox = new QCheckBox("Show navigation arrows on map");
    show_arrows_checkbox->setChecked(true);
    show_arrows_checkbox->setToolTip("Toggle visibility of agent navigation arrows and route preview");
    show_arrows_checkbox->setStyleSheet(
        "QCheckBox {"
        "  font-size: 14px;"
        "  color: #2c3e50;"
        "}");

    control_layout->addWidget(lock_btn);
    control_layout->addWidget(show_arrows_checkbox);
    control_group->setLayout(control_layout);
    main_layout->addWidget(control_group);

    // — Summary area for locked agents with enhanced styling
    QGroupBox *summary_group = new QGroupBox("Assignment Summary");
    summary_group->setStyleSheet(
        "QGroupBox {"
        "  background-color:rgb(229, 236, 234);"
        "  border-color:rgb(39, 174, 122);"
        "}");
    auto *summary_area = new QVBoxLayout;
    summary_group->setLayout(summary_area);
    main_layout->addWidget(summary_group);

    // keep track of which agents have been "locked"
    QVector<bool> lockedFlags(num_actors_, false);

    // Dialog buttons with enhanced styling
    QDialogButtonBox *button_box = new QDialogButtonBox(&dlg);
    button_box->setStyleSheet(
        "QDialogButtonBox QPushButton {"
        // "  min-width: 100px;"
        "  padding: 10px 20px;"
        "  font-size: 14px;"
        "  font-weight: bold;"
        "}");

    finishBtn_ = button_box->addButton("Finish Assignment", QDialogButtonBox::AcceptRole);
    auto *cancelBtn = button_box->addButton("Cancel", QDialogButtonBox::RejectRole);

    finishBtn_->setStyleSheet(
        "QPushButton {"
        "  background-color: #d5f4e6;"
        "  border-color: #27ae60;"
        // "  color: #1e8449;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color: #a9dfbf;"
        "  border-color: #1e8449;"
        "}");
    cancelBtn->setStyleSheet(
        "QPushButton {"
        "  background-color: #fdf2f2;"
        "  border-color: #e74c3c;"
        // "  color: #c0392b;"
        "}"
        "QPushButton:hover:enabled {"
        "  background-color: #f8d7da;"
        "  border-color: #e74c3c;"
        "}");

    main_layout->addWidget(button_box);
    finishBtn_->setEnabled(false);

    finishBtn_->setEnabled(panel_mode_ == EDIT_MODE);

    connect(finishBtn_, &QAbstractButton::clicked, this, [this]()
            {
      resetGoalMarkerColors();
      goal_markers_pub_->publish(goal_markers_); });

    // — Show/Hide arrows checkbox handler
    bool arrows_visible = true; // Track arrow visibility state

    // Function to create/update arrows for a specific agent
    auto updateAgentArrows = [&](int agent_idx)
    {
      if (agent_idx < 0 || agent_idx >= (int)loaded_agent_nodes_.size())
        return;

      // Clear existing arrows and goal cubes for this agent
      auto delete_arrows = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.header.frame_id = "/map";
      delete_marker.header.stamp = rclcpp::Clock().now();
      delete_marker.ns = "agent_arrow";
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_arrows->markers.push_back(delete_marker);

      // Also delete goal cubes
      visualization_msgs::msg::Marker delete_cubes;
      delete_cubes.header.frame_id = "/map";
      delete_cubes.header.stamp = rclcpp::Clock().now();
      delete_cubes.ns = "agent_goal";
      delete_cubes.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_arrows->markers.push_back(delete_cubes);

      initial_pose_publisher->publish(std::move(delete_arrows));

      if (!arrows_visible || agent_goals_[agent_idx].empty())
        return;

      // Get agent's initial position
      const YAML::Node &agentYAML = loaded_agent_nodes_[agent_idx];
      if (!agentYAML["init_pose"])
        return;

      double init_x = agentYAML["init_pose"]["x"].as<double>();
      double init_y = agentYAML["init_pose"]["y"].as<double>();

      // Check if this agent has cyclic navigation enabled
      bool is_cyclic = true;
      if (agentYAML["cyclic_goals"])
      {
        is_cyclic = agentYAML["cyclic_goals"].as<bool>();
      }

      // Get agent color
      QColor agent_color = agent_colors_[agent_idx];

      auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
      int marker_id = next_marker_id_ + 1000 + agent_idx * 100; // Unique ID range for arrows and cubes

      geometry_msgs::msg::Point prev_pt;
      prev_pt.x = init_x;
      prev_pt.y = init_y;
      prev_pt.z = 0.0;

      // Create arrows and cubes between consecutive goals
      for (size_t i = 0; i < agent_goals_[agent_idx].size(); ++i)
      {
        int gid = agent_goals_[agent_idx][i];
        auto it = loaded_global_goals_.find(gid);
        if (it == loaded_global_goals_.end())
          continue;

        geometry_msgs::msg::Point goal_pt = it->second;

        // Create small colored cube at goal_pt
        visualization_msgs::msg::Marker goal_cube = createMarker(
            goal_pt.x, goal_pt.y, marker_id++, "cube", "parser");
        goal_cube.ns = "agent_goal";
        goal_cube.color.r = agent_color.redF();
        goal_cube.color.g = agent_color.greenF();
        goal_cube.color.b = agent_color.blueF();
        goal_cube.color.a = 1.0f;
        markers->markers.push_back(goal_cube);

        // Create arrow from prev_pt to goal_pt
        visualization_msgs::msg::Marker arrow = createArrowMarker(
            prev_pt.x, prev_pt.y, goal_pt.x, goal_pt.y, marker_id++);
        arrow.ns = "agent_arrow";
        arrow.color.r = agent_color.redF();
        arrow.color.g = agent_color.greenF();
        arrow.color.b = agent_color.blueF();
        arrow.color.a = 1.0f;
        markers->markers.push_back(arrow);

        prev_pt = goal_pt;
      }

      // Create closing arrow from last goal back to first goal (if more than 1 goal and cyclic_goals == true)
      if (agent_goals_[agent_idx].size() > 1 && is_cyclic)
      {
        int first_gid = agent_goals_[agent_idx][0];
        auto it_first = loaded_global_goals_.find(first_gid);
        if (it_first != loaded_global_goals_.end())
        {
          const auto &first_pt = it_first->second;
          visualization_msgs::msg::Marker closing_arrow = createArrowMarker(
              prev_pt.x, prev_pt.y, first_pt.x, first_pt.y, marker_id++);
          closing_arrow.ns = "agent_arrow";
          closing_arrow.color.r = agent_color.redF();
          closing_arrow.color.g = agent_color.greenF();
          closing_arrow.color.b = agent_color.blueF();
          closing_arrow.color.a = 1.0f;
          markers->markers.push_back(closing_arrow);
        }
      }

      if (!markers->markers.empty())
      {
        initial_pose_publisher->publish(std::move(markers));
      }
    };

    connect(show_arrows_checkbox, &QCheckBox::toggled, this, [&](bool show_arrows)
            {
              arrows_visible = show_arrows;
              
              if (!show_arrows) {
                // Delete all arrows
                auto delete_arrows = std::make_unique<visualization_msgs::msg::MarkerArray>();
                visualization_msgs::msg::Marker delete_marker;
                delete_marker.header.frame_id = "/map";
                delete_marker.header.stamp = rclcpp::Clock().now();
                delete_marker.ns = "agent_arrow";
                delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
                delete_arrows->markers.push_back(delete_marker);
                initial_pose_publisher->publish(std::move(delete_arrows));
              } else {
                // Recreate arrows for all agents
                for (int i = 0; i < num_actors_; ++i) {
                  updateAgentArrows(i);
                }
              } });

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
        assigned_list->addItem(QString("Goal %1").arg(gid));
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
            
            // 4) Update arrows for the selected agent
            updateAgentArrows(new_agent_idx);

            // 5) repaint the combo text in that agent's color
            QPalette pal = agent_sel->palette();
            pal.setColor(QPalette::Text, c);
            agent_sel->setPalette(pal);

            // 6) rebuild the available/assigned lists
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
            
            // Update arrows for the current agent
            updateAgentArrows(a);
            refresh(); });

    // — Remove goal from agent
    connect(remove_btn, &QPushButton::clicked, this, [&]()
            {
    int a = agent_sel->currentIndex();
    auto &vec = agent_goals_[a];

    for (auto* it : assigned_list->selectedItems()) {
        // parse out the 1-based goal ID directly
        int gid = it->text().split(' ').last().toInt();

        // remove it from this agent's assignments
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
    
    // Update arrows for the current agent
    updateAgentArrows(a);

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
          
          // Enhanced summary row with better styling
          QStringList goal_strs;
          for (int gi : agent_goals_[a]) goal_strs << QString::number(gi);
          QString joined = goal_strs.join(", ");
          auto *row = new QHBoxLayout;
          auto *sq  = new QLabel;
          auto *lbl = new QLabel(QString("Agent %1 → Goals: [%2]").arg(a+1).arg(joined));
          
          lbl->setStyleSheet(
            "QLabel {"
            "  padding: 6px 8px;"
            "  background-color: #d5f4e6;"
            "  border: 1px solid #27ae60;"
            "  border-radius: 4px;"
            "  font-weight: bold;"
            // "  color: #1e8449;"
            "}");
          
          QPixmap pix(8,8); 
          pix.fill(agent_colors_[a]);
          sq->setPixmap(pix); 
          sq->setFixedSize(8,8);
          sq->setStyleSheet("border: 1px solid #bdc3c7; border-radius: 2px;");
          
          row->addWidget(sq); 
          row->addWidget(lbl); 
          // row->addStretch();
          summary_area->insertLayout(a, row);
          
          // recolor all that agent's goals
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
          
          // Update arrows for the locked agent
          updateAgentArrows(a);

          // Update window geometry and resize
          QTimer::singleShot(0, this, [this, &dlg]()
          {
              dlg.adjustSize();
              dlg.updateGeometry(); 
          });

          bool allLocked = std::all_of(
            lockedFlags.begin(), lockedFlags.end(),
            [](bool v){ return v; });
          finishBtn_->setEnabled(panel_mode_ == EDIT_MODE || allLocked); });

    // — Dialog buttons
    connect(button_box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(button_box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

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

      // Update arrows for the initial agent
      updateAgentArrows(a);
    }

    dlg.adjustSize();

    QPoint panelTL = this->mapToGlobal(QPoint(0, 0));
    int x = panelTL.x() + (this->width() - dlg.width()) / 2;
    dlg.move(x, panelTL.y());

    // Initial populate & execute
    refresh();
    dlg.exec();

    if (dlg.result() != QDialog::Accepted)
      return;

    // assign_goals_btn_->setDown(false);

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
      // assign_goals_btn_->setDown(false);
      save_bt_btn_->setEnabled(true);
      checkbox->setEnabled(true);
      resetGoalMarkerColors();
      goal_markers_pub_->publish(goal_markers_);

      // Enhanced completion message
      QMessageBox success_box(this);
      success_box.setWindowTitle("Goal Assignment Complete");
      success_box.setText(QString(
                              "<html><head><style>"
                              "body { font-family: 'Segoe UI', Arial, sans-serif; margin: 8px; }"
                              ".highlight { color: #27ae60; font-weight: bold; }"
                              "</style></head><body>"

                              "<h3 style='color: #2c3e50; margin-bottom: 8px;'>Goals Successfully Assigned!</h3>"

                              "<p>All agents now have their navigation goals configured.</p>"

                              "<p>Next step: Click <span class='highlight'>%1</span> to generate the final configuration files and behavior trees.</p>"

                              "</body></html>")
                              .arg(save_bt_btn_->text()));
      success_box.setIcon(QMessageBox::Information);
      success_box.setStyleSheet(
          "QMessageBox {"
          "  background-color: #e8f5e8;"
          "  border: 2px solid #27ae60;"
          "  border-radius: 8px;"
          "}"
          "QMessageBox QLabel {"
          "  color: #2c3e50;"
          "  padding: 4px;"
          "}");
      success_box.exec();
    }
    else
    {
      // EDIT: copy back from the dialog's temporary arrays
      loaded_agent_goals_ = agent_goals_;
      save_bt_btn_->setEnabled(true);
      checkbox->setEnabled(true);

      // Enhanced update message
      QMessageBox update_box(this);
      update_box.setWindowTitle("Goals Updated");
      update_box.setText(QString(
                             "<html><head><style>"
                             "body { font-family: 'Segoe UI', Arial, sans-serif; margin: 8px; }"
                             ".highlight { color: #27ae60; font-weight: bold; }"
                             "</style></head><body>"

                             "<h3 style='color: #2c3e50; margin-bottom: 8px;'>Goal Assignment Updated!</h3>"

                             "<p>Your agent goal assignments have been successfully updated and saved in memory.</p>"

                             "<p>To finalize the changes, click <span class='highlight'>%1</span> to write out the updated YAML and regenerate behavior trees.</p>"

                             "</body></html>")
                             .arg(save_bt_btn_->text()));
      update_box.setIcon(QMessageBox::Information);
      update_box.setStyleSheet(
          "QMessageBox {"
          "  background-color: #ebf3fd;"
          "  border: 2px solid #3498db;"
          "  border-radius: 8px;"
          "}"
          "QMessageBox QLabel {"
          "  color: #2c3e50;"
          "  padding: 4px;"
          "}");
      update_box.exec();
    }

    assign_goals_btn_->setDown(false);
    assign_goals_btn_->setChecked(false);
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
    if (!agent_goals_.empty())
    {
      for (auto &vec : agent_goals_)
      {
        vec.clear();
      }
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

    {
      auto arr = std::make_unique<visualization_msgs::msg::MarkerArray>();
      delete_all.ns = "agent_arrow";
      arr->markers.push_back(delete_all);
      initial_pose_publisher->publish(std::move(arr));
    }
    {
      auto arr = std::make_unique<visualization_msgs::msg::MarkerArray>();
      delete_all.ns = "agent_goal";
      arr->markers.push_back(delete_all);
      initial_pose_publisher->publish(std::move(arr));
    }

    // Also clear our local copy of “goal_markers_” so future onGoalPicked() starts clean:
    goal_markers_.markers.clear();

    // 3) Clear the QListWidget
    goal_list_widget_->clear();

    // 4) Disable “Assign goals” and reset goal id tracking
    moving_goal_id_ = -1; // Reset goal ID tracking
    assign_goals_btn_->setEnabled(false);

    QMessageBox::information(this,
                             tr("Goals Reset"),
                             tr("All previously-loaded goals have been cleared.\n"
                                "You may now pick new goals on the map."));
  }

  void ActorPanel::onCreateOrEditAgents()
  {
    if (adding_new_agent_)
    {
      return;
    }

    actor_button_->setDown(true);
    actor_button_->setChecked(true);

    if (panel_mode_ == CREATE_MODE)
    {
      // Regular CREATE_MODE workflow
      num_actors_ = actors->text().toInt();
      initAgentColors(num_actors_);
      agent_goals_.assign(num_actors_, {});
      loaded_initial_marker_ids_.assign(num_actors_, -1);
      next_marker_id_ = 0;

      // Clear the loaded lists for fresh creation
      loaded_agent_names_.clear();
      loaded_agent_nodes_.clear();
      loaded_agent_goals_.clear();

      addAgent();
      return;
    }
    else if (panel_mode_ == EDIT_MODE)
    {
      // Regular EDIT_MODE workflow - validate data first
      if (loaded_agent_names_.empty())
      {
        QMessageBox::warning(this, "No Agents", "No agents loaded to edit.");
        actor_button_->setDown(false);
        actor_button_->setChecked(false);
        return;
      }

      clearNonAgentMarkers();
      publishAgentMarkers();
      current_edit_idx_ = 0;
      assign_goals_btn_->setEnabled(!loaded_global_goals_.empty());
      addAgent();
      return;
    }
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
    QString packageName;

    if (simulatorName == "Gazebo Classic")
    {
      packageName = "hunav_gazebo_wrapper";
    }
    else if (simulatorName == "Gazebo Fortress")
    {
      packageName = "hunav_gazebo_fortress_wrapper";
    }
    else if (simulatorName == "Isaac Sim")
    {
      packageName = "hunav_isaac_wrapper";
    }
    else
    { // Webots
      packageName = "hunav_webots_wrapper";
    }

      QString configDir, mapDir;
      try
      {
        QString shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory(packageName.toStdString()));
        std::string srcDir = this->share_to_src_path(shareDir.toStdString());
        configDir = QString::fromStdString(srcDir + "/scenarios");
        mapDir = QString::fromStdString(srcDir + "/maps");
      }
      catch (const std::exception &e)
      {
        QString homePath = QDir::homePath() + "/" + packageName;
        QString dockerPath = "/workspace/hunav_isaac_ws/src/" + packageName;
        QString baseDir = QDir(dockerPath).exists() ? dockerPath : homePath;
        configDir = baseDir + "/scenarios";
        mapDir = baseDir + "/maps";
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

      // Determine the base maps directory using simplified ROS2 package logic
      QString simulatorName = simulator_combo_->currentText();
      QString packageName;

      if (simulatorName == "Gazebo Classic")
      {
        packageName = "hunav_gazebo_wrapper";
      }
      else if (simulatorName == "Gazebo Fortress")
      {
        packageName = "hunav_gazebo_fortress_wrapper";
      }
      else if (simulatorName == "Isaac Sim")
      {
        packageName = "hunav_isaac_wrapper";
      }
      else
      { // Webots
        packageName = "hunav_webots_wrapper";
      }

        QString mapDir;
        try
        {
          QString shareDir = QString::fromStdString(
              ament_index_cpp::get_package_share_directory(packageName.toStdString()));
          std::string srcDir = this->share_to_src_path(shareDir.toStdString());
          mapDir = QString::fromStdString(srcDir + "/maps");
          RCLCPP_INFO(get_logger(), "Found ROS2 package '%s', maps at: %s",
                      packageName.toStdString().c_str(), mapDir.toStdString().c_str());
        }
        catch (const std::exception &e)
        {
          // Fallback to development paths only if package not found
          RCLCPP_WARN(get_logger(), "ROS2 package '%s' not found, falling back to development paths",
                      packageName.toStdString().c_str());

        QString homePath = QDir::homePath() + "/" + packageName + "/maps";
        QString dockerPath = "/workspace/hunav_isaac_ws/src/" + packageName + "/maps";

        mapDir = QDir(dockerPath).exists() ? dockerPath : homePath;
        RCLCPP_INFO(get_logger(), "Using fallback path: %s", mapDir.toStdString().c_str());
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

        // arrow from prev_pt → goal_pt (same color)
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

      // 9e) Finally, draw an arrow from the last goal back to the first goal (only if cyclic)
      if (!assigned_goals.empty())
      {
        // Check if this agent has cyclic navigation
        bool is_cyclic = true;
        if (agent_node["cyclic_goals"])
        {
          is_cyclic = agent_node["cyclic_goals"].as<bool>();
        }

        if (is_cyclic)
        {
          // get the first goal's coordinates
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
      switchButtonLayout(EDIT_MODE);
      current_edit_idx_ = 0;
      actors->hide();
      actor_button_->setText("Edit agents");
      actor_button_->setEnabled(true);
      edit_goals_button_->setVisible(true);
      edit_goals_button_->setDown(false);
      add_agent_button_->setVisible(true);
      add_agent_button_->show();
      add_agent_button_->setEnabled(true);

      n_agents_label_->hide();
      map_group->setTitle("Edit agents or navigation goal:");
      map_group->setEnabled(false);
      map_group->setVisible(false);
      // goal_group_->setTitle("");
      goal_group_->setEnabled(true);
      goal_group_->show();
      goal_group_->update();
      map_select_btn_->hide();
      current_map_label_->hide();
      map_select_btn_->setVisible(false);

      // *** ENABLE GOAL MANAGEMENT UI ELEMENTS ***
      reset_goals_button_->setEnabled(true);
      reset_goals_button_->setVisible(true);
      reset_goals_button_->show();
      reset_goals_button_->raise();
      goal_list_widget_->setEnabled(true);
      goal_list_widget_->setVisible(true);
      goal_list_widget_->show();
      goal_list_widget_->raise();

      enter_goal_mode_btn_->hide();
    }

    // Create a rich completion message
    QString completionTitle = tr("Agents Configuration Loaded");
    QString completionMsg = QString(
                                "<html><head><style>"
                                "body { font-family: 'Segoe UI', Arial, sans-serif; margin: 8px; }"
                                ".header { color: #3498db; font-weight: bold; font-size: 14px; margin-bottom: 5px; }"
                                ".section { margin: 8px 0; }"
                                ".filename { background-color: #ecf0f1; padding: 2px 4px; border-radius: 4px; "
                                "           font-family: 'Consolas', monospace; color: #2c3e50; font-weight: bold; }"
                                ".summary { background-color: #e8f6ff; padding: 4px; border-radius: 6px; "
                                "          border-left: 4px solid #3498db; margin-top: 8px; }"
                                ".action { color: #27ae60; font-weight: bold; }"
                                "</style></head><body>"

                                "<div class='header'>Agent Configuration Successfully Loaded!</div>"

                                "<div class='section'>"
                                "<strong>Source:</strong> <span class='filename'>%1</span><br>"
                                "<strong>Agents:</strong> %2 agents loaded with complete configuration"
                                "</div>"

                                "<div class='summary'>"
                                "<strong>Steps:</strong><br>"
                                "• <span class='action'>Edit agent configurations</span> using the agent panel<br>"
                                "• <span class='action'>Modify navigation goals</span> by entering goal editing mode<br>"
                                "• <span class='action'>Save and generate</span> updated behavior trees when ready"
                                "</div>"

                                "</body></html>")
                                .arg(orig_yaml_base_name_)
                                .arg(loaded_agent_names_.size());

    // Create and style the message box
    QMessageBox msgBox(this);
    msgBox.setWindowTitle(completionTitle);
    msgBox.setText(completionMsg);
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setStyleSheet(
        "QMessageBox {"
        "  background-color: #f8f9fa;"
        "  border: 2px solid #3498db;"
        "  border-radius: 8px;"
        "}"
        "QMessageBox QLabel {"
        "  color: #2c3e50;"
        "  padding: 4px;"
        "}");
    msgBox.exec();
  }

  void ActorPanel::onAddAgent()
  {
    // Set the flag to indicate we're adding a new agent
    adding_new_agent_ = true;

    // Calculate what the new agent's index will be (after current agents)
    current_edit_idx_ = static_cast<int>(loaded_agent_names_.size());

    // Update num_agents to include the slot for the new agent
    num_agents = current_edit_idx_ + 1;

    // Ensure marker IDs array is large enough for the new agent
    // loaded_initial_marker_ids_.resize(num_agents, -1);
    if (loaded_initial_marker_ids_.size() < static_cast<size_t>(num_agents))
    {
      loaded_initial_marker_ids_.resize(num_agents, -1);
    }

    // Update agent colors for the new total count
    initAgentColors(num_agents);

    // Initialize pose tracking
    initial_pose_set = false;

    assign_goals_btn_->setEnabled(false);

    // Pop up the agent configuration dialog
    addAgent();
  }

  void ActorPanel::launchBTWizard()
  {
    if (loaded_agent_names_.empty())
    {
      QMessageBox::information(this, "No Agents Available",
                               "Please create or load agents before configuring behavior trees.\n\n"
                               "Use 'Generate Agents' or 'Edit Existing Configuration' first.");
      return;
    }

    BTConfigDialog wizard(this);

    // Set up agent data
    QStringList agentNames, behaviorTypes;
    for (size_t i = 0; i < loaded_agent_names_.size(); ++i)
    {
      agentNames << QString::fromStdString(loaded_agent_names_[i]);

      QString bt = "Regular";
      try
      {
        if (i < loaded_agent_nodes_.size() &&
            loaded_agent_nodes_[i]["behavior"] &&
            loaded_agent_nodes_[i]["behavior"]["type"])
        {
          bt = QString::fromStdString(loaded_agent_nodes_[i]["behavior"]["type"].as<std::string>());
        }
      }
      catch (...)
      {
        // Use default
      }
      behaviorTypes << bt;
    }

    wizard.setAgentList(agentNames, behaviorTypes);

    // Pass available goal IDs to the wizard
    QList<int> goalIds;
    for (int goalId : goal_ids_)
    {
      goalIds.append(goalId);
    }
    wizard.setAvailableGoals(goalIds);
    
    // Try to load existing BT configuration from XML files
    QString scenarioName = panel_mode_ == EDIT_MODE ? orig_yaml_base_name_ : yaml_base_name_;
    QString simulator = simulator_combo_->currentText();
    QStringList btPaths = BTConfigDialog::generateBTPathsForScenario(
        scenarioName, simulator, loaded_agent_names_.size());
    
    // Attempt to load existing configuration if files exist
    bool configLoaded = wizard.loadExistingConfiguration(btPaths);
    if (configLoaded) {
      qDebug() << "Loaded existing BT configuration from XML files";
    } else {
      qDebug() << "Starting with fresh BT configuration (no existing files or failed to parse)";
    }

    if (wizard.exec() == QDialog::Accepted)
    {
      auto config = wizard.getConfig();

      // Use BTConfigDialog's static validation method
      if (!BTConfigDialog::validateWizardConfig(config, loaded_agent_names_.size()))
      {
        QMessageBox::warning(this, "Invalid Configuration",
                             "The behavior tree configuration is invalid. Please check your settings.");
        return;
      }

      // Generate BT file paths using BTConfigDialog's method
      QString scenarioName = panel_mode_ == EDIT_MODE ? orig_yaml_base_name_ : yaml_base_name_;
      QString simulator = simulator_combo_->currentText();
      QStringList btPaths = BTConfigDialog::generateBTPathsForScenario(
          scenarioName, simulator, loaded_agent_names_.size());

      // Apply configuration using BTConfigDialog's method
      if (BTConfigDialog::patchAllAgentBTFiles(btPaths, config))
      {
        save_bt_btn_->setEnabled(true);
        QMessageBox::information(this, "Configuration Complete",
                                 "Behavior tree configuration has been successfully applied to all agents.");
      }
      else
      {
        QMessageBox::warning(this, "Configuration Error",
                             "Some behavior tree files could not be updated. Check the console for details.");
      }
    }
  }

  void ActorPanel::launchLLMBTGenerator()
  {
    // Check if agents are loaded
    if (loaded_agent_names_.empty())
    {
      QMessageBox::warning(
          this,
          "No Agents Loaded",
          "Please load agents from a YAML file before generating behavior trees.");
      return;
    }

    // Determine the YAML file path based on panel mode
    QString yamlFilePath;

    if (panel_mode_ == EDIT_MODE)
    {
      // In EDIT mode, use the loaded file path
      if (pkg_shared_tree_dir_.empty())
      {
        QMessageBox::warning(
            this,
            "No Scenario File",
            "No scenario YAML file is currently loaded. Please load a scenario first.");
        return;
      }
      yamlFilePath = QString::fromStdString(pkg_shared_tree_dir_);
    }
    else if (panel_mode_ == CREATE_MODE)
    {
      // In CREATE mode, construct the path from yaml_base_name_
      if (yaml_base_name_.isEmpty())
      {
        QMessageBox::warning(
            this,
            "No Scenario Saved",
            "Please save your scenario configuration before generating behavior trees.\n"
            "Use the 'Save agents & generate BTs' button first.");
        return;
      }

      // Determine the simulator wrapper package
      QString simulatorName = simulator_combo_->currentText();
      QString packageName;

      if (simulatorName == "Gazebo Classic")
      {
        packageName = "hunav_gazebo_wrapper";
      }
      else if (simulatorName == "Gazebo Fortress")
      {
        packageName = "hunav_gazebo_fortress_wrapper";
      }
      else if (simulatorName == "Isaac Sim")
      {
        packageName = "hunav_isaac_wrapper";
      }
      else
      { // Webots
        packageName = "hunav_webots_wrapper";
      }

      // Get the config directory
      QString configDir;
      try
      {
        QString shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory(packageName.toStdString()));
        std::string srcDir = this->share_to_src_path(shareDir.toStdString());
        configDir = QString::fromStdString(srcDir + "/scenarios");
      }
      catch (const std::exception &e)
      {
        QString homePath = QDir::homePath() + "/" + packageName;
        QString dockerPath = "/workspace/hunav_isaac_ws/src/" + packageName;
        QString baseDir = QDir(dockerPath).exists() ? dockerPath : homePath;
        configDir = baseDir + "/scenarios";
      }

      yamlFilePath = configDir + "/" + yaml_base_name_ + ".yaml";
    }
    else
    {
      QMessageBox::warning(
          this,
          "Invalid Mode",
          "Cannot determine panel mode. Please load or create a scenario first.");
      return;
    }

    // Verify the YAML file exists
    if (!QFile::exists(yamlFilePath))
    {
      QMessageBox::critical(
          this,
          "File Not Found",
          QString("The scenario YAML file does not exist:\n%1\n\n"
                  "Please save your scenario configuration before generating behavior trees.")
              .arg(yamlFilePath));
      return;
    }

    RCLCPP_INFO(get_logger(), "Launching LLM BT Generator for scenario: %s",
                yamlFilePath.toStdString().c_str());

    // Build the ROS2 command string
    QString ros2Command = QString("ros2 run hunav_behavior_tree_generator generator_cli --mode scenario --yaml-file %1 --simulator \"%2\"")
                              .arg(QFileInfo(yamlFilePath).fileName())
                              .arg(simulator_combo_->currentText());

    // Detect available terminal emulator and build the command to open a new terminal window
    QString terminalCommand;
    QStringList terminalArgs;

    // Try common terminal emulators in order of preference
    QProcess checkTerminal;
    
    // Check for gnome-terminal
    checkTerminal.start("which", QStringList() << "gnome-terminal");
    checkTerminal.waitForFinished(1000);
    if (checkTerminal.exitCode() == 0)
    {
      terminalCommand = "gnome-terminal";
      terminalArgs << "--" << "bash" << "-c" 
                   << QString("%1; echo ''; echo 'Press Enter to close this window...'; read").arg(ros2Command);
    }
    else
    {
      // Check for xterm
      checkTerminal.start("which", QStringList() << "xterm");
      checkTerminal.waitForFinished(1000);
      if (checkTerminal.exitCode() == 0)
      {
        terminalCommand = "xterm";
        terminalArgs << "-hold" << "-e" << ros2Command;
      }
      else
      {
        // Check for konsole (KDE)
        checkTerminal.start("which", QStringList() << "konsole");
        checkTerminal.waitForFinished(1000);
        if (checkTerminal.exitCode() == 0)
        {
          terminalCommand = "konsole";
          terminalArgs << "-e" << "bash" << "-c" 
                       << QString("%1; echo ''; echo 'Press Enter to close this window...'; read").arg(ros2Command);
        }
        else
        {
          // Check for xfce4-terminal
          checkTerminal.start("which", QStringList() << "xfce4-terminal");
          checkTerminal.waitForFinished(1000);
          if (checkTerminal.exitCode() == 0)
          {
            terminalCommand = "xfce4-terminal";
            terminalArgs << "-e" << "bash -c '" + ros2Command + "; echo ''; echo 'Press Enter to close...'; read'";
          }
          else
          {
            // Fallback error
            QMessageBox::critical(
                this,
                "Terminal Not Found",
                "Could not find a suitable terminal emulator (gnome-terminal, xterm, konsole, or xfce4-terminal).\n\n"
                "Please install one of these terminal emulators to use this feature.");
            return;
          }
        }
      }
    }

    // Show information dialog before starting
    QMessageBox::information(
        this,
        "Launching LLM BT Generator",
        QString("<html>Opening a new terminal window to run LLM-based behavior tree generation for:\n\n"
                "<b>%1</b>\n\n"
                "The generator will run interactively in the terminal.\n"
                "Please follow the prompts in the terminal window.</html>")
            .arg(QFileInfo(yamlFilePath).fileName()));

    // Launch the terminal with the command
    RCLCPP_INFO(get_logger(), "Executing in terminal: %s %s",
                terminalCommand.toStdString().c_str(),
                terminalArgs.join(" ").toStdString().c_str());

    bool success = QProcess::startDetached(terminalCommand, terminalArgs);

    if (success)
    {
      RCLCPP_INFO(get_logger(), "Successfully launched LLM BT Generator in new terminal window");
      QMessageBox::information(
          this,
          "Terminal Launched",
          "The LLM BT Generator has been launched in a new terminal window.\n\n"
          "Please interact with it in that terminal to generate behavior trees.");
    }
    else
    {
      QString errorMsg = QString("Failed to launch terminal command:\n%1 %2")
                             .arg(terminalCommand)
                             .arg(terminalArgs.join(" "));
      RCLCPP_ERROR(get_logger(), "%s", errorMsg.toStdString().c_str());
      QMessageBox::critical(
          this,
          "Launch Failed",
          errorMsg);
    }
  }
  

  void ActorPanel::resetBTConfiguration(const QStringList &btPaths)
  {
    // Validate that agents are loaded
    if (loaded_agent_names_.empty())
    {
      QMessageBox::warning(this, "No Agents Loaded",
                           "No agents are currently loaded. Please load or create agents first.");
      return;
    }

    // Confirm with user
    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        "Reset Behavior Trees",
        QString("This will reset all %1 agent behavior trees to their predefined configurations.\n\n"
                "Any current custom BT configurations made will be lost.\n\n"
                "Do you want to continue?")
            .arg(loaded_agent_names_.size()),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);

    if (reply != QMessageBox::Yes)
    {
      return;
    }

    try
    {
      int successCount = 0;
      int failCount = 0;
      QStringList errors;

      // Use provided paths or generate them if not provided
      QStringList filePaths = btPaths;
      if (filePaths.isEmpty())
      {
        QString scenarioName = panel_mode_ == EDIT_MODE ? orig_yaml_base_name_ : yaml_base_name_;
        QString simulator = simulator_combo_->currentText();
        
        if (scenarioName.isEmpty())
        {
          QMessageBox::warning(this, "Invalid Configuration",
                               "No scenario name available. Please save your configuration first.");
          return;
        }
        
        filePaths = BTConfigDialog::generateBTPathsForScenario(
            scenarioName, simulator, loaded_agent_names_.size());
      }

      // Reset each agent's BT to its predefined configuration
      for (size_t i = 0; i < loaded_agent_names_.size() && i < loaded_agent_nodes_.size(); ++i)
      {
        // Determine the behavior type for this agent
        QString behaviorType = "Regular"; // Default
        try
        {
          if (loaded_agent_nodes_[i]["behavior"] &&
              loaded_agent_nodes_[i]["behavior"]["type"])
          {
            behaviorType = QString::fromStdString(
                loaded_agent_nodes_[i]["behavior"]["type"].as<std::string>());
          }
        }
        catch (...)
        {
          // Use default
        }

        // Generate the predefined BT for this agent
        QString btContent = generateDefaultBTForAgent(i);
        
        if (btContent.isEmpty())
        {
          QString error = QString("Agent %1: Failed to generate predefined BT for behavior '%2'")
                              .arg(i + 1)
                              .arg(behaviorType);
          errors.append(error);
          failCount++;
          continue;
        }

        // Use the pre-computed file path
        QString btFilePath = (static_cast<int>(i) < filePaths.size()) ? filePaths[i] : QString();
        if (btFilePath.isEmpty())
        {
          QString error = QString("Agent %1: No file path available").arg(i + 1);
          errors.append(error);
          failCount++;
          continue;
        }
        
        // Ensure the directory exists
        QFileInfo fileInfo(btFilePath);
        QDir dir = fileInfo.absoluteDir();
        if (!dir.exists())
        {
          if (!dir.mkpath("."))
          {
            QString error = QString("Agent %1: Failed to create directory: %2")
                                .arg(i + 1)
                                .arg(dir.absolutePath());
            errors.append(error);
            failCount++;
            continue;
          }
        }

        // Write the BT file
        QFile file(btFilePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
          QString error = QString("Agent %1: Failed to write BT file: %2")
                              .arg(i + 1)
                              .arg(btFilePath);
          errors.append(error);
          failCount++;
          continue;
        }

        QTextStream out(&file);
        out << btContent;
        file.close();

        RCLCPP_INFO(this->get_logger(),
                    "Reset BT for agent %zu (%s) to predefined behavior '%s'",
                    i + 1,
                    loaded_agent_names_[i].c_str(),
                    behaviorType.toStdString().c_str());
        successCount++;
      }

      // Clear any cached wizard configuration
      agentBTConfig_.clear();
      lastWizardConfig_ = BTConfigDialog::Config();

      // Show results to user
      QString message;
      if (failCount == 0)
      {
        message = QString("Successfully reset all %1 agent behavior trees to their predefined configurations.")
                      .arg(successCount);
        QMessageBox::information(this, "Reset Complete", message);
      }
      else if (successCount > 0)
      {
        message = QString("Reset %1 agent(s) successfully, but %2 failed.\n\nErrors:\n%3")
                      .arg(successCount)
                      .arg(failCount)
                      .arg(errors.join("\n"));
        QMessageBox::warning(this, "Partial Reset", message);
      }
      else
      {
        message = QString("Failed to reset behavior trees.\n\nErrors:\n%1")
                      .arg(errors.join("\n"));
        QMessageBox::critical(this, "Reset Failed", message);
      }

      RCLCPP_INFO(this->get_logger(),
                  "Reset BT configuration: %d succeeded, %d failed",
                  successCount, failCount);
    }
    catch (const std::exception &e)
    {
      QString errorMsg = QString("Exception during BT reset: %1").arg(e.what());
      QMessageBox::critical(this, "Reset Error", errorMsg);
      RCLCPP_ERROR(this->get_logger(), "%s", errorMsg.toStdString().c_str());
    }
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
                           le->setCursorPosition(defaultName_.length()); });
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

    // Determine the simulator wrapper directories
    QString simulatorName = simulator_combo_->currentText();
    QString packageName;

    if (simulatorName == "Gazebo Classic")
    {
      packageName = "hunav_gazebo_wrapper";
    }
    else if (simulatorName == "Gazebo Fortress")
    {
      packageName = "hunav_gazebo_fortress_wrapper";
    }
    else if (simulatorName == "Isaac Sim")
    {
      packageName = "hunav_isaac_wrapper";
    }
    else
    { // Webots
      packageName = "hunav_webots_wrapper";
    }

      QString configDir, btDir;
      try
      {
        QString shareDir = QString::fromStdString(
            ament_index_cpp::get_package_share_directory(packageName.toStdString()));
        std::string srcDir = this->share_to_src_path(shareDir.toStdString());
        configDir = QString::fromStdString(srcDir + "/scenarios");
        btDir = QString::fromStdString(srcDir + "/behavior_trees");
      }
      catch (const std::exception &e)
      {
        QString homePath = QDir::homePath() + "/" + packageName;
        QString dockerPath = "/workspace/hunav_isaac_ws/src/" + packageName;
        QString baseDir = QDir(dockerPath).exists() ? dockerPath : homePath;
        configDir = baseDir + "/scenarios";
        btDir = baseDir + "/behavior_trees";
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
                "Wrote agents yaml file to %s",
                fullpath.toStdString().c_str());

    // Generate BT for each agent
    agentBtPaths_.clear();

    for (int i = 0; i < (int)loaded_agent_names_.size(); ++i)
    {
      // 4a) Generate default/template BT
      QString outXml = generateDefaultBTForAgent(i);

      // 4b) Write to disk
      QString fname = btDir + QString("/%1__agent_%2_bt.xml")
                                  .arg(yaml_base_name_)
                                  .arg(i + 1);
      agentBtPaths_.push_back(fname);

      QFile file(fname);
      if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
      {
        QMessageBox::warning(this, "Write Error",
                             "Could not write file:\n" + fname);
        continue;
      }
      file.write(outXml.toUtf8());
      file.close();
    }

    // Log the BT generation
    QString title;
    QString verbYaml;
    QString verbBT;
    if (panel_mode_ == CREATE_MODE)
    {
      title = tr("Agents Configuration Complete");
      verbYaml = tr("Created new agents YAML");
      verbBT = tr("generated");
    }
    else
    {
      title = tr("Agents Configuration Updated");
      verbYaml = tr("Updated agents YAML");
      verbBT = tr("re-generated");
    }

    // Create a rich, informative completion message
    QString msg = QString(
                      "<html><head><style>"
                      "body { font-family: 'Segoe UI', Arial, sans-serif; margin: 8px; }"
                      ".header { color: #27ae60; font-weight: bold; font-size: 14px; margin-bottom: 2px; }"
                      ".section { margin: 8px 0; }"
                      ".filename { background-color: #ecf0f1; padding: 4px 8px; border-radius: 4px; "
                      "           font-family: 'Consolas', monospace; color: #2c3e50; font-weight: bold; }"
                      ".path { font-size: 13px; color:rgb(139, 141, 141); margin-top: 2px; }"
                      ".summary { background-color: #e8f5e8; padding: 4px; border-radius: 6px; "
                      "          border-left: 4px solid #27ae60; margin-top: 2px; }"
                      "</style></head><body>"

                      "<div class='header'>%1 Successfully!</div>"

                      "<div class='section'>"
                      "<strong>YAML Configuration:</strong><br>"
                      "<span class='filename'>%2</span>"
                      "<div class='path'>%3</div>"
                      "</div>"

                      "<div class='section'>"
                      "<strong>Behavior Trees:</strong><br>"
                      "%4 <strong>%5 BT files</strong> for individual agent behaviors"
                      "<div class='path'>%6</div>"
                      "</div>"

                      "<div class='summary'>"
                      "Configuration ready for <strong>%7 agents</strong><br>"
                      "You can now launch your simulation with the %8 simulator!"
                      "</div>"

                      "</body></html>")
                      .arg(verbYaml.contains("new") ? "Configuration Created" : "Configuration Updated")
                      .arg(outName)
                      .arg(configDir)
                      .arg(verbBT.at(0).toUpper() + verbBT.mid(1)) // Capitalize first letter
                      .arg(loaded_agent_names_.size())
                      .arg(btDir)
                      .arg(loaded_agent_names_.size())
                      .arg(simulator_combo_->currentText());

    // Create and style the message box
    QMessageBox msgBox(this);
    msgBox.setWindowTitle(title);
    msgBox.setText(msg);
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setStyleSheet(
        "QMessageBox {"
        "  background-color: #f8f9fa;"
        "  border: 2px solid #27ae60;"
        "  border-radius: 8px;"
        "}"
        "QMessageBox QLabel {"
        "  color: #2c3e50;"
        "  padding: 4px;"
        "}");
    msgBox.exec();
    bt_group_->setEnabled(true);
  }

  QString ActorPanel::generateDefaultBTForAgent(int agentIndex)
  {
    // 0) sanity check
    if (agentIndex < 0 || agentIndex >= (int)loaded_agent_nodes_.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid agent index: %d", agentIndex);
      return {};
    }

    // 1) Load TreeNodesModel.xml
    QString modelXml = loadFile("TreeNodesModel.xml");
    if (modelXml.isEmpty())
    {
      RCLCPP_ERROR(this->get_logger(), "Cannot open TreeNodesModel.xml");
      return {};
    }

    // 2) Pick the right .xml template to include and determine tree name
    static const QMap<QString, QString> templateMap = {
        {"Regular", "BTRegularNav.xml"},
        {"Impassive", "BTRegularNav.xml"},
        {"Scared", "BTScaredNav.xml"},
        {"Surprised", "BTSurprisedNav.xml"},
        {"Curious", "BTCuriousNav.xml"},
        {"Threatening", "BTThreateningNav.xml"}};

    // Map template files to their corresponding tree names
    static const QMap<QString, QString> treeNameMap = {
        {"BTRegularNav.xml", "RegularNavTree"},
        {"BTScaredNav.xml", "ScaredNavTree"},
        {"BTSurprisedNav.xml", "SurprisedNavTree"},
        {"BTCuriousNav.xml", "CuriousNavTree"},
        {"BTThreateningNav.xml", "ThreateningNavTree"}};

    auto behNode = loaded_agent_nodes_[agentIndex]["behavior"];
    QString type = behNode["type"]
                       ? QString::fromStdString(behNode["type"].as<std::string>())
                       : "Regular";
    QString tmpl = templateMap.value(type, "BTRegularNav.xml");
    QString treeName = treeNameMap.value(tmpl, "DefaultTree");

    // 3) Update the main_tree_to_execute attribute in TreeNodesModel.xml
    QString rootPattern = "<root main_tree_to_execute=\"DefaultTree\" BTCPP_format=\"4\">";
    QString newRootTag = QString("<root main_tree_to_execute=\"%1\" BTCPP_format=\"4\">").arg(treeName);
    modelXml.replace(rootPattern, newRootTag);

    const QString closingTag = "</TreeNodesModel>";
    int insertPos = modelXml.indexOf(closingTag);
    if (insertPos < 0)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "TreeNodesModel.xml missing closing tag");
      return {};
    }
    insertPos += closingTag.length();

    QString includeLine = QString("  <include path=\"BTRegularNav.xml\" />\n\n");
    QString goalsBlock = buildGoalSequence(agentIndex);

    // 4) Gather all placeholder substitutions
    QMap<QString, QString> subs = {
        {"{GOAL_SEQUENCE}", goalsBlock},
        // common
        {"{dist}", behNode["dist"] ? QString::number(behNode["dist"].as<double>()) : "5"},
        {"{duration}", behNode["duration"] ? QString::number(behNode["duration"].as<double>()) : "40"},
        {"{once}", behNode["once"] ? (behNode["once"].as<bool>() ? "true" : "false") : "false"},
        // extras
        {"{maxvel}", loaded_agent_nodes_[agentIndex]["max_vel"]
                         ? QString::number(loaded_agent_nodes_[agentIndex]["max_vel"].as<double>())
                         : "2.0"},
        {"{forcefactor}", behNode["other_force_factor"]
                              ? QString::number(behNode["other_force_factor"].as<double>())
                              : "20"},
        {"{stopdist}", behNode["stop_dist"]
                           ? QString::number(behNode["stop_dist"].as<double>())
                           : "1.5"},
        {"{frontdist}", behNode["front_dist"]
                            ? QString::number(behNode["front_dist"].as<double>())
                            : "1.5"},
        {"{safe_distance}", behNode["safe_distance"]
                            ? QString::number(behNode["safe_distance"].as<double>())
                            : "5.0"}};

    // 5) Now build the <BehaviorTree> snippet from the selected template file
    QString templateContent = loadFile(tmpl);
    if (templateContent.isEmpty())
    {
      RCLCPP_WARN(this->get_logger(),
                  "Could not load specific BT template: %s", tmpl.toStdString().c_str());
      return {};
    }

    int btPos = templateContent.indexOf("<BehaviorTree");
    if (btPos >= 0)
    {
      templateContent = templateContent.mid(btPos);
    }

    // substitute placeholders
    for (auto it = subs.constBegin(); it != subs.constEnd(); ++it)
    {
      templateContent.replace(it.key(), it.value());
    }

    // 6) Stitch everything back into the Model:
    QString result =
        modelXml.left(insertPos) + "\n" +
        includeLine +
        + "    " + templateContent;
        // templateContent + "\n" +
        // modelXml.mid(insertPos);

    RCLCPP_INFO(this->get_logger(),
                "Generated BT for agent %d with behavior '%s', template '%s', tree name '%s'",
                agentIndex + 1, type.toStdString().c_str(),
                tmpl.toStdString().c_str(), treeName.toStdString().c_str());

    return result;
  }

  QString ActorPanel::buildGoalSequence(int agentIndex)
  {
      const auto &goals = loaded_agent_goals_[agentIndex];
      
      QString xml;
      xml += "<Sequence name=\"SetGoals\">\n";
      
      if (goals.empty())
      {
          // Fallback single goal
          xml += "            <!-- no goals, defaulting to goal 1 -->\n";
          xml += "            <RunOnce>\n";
          xml += "              <SetGoal agent_id=\"{id}\" goal_id=\"1\"/>\n";
          xml += "            </RunOnce>\n";
      }
      else
      {
          // Process each goal
          for (size_t j = 0; j < goals.size(); ++j)
          {
              int gid = goals[j];
              
              if (j + 1 < goals.size())
              {
                  // Intermediate goals: just RunOnce
                  xml += "                <RunOnce>\n";
                  xml += QString("                    <SetGoal agent_id=\"{id}\" goal_id=\"%1\"/>\n").arg(gid);
                  xml += "                </RunOnce>\n";
              }
              else
              {
                  // Final goal: wrapped in Inverter
                  xml += "                <Inverter>\n";
                  xml += "                    <RunOnce>\n";
                  xml += QString("                        <SetGoal agent_id=\"{id}\" goal_id=\"%1\"/>\n").arg(gid);
                  xml += "                    </RunOnce>\n";
                  xml += "                </Inverter>\n";
              }
          }
      }
      
      xml += "            </Sequence>";
      
      return xml;
  }

  int ActorPanel::checkComboBox()
  {
    std::string aux = behavior_type_combobox->currentText().toStdString();

    // First, hide all specific behavior parameters
    dur->setVisible(false);
    beh_duration->setVisible(false);
    once->setVisible(false);
    beh_once->setVisible(false);
    vel->setVisible(false);
    beh_vel->setVisible(false);
    dist->setVisible(false);
    beh_dist->setVisible(false);
    other->setVisible(false);
    beh_otherff->setVisible(false);
    front->setVisible(false);
    beh_front_dist->setVisible(false);
    safe_dist->setVisible(false);
    beh_safe_distance->setVisible(false);
    stop->setVisible(false);
    beh_stop_dist->setVisible(false);

    // All behaviors show basic force parameters
    gff->setVisible(true);
    beh_gff->setVisible(true);
    off->setVisible(true);
    beh_off->setVisible(true);
    sff->setVisible(true);
    beh_sff->setVisible(true);

    if (aux.compare("Regular") == 0)
    {
      // Regular behavior: only basic force parameters (already set above)
      
      // Trigger window resize after widget visibility changes
      if (window && window->isVisible())
      {
        QTimer::singleShot(0, this, [this]()
                           {
              window->adjustSize();
              window->updateGeometry(); });
      }

      return hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
    }
    else if (aux.compare("Impassive") == 0)
    {
      // Impassive behavior: same as regular, only basic force parameters

      if (window && window->isVisible())
      {
        QTimer::singleShot(0, this, [this]()
                           {
              window->adjustSize();
              window->updateGeometry(); });
      }
      return hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE;
    }
    else if (aux.compare("Surprised") == 0)
    {
      // Surprised behavior: duration, once, distance (for IsRobotVisible)
      dur->setVisible(true);
      beh_duration->setVisible(true);
      once->setVisible(true);
      beh_once->setVisible(true);
      dist->setVisible(true);
      beh_dist->setVisible(true);

      if (window && window->isVisible())
      {
        QTimer::singleShot(0, this, [this]()
                           {
              window->adjustSize();
              window->updateGeometry(); });
      }

      return hunav_msgs::msg::AgentBehavior::BEH_SURPRISED;
    }
    else if (aux.compare("Scared") == 0)
    {
      // Scared behavior: duration, maxvel, safe_distance (for AvoidRobot), distance (for IsRobotVisible)
      dur->setVisible(true);
      beh_duration->setVisible(true);
      vel->setVisible(true);
      beh_vel->setVisible(true);
      safe_dist->setVisible(true);
      beh_safe_distance->setVisible(true);
      dist->setVisible(true);
      beh_dist->setVisible(true);

      if (window && window->isVisible())
      {
        QTimer::singleShot(0, this, [this]()
                           {
              window->adjustSize();
              window->updateGeometry(); });
      }

      return hunav_msgs::msg::AgentBehavior::BEH_SCARED;
    }
    else if (aux.compare("Curious") == 0)
    {
      // Curious behavior: duration, maxvel, distance, stop_distance (for ApproachRobot)
      dur->setVisible(true);
      beh_duration->setVisible(true);
      vel->setVisible(true);
      beh_vel->setVisible(true);
      dist->setVisible(true);
      beh_dist->setVisible(true);
      stop->setVisible(true);
      beh_stop_dist->setVisible(true);

      if (window && window->isVisible())
      {
        QTimer::singleShot(0, this, [this]()
                           {
              window->adjustSize();
              window->updateGeometry(); });
      }

      return hunav_msgs::msg::AgentBehavior::BEH_CURIOUS;
    }
    else
    {
      // Threatening behavior: duration, distance, front_distance (for BlockRobot)
      dur->setVisible(true);
      beh_duration->setVisible(true);
      dist->setVisible(true);
      beh_dist->setVisible(true);
      front->setVisible(true);
      beh_front_dist->setVisible(true);

      if (window && window->isVisible())
      {
        QTimer::singleShot(0, this, [this]()
                           {
              window->adjustSize();
              window->updateGeometry(); });
      }

      return hunav_msgs::msg::AgentBehavior::BEH_THREATENING;
    }
  }

  void ActorPanel::checkComboBoxConf()
  {
    int beh = checkComboBox();
    std::string conf = behavior_conf_combobox->currentText().toStdString();

    if (conf == "Default")
    {
      // Set default values for basic force parameters (common to all behaviors)
      if (simulator_combo_->currentText() == "Isaac Sim")
      {
        beh_gff->setText(QString::number(10.0));
        beh_off->setText(QString::number(2.0));
      }
      else // Gazebo Classic, Gazebo Fortress, or Webots
      {
        beh_gff->setText(QString::number(2.0));
        beh_off->setText(QString::number(10.0));
      }
      beh_gff->setEnabled(false);
      beh_off->setEnabled(false);
      beh_sff->setText(QString::number(5.0));
      beh_sff->setEnabled(false);

      // Set behavior-specific default values
      if (beh == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED) // surprised
      {
        beh_duration->setText(QString::number(30.0));
        beh_duration->setEnabled(false);
        beh_dist->setText(QString::number(4.0));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_SCARED) // scared
      {
        beh_duration->setText(QString::number(40.0));
        beh_duration->setEnabled(false);
        beh_vel->setText(QString::number(0.6));
        beh_vel->setEnabled(false);
        beh_safe_distance->setText(QString::number(5.0));
        beh_safe_distance->setEnabled(false);
        beh_dist->setText(QString::number(4.0));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS) // curious
      {
        beh_duration->setText(QString::number(40.0));
        beh_duration->setEnabled(false);
        beh_vel->setText(QString::number(1.0));
        beh_vel->setEnabled(false);
        beh_dist->setText(QString::number(10.0));
        beh_dist->setEnabled(false);
        beh_stop_dist->setText(QString::number(1.5));
        beh_stop_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_THREATENING) // threatening
      {
        beh_duration->setText(QString::number(40.0));
        beh_duration->setEnabled(false);
        beh_dist->setText(QString::number(4.0));
        beh_dist->setEnabled(false);
        beh_front_dist->setText(QString::number(1.4));
        beh_front_dist->setEnabled(false);
      }
    }
    else if (conf == "Custom")
    {
      // Enable and set placeholders for basic force parameters (common to all behaviors)
      beh_gff->setEnabled(true);
      beh_gff->setText("");
      if (simulator_combo_->currentText() == "Isaac Sim")
      {
        beh_gff->setPlaceholderText("[5.0 - 10.0]");
      }
      else
      {
        beh_gff->setPlaceholderText("[2.0 - 5.0]");
      }
      beh_gff->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

      beh_off->setEnabled(true);
      beh_off->setText("");
      if (simulator_combo_->currentText() == "Isaac Sim")
      {
        beh_off->setPlaceholderText("[0.5 - 5.0]");
      }
      else
      {
        beh_off->setPlaceholderText("[2.0 - 50.0]");
      }
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

      // Set behavior-specific custom placeholders based on visible parameters
      if (beh == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED) // surprised
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
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_SCARED) // scared
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

        beh_vel->setEnabled(true);
        beh_vel->setText("");
        beh_vel->setPlaceholderText("[0.4 - 1.8]");
        beh_vel->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

        beh_safe_distance->setEnabled(true);
        beh_safe_distance->setText("");
        beh_safe_distance->setPlaceholderText("[2.0 - 10.0]");
        beh_safe_distance->setStyleSheet(R"(
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
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS) // curious
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

        beh_stop_dist->setEnabled(true);
        beh_stop_dist->setText("");
        beh_stop_dist->setPlaceholderText("[0.5 - 3.0]");
        beh_stop_dist->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_THREATENING) // threatening
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

        beh_dist->setEnabled(true);
        beh_dist->setText("");
        beh_dist->setPlaceholderText("[0.5 - 15.0]");
        beh_dist->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");

        beh_front_dist->setEnabled(true);
        beh_front_dist->setText("");
        beh_front_dist->setPlaceholderText("[0.5 - 2.0]");
        beh_front_dist->setStyleSheet(R"(
                        QLineEdit::placeholder {
                          font-style: italic;
                          color: gray;
                        }
                      )");
      }
    }

    else if (conf == "Random-normal distribution")
    {
      // Generate random values (normal distribution)
      std::random_device rd;
      std::mt19937 gen(rd());
      
      // Set basic force parameters (common to all behaviors)
      if (simulator_combo_->currentText() == "Isaac Sim")
      {
        std::normal_distribution<> dis_gff{5.0, 1.5};
        double facGoal = dis_gff(gen);
        facGoal = (facGoal < 5.0) ? 5.0 : facGoal;
        beh_gff->setText(QString::number(facGoal));
        beh_gff->setEnabled(false);

        std::normal_distribution<> dis_off{2.0, 4.0};
        double facObstacle = dis_off(gen);
        facObstacle = (facObstacle < 0.5) ? 0.5 : facObstacle;
        beh_off->setText(QString::number(facObstacle));
        beh_off->setEnabled(false);
      }
      else
      {
        // Gazebo or Webots
        std::normal_distribution<> dis_gff{2.0, 1.5};
        double facGoal = dis_gff(gen);
        facGoal = (facGoal < 2.0) ? 2.0 : facGoal;
        beh_gff->setText(QString::number(facGoal));
        beh_gff->setEnabled(false);

        std::normal_distribution<> dis_off{10.0, 4.0};
        double facObstacle = dis_off(gen);
        facObstacle = (facObstacle < 2.0) ? 2.0 : facObstacle;
        beh_off->setText(QString::number(facObstacle));
        beh_off->setEnabled(false);
      }
      std::normal_distribution<> dis_sff{4.0, 3.5};
      double facSocial = dis_sff(gen);
      facSocial = (facSocial < 3.0) ? 3.0 : facSocial;
      beh_sff->setText(QString::number(facSocial));
      beh_sff->setEnabled(false);

      // Set behavior-specific random values
      if (beh == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED) // surprised
      {
        std::normal_distribution<> dis_dur(30.0, 15.0);
        double duration = dis_dur(gen);
        duration = (duration < 10.0) ? 10.0 : duration;
        beh_duration->setText(QString::number(duration));
        beh_duration->setEnabled(false);
        
        std::normal_distribution<> dis_detect_dist(4.0, 2.0);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_SCARED) // scared
      {
        std::normal_distribution<> dis_dur(40.0, 15.0);
        double duration = dis_dur(gen);
        duration = (duration < 10.0) ? 10.0 : duration;
        beh_duration->setText(QString::number(duration));
        beh_duration->setEnabled(false);
        
        std::normal_distribution<> dis_vel(0.6, 0.2);
        double vel = dis_vel(gen);
        vel = (vel < 0.4) ? 0.4 : vel;
        beh_vel->setText(QString::number(vel));
        beh_vel->setEnabled(false);
        
        std::normal_distribution<> dis_safe_distance(5.0, 1.5);
        double safe_distance = dis_safe_distance(gen);
        safe_distance = (safe_distance < 2.0) ? 2.0 : safe_distance;
        beh_safe_distance->setText(QString::number(safe_distance));
        beh_safe_distance->setEnabled(false);

        std::normal_distribution<> dis_detect_dist(4.0, 2.0);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS) // curious
      {
        std::normal_distribution<> dis_dur(40.0, 15.0);
        double duration = dis_dur(gen);
        duration = (duration < 10.0) ? 10.0 : duration;
        beh_duration->setText(QString::number(duration));
        beh_duration->setEnabled(false);
        
        std::normal_distribution<> dis_vel(1.0, 0.3);
        double vel = dis_vel(gen);
        vel = (vel < 0.4) ? 0.4 : vel;
        beh_vel->setText(QString::number(vel));
        beh_vel->setEnabled(false);
        
        std::normal_distribution<> dis_detect_dist(10.0, 3.0);
        double detection_distance = dis_detect_dist(gen);
        detection_distance = (detection_distance < 1.5) ? 1.5 : detection_distance;
        beh_dist->setText(QString::number(detection_distance));
        beh_dist->setEnabled(false);
        
        std::normal_distribution<> dis_stop_dist(1.5, 0.3);
        double stop_dist = dis_stop_dist(gen);
        stop_dist = (stop_dist < 0.5) ? 0.5 : stop_dist;
        beh_stop_dist->setText(QString::number(stop_dist));
        beh_stop_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_THREATENING) // threatening
      {
        std::normal_distribution<> dis_dur(40.0, 15.0);
        double duration = dis_dur(gen);
        duration = (duration < 10.0) ? 10.0 : duration;
        beh_duration->setText(QString::number(duration));
        beh_duration->setEnabled(false);
        
        std::normal_distribution<> dis_detect_dist(4.0, 2.0);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
        
        std::normal_distribution<> dis_front(1.4, 0.3);
        double front_dist = dis_front(gen);
        front_dist = (front_dist < 0.5) ? 0.5 : front_dist;
        beh_front_dist->setText(QString::number(front_dist));
        beh_front_dist->setEnabled(false);
      }
    }
    else
    {
      // Generate random values (uniform distribution)
      std::random_device rd;
      std::mt19937 gen(rd());
      
      // Set basic force parameters (common to all behaviors)
      std::uniform_real_distribution<> dis_gff(2.0, 5.0);
      double facGoal = dis_gff(gen);
      facGoal = (facGoal < 2.0) ? 2.0 : facGoal;
      beh_gff->setText(QString::number(facGoal));
      beh_gff->setEnabled(false);

      std::uniform_real_distribution<> dis_off(2.0, 50.0);
      double facObstacle = dis_off(gen);
      facObstacle = (facObstacle < 2.0) ? 2.0 : facObstacle;
      beh_off->setText(QString::number(facObstacle));
      beh_off->setEnabled(false);

      std::uniform_real_distribution<> dis_sff(4.0, 20.0);
      double facSocial = dis_sff(gen);
      facSocial = (facSocial < 4.0) ? 4.0 : facSocial;
      beh_sff->setText(QString::number(facSocial));
      beh_sff->setEnabled(false);

      // Set behavior-specific random values
      if (beh == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED) // surprised
      {
        std::uniform_real_distribution<> dis_dur(25.0, 60.0);
        beh_duration->setText(QString::number(dis_dur(gen)));
        beh_duration->setEnabled(false);
        
        std::uniform_real_distribution<> dis_detect_dist(2.0, 6.0);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_SCARED) // scared
      {
        std::uniform_real_distribution<> dis_dur(25.0, 60.0);
        beh_duration->setText(QString::number(dis_dur(gen)));
        beh_duration->setEnabled(false);
        
        std::uniform_real_distribution<> dis_vel(0.4, 1.0);
        double vel = dis_vel(gen);
        vel = (vel < 0.4) ? 0.4 : vel;
        beh_vel->setText(QString::number(vel));
        beh_vel->setEnabled(false);
        
        std::uniform_real_distribution<> dis_safe_distance(3.0, 8.0);
        double safe_distance = dis_safe_distance(gen);
        safe_distance = (safe_distance < 2.0) ? 2.0 : safe_distance;
        beh_safe_distance->setText(QString::number(safe_distance));
        beh_safe_distance->setEnabled(false);

        std::uniform_real_distribution<> dis_detect_dist(2.0, 6.0);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS) // curious
      {
        std::uniform_real_distribution<> dis_dur(25.0, 60.0);
        beh_duration->setText(QString::number(dis_dur(gen)));
        beh_duration->setEnabled(false);
        
        std::uniform_real_distribution<> dis_vel(0.6, 1.2);
        double vel = dis_vel(gen);
        vel = (vel < 0.4) ? 0.4 : vel;
        beh_vel->setText(QString::number(vel));
        beh_vel->setEnabled(false);
        
        std::uniform_real_distribution<> dis_detect_dist(5.0, 15.0);
        double detection_distance = dis_detect_dist(gen);
        detection_distance = (detection_distance < 1.5) ? 1.5 : detection_distance;
        beh_dist->setText(QString::number(detection_distance));
        beh_dist->setEnabled(false);
        
        std::uniform_real_distribution<> dis_stop_dist(1.0, 2.5);
        double stop_dist = dis_stop_dist(gen);
        stop_dist = (stop_dist < 0.5) ? 0.5 : stop_dist;
        beh_stop_dist->setText(QString::number(stop_dist));
        beh_stop_dist->setEnabled(false);
      }
      else if (beh == hunav_msgs::msg::AgentBehavior::BEH_THREATENING) // threatening
      {
        std::uniform_real_distribution<> dis_dur(25.0, 60.0);
        beh_duration->setText(QString::number(dis_dur(gen)));
        beh_duration->setEnabled(false);
        
        std::uniform_real_distribution<> dis_detect_dist(2.0, 6.0);
        double dist = dis_detect_dist(gen);
        dist = (dist < 1.5) ? 1.5 : dist;
        beh_dist->setText(QString::number(dist));
        beh_dist->setEnabled(false);
        
        std::uniform_real_distribution<> dis_front(0.5, 2.0);
        double front_dist = dis_front(gen);
        front_dist = (front_dist < 0.5) ? 0.5 : front_dist;
        beh_front_dist->setText(QString::number(front_dist));
        beh_front_dist->setEnabled(false);
      }
    }
    if (window && window->isVisible())
    {
      // Force the layout to recalculate and resize the window
      QTimer::singleShot(0, this, [this]()
                         {
            window->adjustSize();
            window->updateGeometry(); });
    }
  }

  int ActorPanel::checkComboBoxSkin()
  {
    QString currentSim = simulator_combo_->currentText();
    std::string skinName = skin_combobox->currentText().toStdString();
    int skinIndex = skin_combobox->currentIndex();

    if (currentSim == "Gazebo Classic" || currentSim == "Gazebo Fortress")
    {
      // Gazebo skin mapping (unchanged)
      if (skinName == "Elegant man")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
        return 0;
      }
      else if (skinName == "Casual man")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/casual_man.dae";
        return 1;
      }
      else if (skinName == "Elegant woman")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/elegant_woman.dae";
        return 2;
      }
      else if (skinName == "Regular man")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/regular_man.dae";
        return 3;
      }
      else if (skinName == "Worker man")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/worker_man.dae";
        return 4;
      }
      else if (skinName == "Blue jeans")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
        return 5;
      }
      else if (skinName == "Green t-shirt")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
        return 6;
      }
      else if (skinName == "Blue t-shirt")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
        return 7;
      }
      else if (skinName == "Red t-shirt")
      {
        person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
        return 8;
      }
    }
    else if (currentSim == "Isaac Sim")
    {
      // Isaac Sim skin mapping - use generic mesh for RViz display
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";

      return skinIndex;
    }
    else if (currentSim == "Webots")
    {
      // Webots skin mapping - use generic mesh for RViz display
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";

      return skinIndex;
    }
    else
    {
      // Default fallback
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
      return 0;
    }

    // Default return
    return 0;
  }

  void ActorPanel::checkParserSkin(int skin)
  {
    QString currentSim = simulator_combo_->currentText();

    if (currentSim == "Gazebo Classic" || currentSim == "Gazebo Fortress")
    {
      // Gazebo skin mapping (unchanged)
      if (skin == 0)
        person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
      else if (skin == 1)
        person_skin = "package://hunav_rviz2_panel/meshes/casual_man.dae";
      else if (skin == 2)
        person_skin = "package://hunav_rviz2_panel/meshes/elegant_woman.dae";
      else if (skin == 3)
        person_skin = "package://hunav_rviz2_panel/meshes/regular_man.dae";
      else if (skin == 4)
        person_skin = "package://hunav_rviz2_panel/meshes/worker_man.dae";
      else
        person_skin = "package://hunav_rviz2_panel/meshes/walk.dae";
    }
    else if (currentSim == "Isaac Sim" || currentSim == "Webots")
    {
      // For Isaac Sim and Webots, use generic mesh for RViz display
      // The actual skin index is stored in YAML and will be used by the simulator
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
    }
    else
    {
      // Default fallback
      person_skin = "package://hunav_rviz2_panel/meshes/elegant_man.dae";
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
    create_button_->setDown(false);
    open_button_->setChecked(false);
    open_button_->setDown(false);

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
    agent_colors_.clear();
    next_marker_id_ = 0;

    // 2a) Clear file paths and names
    map_file_.clear();
    pkg_shared_tree_dir_.clear();
    orig_yaml_base_name_.clear();
    yaml_base_name_.clear();
    defaultName_.clear();

    // Reset dialog/window states
    adding_new_agent_ = false;
    goal_picking_mode_ = false;
    initial_pose_set = false;
    moving_goal_id_ = -1;

    // Reset information flags
    first_goal_picking_info_shown_ = false;
    initial_pose_tip_shown_ = false;
    show_file_selector_once = false;

    // 3) Reset mode & indices
    panel_mode_ = CREATE_MODE;
    current_edit_idx_ = 0;
    iterate_actors_ = 1;
    agent_count = 1;
    num_agents = 0;
    num_actors_ = 0;

    // 4) Reset UI back to “fresh” state
    map_group->setEnabled(false);
    map_group->setVisible(true);
    simulator_combo_->setCurrentIndex(-1);
    map_select_btn_->show();
    map_select_btn_->setVisible(true);
    map_select_btn_->setEnabled(false);
    current_map_label_->clear();
    current_map_label_->show();

    // Reset agent creation fields
    actors->clear();
    actors->show();
    actors->setEnabled(false);
    n_agents_label_->show();
    n_agents_label_->setEnabled(false);

    // Actor button to constructor state
    actor_button_->setText(tr("Generate Agents"));
    actor_button_->setEnabled(false);
    actor_button_->setDown(false);
    actor_button_->setChecked(false);

    // Edit mode buttons to hidden state
    edit_goals_button_->hide();
    edit_goals_button_->setDown(false);
    edit_goals_button_->setChecked(false);
    add_agent_button_->hide();
    add_agent_button_->setEnabled(false);
    add_agent_button_->setDown(false);
    add_agent_button_->setChecked(false);

    // YAML file label
    yaml_file_label_->hide();
    yaml_file_label_->clear();

    // Goal management section
    goal_group_->setTitle("Define agents goals");
    goal_group_->setEnabled(false);

    // Goal picking button
    enter_goal_mode_btn_->show();
    enter_goal_mode_btn_->setDown(false);
    enter_goal_mode_btn_->setText("Enter Goal-Picking Mode");
    enter_goal_mode_btn_->setEnabled(true);

    // Reset: Goal management UI to disabled state
    goal_list_widget_->clear();
    goal_list_widget_->setEnabled(false);
    reset_goals_button_->hide();
    reset_goals_button_->setEnabled(false);
    assign_goals_btn_->setEnabled(false);
    assign_goals_btn_->setDown(false);

    // Reset: Save and other buttons
    save_bt_btn_->setEnabled(false);
    save_bt_btn_->setText("Save agents YAML/Generate BTs");

    checkbox->setEnabled(false);
    checkbox->setChecked(true);

    // Reset: Behavior tree group
    bt_group_->setEnabled(false);
    // edit_bt_btn_->setEnabled(true);

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

  /**
   * @brief Convert an install share directory path to the corresponding src directory path
   *
   * Converts a path from the ROS2 install directory structure to the src directory structure.
   * Handles special cases where packages are located inside metapackages.
   *
   * Example:
   * Input:  /home/hunav_gz_classic_ws/install/hunav_agent_manager/share/hunav_agent_manager
   * Output: /home/hunav_gz_classic_ws/src/hunav_sim/hunav_agent_manager
   *
   * @param install_share_path Full path to a package's share directory in the install folder
   * @return Full path to the package in the src directory
   *
   * @note Special handling for packages inside hunav_sim metapackage:
   *       hunav_agent_manager, hunav_behavior_tree_generator, hunav_evaluator,
   *       hunav_msgs, hunav_rviz2_panel, hunav_webots_wrapper
   */
  std::string ActorPanel::share_to_src_path(const std::string& install_share_path)
  {

    // Example:
    // input: /home/hunav_gz_classic_ws/install/hunav_gazebo_wrapper/share/hunav_gazebo_wrapper
    // output: /home/hunav_gz_classic_ws/src/hunav_gazebo_wrapper
    // Special case for hunav_sim metapackage:
    // input: /home/hunav_gz_classic_ws/install/hunav_agent_manager/share/hunav_agent_manager
    // output: /home/hunav_gz_classic_ws/src/hunav_sim/hunav_agent_manager


    // Packages that are located inside the hunav_sim metapackage
    static const std::set<std::string> metapackage_children = {
      "hunav_agent_manager",
      "hunav_behavior_tree_generator",
      "hunav_evaluator",
      "hunav_msgs",
      "hunav_rviz2_panel",
      "hunav_sim"
    };

    // Extract package name from path
    // Expected format: .../install/<package_name>/share/<package_name>
    size_t install_pos = install_share_path.find("/install/");
    if (install_pos == std::string::npos) {
      //return ""; // Invalid path format
      throw std::runtime_error("The path does not have the expected structure. Install directory not found.");
    }

    // Extract workspace root
    std::string workspace_root = install_share_path.substr(0, install_pos);

    // Extract package name - find it between /install/ and /share/
    size_t start = install_pos + 9; // length of "/install/"
    size_t end = install_share_path.find("/share/", start);
    if (end == std::string::npos) {
      //return ""; // Invalid path format
      throw std::runtime_error("The path does not have the expected structure. share directory not found.");
    }

    std::string package_name = install_share_path.substr(start, end - start);

    // Build src path
    std::string src_path = workspace_root + "/src/";

    // Check if this package is inside the hunav_sim metapackage
    if (metapackage_children.find(package_name) != metapackage_children.end() &&
        package_name != "hunav_sim") {
      src_path += "hunav_sim/";
    }

    src_path += package_name;

    return src_path;
  }

} // namespace hunav_rviz2_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hunav_rviz2_panel::ActorPanel, rviz_common::Panel)
