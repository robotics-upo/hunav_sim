#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdio.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QScrollArea>
#include <QPushButton>
#include <QMessageBox>

#include <QDebug>

#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>

#include <memory>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/tool.hpp"
#include "std_msgs/msg/string.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_resource.hpp>

#include "headers/metrics_panel.hpp"

using std::placeholders::_1;

namespace hunav_rviz2_panel {
MetricsPanel::MetricsPanel(QWidget *parent)
    : rviz_common::Panel(parent), rclcpp::Node("hunav_metrics_panel") {
  window = new QWidget(this);
  metrics_layout = new QVBoxLayout(window);
  
  // Create title
  QLabel *title = new QLabel("<h2>Metrics Configuration</h2>");
  title->setAlignment(Qt::AlignCenter);
  metrics_layout->addWidget(title);
  
  // Create search box
  QHBoxLayout *search_layout = new QHBoxLayout();
  QLabel *search_label = new QLabel("Search metrics:");
  search_box = new QLineEdit();
  search_box->setPlaceholderText("Type to filter metrics...");
  search_layout->addWidget(search_label);
  search_layout->addWidget(search_box);
  metrics_layout->addLayout(search_layout);
  
  // Create control buttons
  QHBoxLayout *button_layout = new QHBoxLayout();
  select_all_btn = new QPushButton("Select All");
  deselect_all_btn = new QPushButton("Deselect All");
  save_metrics = new QPushButton("Save Configuration");
  
  // Style the save button
  save_metrics->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
  
  button_layout->addWidget(select_all_btn);
  button_layout->addWidget(deselect_all_btn);
  button_layout->addStretch();
  button_layout->addWidget(save_metrics);
  metrics_layout->addLayout(button_layout);
  
  // Create status label
  status_label = new QLabel("");
  status_label->setStyleSheet("QLabel { color: #666; font-style: italic; }");
  metrics_layout->addWidget(status_label);

  // Layout style
  metrics_layout->setSpacing(10);
  metrics_layout->setContentsMargins(10, 10, 10, 10);

  // Logic
  loadMetrics();
  metricsSelectionWindow();
  
  // Connect signals
  connect(search_box, &QLineEdit::textChanged, this, &MetricsPanel::onSearchTextChanged);
  connect(select_all_btn, &QPushButton::clicked, this, &MetricsPanel::selectAllMetrics);
  connect(deselect_all_btn, &QPushButton::clicked, this, &MetricsPanel::deselectAllMetrics);
  connect(save_metrics, &QPushButton::clicked, this, &MetricsPanel::saveMetricsYaml);
  
  setLayout(metrics_layout);
}

MetricsPanel::~MetricsPanel() {}

// void MetricsPanel::selectMetrics(QVBoxLayout *metrics_layout) {
//   paper_selection = new QComboBox();

//   for (auto i : papers_parsed) {
//     paper_selection->addItem(QString::fromStdString(i));
//   }

//   metrics_layout->addWidget(paper_selection);
// }

void MetricsPanel::loadMetrics() {

  YAML::Node metrics_file;

  try {
    // pkg_shared_dir = ament_index_cpp::get_package_prefix("hunav_evaluator");
    pkg_shared_dir =
        ament_index_cpp::get_package_share_directory("hunav_evaluator");
    // std::string toReplace("install/hunav_evaluator");
    // size_t pos = pkg_shared_dir.find(toReplace);
    // pkg_shared_dir.replace(pos, toReplace.length(),
    // "src/hunav_sim/hunav_evaluator");

  } catch (const char *msg) {
    RCLCPP_ERROR(this->get_logger(),
                 "Package hunav_evaluator not found in dir: %s!!!",
                 pkg_shared_dir.c_str());
  }

  pkg_shared_dir = pkg_shared_dir + "/config/metrics.yaml";

  RCLCPP_INFO(this->get_logger(), "RUTA: %s", pkg_shared_dir.c_str());

  metrics_file = YAML::LoadFile(pkg_shared_dir);

  // Get metrics
  YAML::Node metrics =
      metrics_file["hunav_evaluator_node"]["ros__parameters"]["metrics"];
  for (YAML::iterator it = metrics.begin(); it != metrics.end(); ++it) {
    std::string key = it->first.as<std::string>();
    // it->first >> key;
    bool value = it->second.as<bool>();
    metrics_[key] = value;
    RCLCPP_INFO(this->get_logger(), "METRIC: %s VALUE: %i", key.c_str(),
                (int)value);
  }
}

void MetricsPanel::metricsSelectionWindow() {

  scroll_area = new QScrollArea(this);
  scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setFrameShadow(QFrame::Plain);
  scroll_area->setWidgetResizable(true);
  scroll_area->setStyleSheet(
    "QScrollArea {"
    "  border-radius: 5px;"
    "  padding: 8px;"
    "}"
  );
  scroll_widget = new QWidget;
  QVBoxLayout *scroll_layout = new QVBoxLayout(scroll_widget);
  scroll_layout->setSpacing(8);
  scroll_layout->setContentsMargins(8, 8, 8, 8);

  // Clear previous data
  checkboxes.clear();
  category_groups.clear();
  category_checkboxes.clear();

  // Create category groups
  QStringList categories = {"Navigation Performance", "Social Behavior", "Motion Analysis", "Safety & Collisions", "Social Forces", "Danger & Surprise Metrics"};
  
  for (const QString &category : categories) {
    QGroupBox *group = new QGroupBox(category);
    // Enhanced styling for better visual containment
    group->setStyleSheet(
      "QGroupBox {"
      "  font-weight: bold;"
      "  font-size: 16px;"
      "  color: #2c3e50;"
      "  margin-top: 15px;"
      "  margin-bottom: 5px;"
      "  padding-top: 10px;"
      "  border: 2px solid #bdc3c7;"
      "  border-radius: 8px;"
      "  background-color: #f8f9fa;"
      "}"
      "QGroupBox::title {"
      "  subcontrol-origin: margin;"
      "  subcontrol-position: top left;"
      "  padding: 5px 10px;"
      "  background-color: #e9ecef;"
      "  border: 1px solid #bdc3c7;"
      "  border-radius: 4px;"
      "  color: #2c3e50;"
      "}"
      "QGroupBox QCheckBox {"
      "  margin: 4px 8px;"
      "  padding: 2px;"
      "  color: #2c3e50;"
      "}"
      "QGroupBox QCheckBox:hover {"
      "  background-color: #e3f2fd;"
      "  border-radius: 3px;"
      "}"
    );
    QVBoxLayout *group_layout = new QVBoxLayout(group);
    group_layout->setSpacing(3);
    group_layout->setContentsMargins(15, 15, 10, 10);
    category_groups[category] = group;
    scroll_layout->addWidget(group);
  }

  // Populate metrics by category
  for (const auto &m : metrics_) {
    QString metric_name = QString::fromStdString(m.first);
    QString category = categorizeMetric(metric_name);
    QString formatted_name = formatMetricName(metric_name);
    
    QCheckBox *check = new QCheckBox(formatted_name, this);
    check->setChecked(m.second);
    check->setToolTip(getMetricTooltip(metric_name));
    check->setToolTipDuration(5000); // Show tooltip for 5 seconds
    
    // Store original metric name as property for saving
    check->setProperty("originalName", metric_name);
    
    // Connect checkbox state change to update status
    connect(check, &QCheckBox::toggled, this, &MetricsPanel::updateStatusLabel);
    
    checkboxes.push_back(check);
    category_checkboxes[category].push_back(check);
    
    if (category_groups.find(category) != category_groups.end()) {
      category_groups[category]->layout()->addWidget(check);
    }
  }

  scroll_area->setWidget(scroll_widget);
  metrics_layout->addWidget(scroll_area);
  
  // Update status
  updateStatusLabel();
}

// void MetricsPanel::updateMetricsVector() {

//   bool found = false;
//   for (int i = 0; i < static_cast<int>(checkboxes.size()); i++) {
//     if (checkboxes[i]->isChecked()) {
//       std::string selected = checkboxes[i]->text().toStdString();
//       std::string selected_modified = selected;
//       std::for_each(selected_modified.begin(), selected_modified.end(),
//                     [](char &c) { c = ::tolower(c); });
//       std::replace(selected_modified.begin(), selected_modified.end(), ' ',
//                    '_');

//       // Check if chose metric is not in the multimap.
//       for (const auto &it : metrics_selected_array) {
//         if (selected_modified.compare(it.second) == 0) {
//           found = true;
//         }
//       }

//       // If not found, insert it.
//       if (found == false) {
//         metrics_selected_array.insert(
//             make_pair(current_paper, selected_modified));
//       }
//     }
//   }
// }

// void MetricsPanel::removeMetrics(QLayout *layout) {
//   QLayoutItem *item;
//   while ((item = layout->takeAt(3))) {
//     if (item->widget()) {
//       delete item->widget();
//     }
//     delete item;
//   }

//   checkboxes.clear();
// }

void MetricsPanel::saveMetricsYaml() {

  // Update metrics selection using original names
  for (QCheckBox *checkbox : checkboxes) {
    QString originalName = checkbox->property("originalName").toString();
    std::string key = originalName.toStdString();
    metrics_[key] = checkbox->isChecked();
  }

  try {
    pkg_shared_dir =
        ament_index_cpp::get_package_share_directory("hunav_evaluator");

  } catch (const char *msg) {
    RCLCPP_ERROR(this->get_logger(),
                 "Package hunav_evaluator not found in dir: %s!!!",
                 pkg_shared_dir.c_str());
    status_label->setText("Error: hunav_evaluator package not found!");
    status_label->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    return;
  }
  
  pkg_shared_dir = pkg_shared_dir + "/config/metrics.yaml";

  YAML::Node output_yaml;
  
  try {
    output_yaml = YAML::LoadFile(pkg_shared_dir);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading YAML file: %s", e.what());
    status_label->setText("Error: Could not load metrics.yaml file!");
    status_label->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    return;
  }

  // Update the values
  for (YAML::iterator it =
           output_yaml["hunav_evaluator_node"]["ros__parameters"]["metrics"]
               .begin();
       it !=
       output_yaml["hunav_evaluator_node"]["ros__parameters"]["metrics"].end();
       ++it) {
    std::string key = it->first.as<std::string>();
    if (metrics_.find(key) != metrics_.end()) {
      output_yaml["hunav_evaluator_node"]["ros__parameters"]["metrics"][key] =
          metrics_[key];
    }
  }

  // Open file to save metrics
  std::ofstream file;
  file.open(pkg_shared_dir, std::ofstream::trunc);
  if (file.is_open()) {
    file << output_yaml;
    file.close();
    RCLCPP_INFO(this->get_logger(), "Metric file (%s) successfully written!",
                pkg_shared_dir.c_str());
    
    // Update status with success message
    int selected = 0;
    for (const auto &m : metrics_) {
      if (m.second) selected++;
    }
    status_label->setText(QString("✓ Configuration saved successfully! (%1 metrics selected)").arg(selected));
    status_label->setStyleSheet("QLabel { color: green; font-weight: bold; }");
    
    // Reset status color after 3 seconds
    QTimer::singleShot(3000, [this]() {
      status_label->setStyleSheet("QLabel { color: #666; font-style: italic; }");
    });
  } else {
    RCLCPP_ERROR(this->get_logger(), "Could not open file for writing: %s", pkg_shared_dir.c_str());
    status_label->setText("Error: Could not save configuration file!");
    status_label->setStyleSheet("QLabel { color: red; font-weight: bold; }");
  }
}

// std::string MetricsPanel::fix_typo(std::string metric) {

//   std::for_each(metric.begin(), metric.end(),
//                 [](char &c) { c = ::tolower(c); });
//   std::replace(metric.begin(), metric.end(), '_', ' ');
//   metric[0] = toupper(metric[0]);
//   return metric;
// }

void MetricsPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  // config.mapSetValue("Topic", output_topic_);
}

// Load all configuration data for this panel from the given Config object.
void MetricsPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
}

QString MetricsPanel::categorizeMetric(const QString& metricName) {
  // Navigation Performance metrics
  if (metricName.contains("time_to_reach_goal") || metricName.contains("path_length") || 
      metricName.contains("completed") || metricName.contains("distance_to_target") ||
      metricName.contains("time_not_moving")) {
    return "Navigation Performance";
  }
  
  // Social Behavior metrics
  if (metricName.contains("space_intrusions") || metricName.contains("distance_to_closest_person") ||
      metricName.contains("minimum_distance_to_people") || metricName.contains("group_") ||
      metricName.contains("pedestrian_velocity")) {
    return "Social Behavior";
  }
  
  // Motion Analysis metrics
  if (metricName.contains("speed") || metricName.contains("acceleration") || 
      metricName.contains("heading_changes") || metricName.contains("velocity")) {
    return "Motion Analysis";
  }
  
  // Safety & Collisions metrics
  if (metricName.contains("collision") || metricName.contains("minimum_distance")) {
    return "Safety & Collisions";
  }
  
  // Social Forces metrics
  if (metricName.contains("force") || metricName.contains("social_work")) {
    return "Social Forces";
  }

  // Danger metrics
  if (metricName.contains("danger") || metricName.contains("surprise")) {
    return "Danger & Surprise Metrics";
  }
  
  return "Navigation Performance"; // default category
}

QString MetricsPanel::formatMetricName(const QString& metricName) {
  QString formatted = metricName;
  formatted.replace("_", " ");
  
  // Capitalize first letter of each word
  QStringList words = formatted.split(" ");
  for (int i = 0; i < words.size(); ++i) {
    if (!words[i].isEmpty()) {
      words[i][0] = words[i][0].toUpper();
    }
  }
  
  return words.join(" ");
}

QString MetricsPanel::getMetricTooltip(const QString& metricName) {
  // Provide comprehensive tooltips for metrics
  if (metricName == "time_to_reach_goal") {
    return "<b>Time To Reach Goal</b><br/>"
           "Total time taken by the robot to reach its navigation goal.<br/>"
           "<i>Units:</i> seconds<br/>"
           "<i>Lower values indicate better performance.</i>";
  } else if (metricName == "path_length") {
    return "<b>Path Length</b><br/>"
           "Total distance traveled by the robot during navigation.<br/>"
           "<i>Units:</i> meters<br/>"
           "<i>Shorter paths are generally more efficient.</i>";
  } else if (metricName == "cumulative_heading_changes") {
    return "<b>Cumulative Heading Changes</b><br/>"
           "Sum of all angular changes in robot's heading during navigation.<br/>"
           "<i>Units:</i> radians<br/>"
           "<i>Lower values indicate smoother, more direct motion.</i>";
  } else if (metricName == "avg_distance_to_closest_person") {
    return "<b>Average Distance to Closest Person</b><br/>"
           "Mean distance maintained to the nearest person throughout navigation.<br/>"
           "<i>Units:</i> meters<br/>"
           "<i>Higher values indicate better social distancing.</i>";
  } else if (metricName == "minimum_distance_to_people") {
    return "<b>Minimum Distance to People</b><br/>"
           "Closest approach distance to any person during the entire trajectory.<br/>"
           "<i>Units:</i> meters<br/>"
           "<i>Critical for safety assessment.</i>";
  } else if (metricName == "intimate_space_intrusions") {
    return "<b>Intimate Space Intrusions</b><br/>"
           "Number of times robot entered intimate personal space (&lt; 0.45m).<br/>"
           "<i>Based on:</i> Proxemics theory (Edward T. Hall)<br/>"
           "<i>Zero intrusions is ideal for social compliance.</i>";
  } else if (metricName == "personal_space_intrusions") {
    return "<b>Personal Space Intrusions</b><br/>"
           "Number of times robot entered personal space (0.45m - 1.2m).<br/>"
           "<i>Based on:</i> Proxemics theory<br/>"
           "<i>Should be minimized for comfortable interaction.</i>";
  } else if (metricName == "social_space_intrusions") {
    return "<b>Social Space Intrusions</b><br/>"
           "Number of times robot entered social space (1.2m - 3.6m).<br/>"
           "<i>Based on:</i> Proxemics theory<br/>"
           "<i>Less critical but still important for social navigation.</i>";
  } else if (metricName.contains("group_") && metricName.contains("space_intrusions")) {
    return "<b>Group Space Intrusions</b><br/>"
           "Space violations considering group formations and dynamics.<br/>"
           "<i>Groups have different spatial boundaries than individuals.</i><br/>"
           "<i>Important for understanding social group behavior.</i>";
  } else if (metricName == "completed") {
    return "<b>Task Completion</b><br/>"
           "Boolean indicator of whether the robot successfully reached its goal.<br/>"
           "<i>Values:</i> true/false<br/>"
           "<i>Fundamental measure of navigation success.</i>";
  } else if (metricName == "minimum_distance_to_target") {
    return "<b>Minimum Distance to Target</b><br/>"
           "Closest approach distance to the navigation goal.<br/>"
           "<i>Units:</i> meters<br/>"
           "<i>Useful for assessing goal-reaching precision.</i>";
  } else if (metricName == "final_distance_to_target") {
    return "<b>Final Distance to Target</b><br/>"
           "Distance between robot and goal when navigation ends.<br/>"
           "<i>Units:</i> meters<br/>"
           "<i>Measures final positioning accuracy.</i>";
  } else if (metricName == "robot_on_person_collision") {
    return "<b>Robot-on-Person Collisions</b><br/>"
           "Number of collisions where the robot contacted a person.<br/>"
           "<i>Critical safety metric - should always be zero.</i>";
  } else if (metricName == "person_on_robot_collision") {
    return "<b>Person-on-Robot Collisions</b><br/>"
           "Number of collisions where a person contacted the robot.<br/>"
           "<i>May indicate unpredictable human behavior or poor robot positioning.</i>";
  } else if (metricName == "time_not_moving") {
    return "<b>Time Not Moving</b><br/>"
           "Total time spent stationary during navigation.<br/>"
           "<i>Units:</i> seconds<br/>"
           "<i>High values may indicate planning difficulties or deadlock situations.</i>";
  } else if (metricName == "avg_robot_linear_speed") {
    return "<b>Average Robot Linear Speed</b><br/>"
           "Mean forward velocity of the robot during navigation.<br/>"
           "<i>Units:</i> m/s<br/>"
           "<i>Higher speeds improve efficiency but may reduce safety.</i>";
  } else if (metricName == "avg_robot_angular_speed") {
    return "<b>Average Robot Angular Speed</b><br/>"
           "Mean rotational velocity of the robot during navigation.<br/>"
           "<i>Units:</i> rad/s<br/>"
           "<i>High values may indicate oscillatory or inefficient motion.</i>";
  } else if (metricName == "avg_acceleration") {
    return "<b>Average Acceleration</b><br/>"
           "Mean rate of change of robot velocity.<br/>"
           "<i>Units:</i> m/s²<br/>"
           "<i>Smooth acceleration patterns indicate better motion planning.</i>";
  } else if (metricName == "avg_overacceleration") {
    return "<b>Average Over-acceleration</b><br/>"
           "Mean acceleration values exceeding comfortable thresholds.<br/>"
           "<i>Units:</i> m/s²<br/>"
           "<i>High values may cause discomfort or indicate aggressive navigation.</i>";
  } else if (metricName == "avg_pedestrian_velocity") {
    return "<b>Average Pedestrian Velocity</b><br/>"
           "Mean walking speed of all people in the environment.<br/>"
           "<i>Units:</i> m/s<br/>"
           "<i>Useful for understanding environment dynamics.</i>";
  } else if (metricName == "avg_closest_pedestrian_velocity") {
    return "<b>Average Closest Pedestrian Velocity</b><br/>"
           "Mean walking speed of the person nearest to the robot.<br/>"
           "<i>Units:</i> m/s<br/>"
           "<i>Important for immediate interaction analysis.</i>";
  } else if (metricName == "social_force_on_agents") {
    return "<b>Social Force on Agents</b><br/>"
           "Social forces experienced by people due to robot presence.<br/>"
           "<i>Based on:</i> Social Force Model (Helbing & Molnár)<br/>"
           "<i>Lower values indicate less disruptive robot behavior.</i>";
  } else if (metricName == "social_force_on_robot") {
    return "<b>Social Force on Robot</b><br/>"
           "Social forces experienced by robot due to human presence.<br/>"
           "<i>Based on:</i> Social Force Model<br/>"
           "<i>Indicates level of social pressure on the robot.</i>";
  } else if (metricName == "social_work") {
    return "<b>Social Work Function</b><br/>"
           "Energy-based measure of social appropriateness of robot motion.<br/>"
           "<i>Used in:</i> Social force window planner<br/>"
           "<i>Lower values indicate more socially acceptable trajectories.</i>";
  } else if (metricName == "obstacle_force_on_robot") {
    return "<b>Obstacle Force on Robot</b><br/>"
           "Physical forces from obstacles acting on the robot.<br/>"
           "<i>Based on:</i> Social Force Model<br/>"
           "<i>Indicates navigation difficulty due to static obstacles.</i>";
  } else if (metricName == "obstacle_force_on_agents") {
    return "<b>Obstacle Force on Agents</b><br/>"
           "Physical forces from obstacles acting on people.<br/>"
           "<i>Based on:</i> Social Force Model<br/>"
           "<i>Shows how obstacles affect human movement patterns.</i>";
  } else if (metricName == "danger_fear_cost") {
    return "<b>Fear cost</b><br/>"
           "Perceived feeling of fear of collision with the robot.<br/>"
           "<i>Based on:</i> Depends on the velocities of humans and the robot<br/>"
           "<i>Values close to zero indicate no fear</i>";
  } else if (metricName == "danger_panic_cost") {
    return "<b>Panic cost</b><br/>"
           "Perceived feelings of panic.<br/>"
           "<i>Based on:</i> Depends on the velocities of humans and the robot<br/>"
           "<i>Values close to zero indicate no panic</i>";
  } else if (metricName == "surprise_visibility_cost") {
    return "<b>Surprise visibility cost</b><br/>"
           "Perceived feeling of surprise based on the visibility<br/>"
           "<i>Based on:</i> Measures the surprise based on the distance and angle between human and robot<br/>"
           "<i>Values close to zero indicate no panic</i>";
  } else if (metricName == "surprise_shock_cost") {
    return "<b>Surprise shock cost</b><br/>"
           "Perceived feeling of shock when something happens before it can be recognized<br/>"
           "<i>Based on:</i> Measures surprise in terms of time <br/>"
           "<i>Values close to zero indicate no panic</i>";
  } else if (metricName == "surprise_react_cost") {
    return "<b>Surprise react cost</b><br/>"
           "Perceived feeling of reaction to sudden aparitions<br/>"
           "<i>Based on:</i> Measures surprise in terms of time <br/>"
           "<i>Values close to zero indicate no panic</i>";
  }
  
  // Default tooltip for unknown metrics
  QString formatted = formatMetricName(metricName);
  return QString("<b>%1</b><br/>Navigation evaluation metric.<br/><i>Consult documentation for detailed description.</i>").arg(formatted);
}

void MetricsPanel::onSearchTextChanged() {
  QString searchText = search_box->text().toLower();
  
  for (QCheckBox *checkbox : checkboxes) {
    QString metricText = checkbox->text().toLower();
    QString originalName = checkbox->property("originalName").toString().toLower();
    
    bool visible = searchText.isEmpty() || 
                   metricText.contains(searchText) || 
                   originalName.contains(searchText);
    
    checkbox->setVisible(visible);
  }
  
  // Hide empty categories
  for (auto it = category_groups.begin(); it != category_groups.end(); ++it) {
    bool hasVisibleItems = false;
    for (QCheckBox *checkbox : category_checkboxes[it->first]) {
      if (checkbox->isVisible()) {
        hasVisibleItems = true;
        break;
      }
    }
    it->second->setVisible(hasVisibleItems);
  }
}

void MetricsPanel::selectAllMetrics() {
  // Temporarily disconnect signals to avoid multiple updates
  for (QCheckBox *checkbox : checkboxes) {
    disconnect(checkbox, &QCheckBox::toggled, this, &MetricsPanel::updateStatusLabel);
  }
  
  for (QCheckBox *checkbox : checkboxes) {
    if (checkbox->isVisible()) {
      checkbox->setChecked(true);
    }
  }
  
  // Reconnect signals
  for (QCheckBox *checkbox : checkboxes) {
    connect(checkbox, &QCheckBox::toggled, this, &MetricsPanel::updateStatusLabel);
  }
  
  updateStatusLabel();
}

void MetricsPanel::deselectAllMetrics() {
  // Temporarily disconnect signals to avoid multiple updates
  for (QCheckBox *checkbox : checkboxes) {
    disconnect(checkbox, &QCheckBox::toggled, this, &MetricsPanel::updateStatusLabel);
  }
  
  for (QCheckBox *checkbox : checkboxes) {
    if (checkbox->isVisible()) {
      checkbox->setChecked(false);
    }
  }
  
  // Reconnect signals
  for (QCheckBox *checkbox : checkboxes) {
    connect(checkbox, &QCheckBox::toggled, this, &MetricsPanel::updateStatusLabel);
  }
  
  updateStatusLabel();
}

void MetricsPanel::toggleCategory(const QString& categoryName) {
  if (category_checkboxes.find(categoryName) == category_checkboxes.end()) return;
  
  // Check if any checkbox in the category is unchecked
  bool anyUnchecked = false;
  for (QCheckBox *checkbox : category_checkboxes[categoryName]) {
    if (!checkbox->isChecked()) {
      anyUnchecked = true;
      break;
    }
  }
  
  // Temporarily disconnect signals to avoid multiple updates
  for (QCheckBox *checkbox : category_checkboxes[categoryName]) {
    disconnect(checkbox, &QCheckBox::toggled, this, &MetricsPanel::updateStatusLabel);
  }
  
  // If any unchecked, check all; otherwise uncheck all
  for (QCheckBox *checkbox : category_checkboxes[categoryName]) {
    checkbox->setChecked(anyUnchecked);
  }
  
  // Reconnect signals
  for (QCheckBox *checkbox : category_checkboxes[categoryName]) {
    connect(checkbox, &QCheckBox::toggled, this, &MetricsPanel::updateStatusLabel);
  }
  
  updateStatusLabel();
}

void MetricsPanel::updateStatusLabel() {
  int total = checkboxes.size();
  int selected = 0;
  for (QCheckBox *checkbox : checkboxes) {
    if (checkbox->isChecked()) selected++;
  }
  status_label->setText(QString("Total metrics: %1, Selected: %2").arg(total).arg(selected));
}
} // namespace hunav_rviz2_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hunav_rviz2_panel::MetricsPanel, rviz_common::Panel)