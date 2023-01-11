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
  // QPushButton *show_metrics = new QPushButton("Show metrics");
  save_metrics = new QPushButton("Save metrics");

  // Layout style
  metrics_layout->setSpacing(20);
  // metrics_layout->insertStretch(1, -1);

  // Logic
  loadMetrics();
  metricsSelectionWindow();
  // selectMetrics(metrics_layout);
  // metrics_layout->addWidget(show_metrics);
  metrics_layout->addWidget(save_metrics);
  metrics_layout->addStretch(1);
  // connect(show_metrics, SIGNAL(clicked()), this,
  //        SLOT(metricsSelectionWindow()));
  metrics_layout->setSpacing(0);
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

  QScrollArea *scrollArea = new QScrollArea(this);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  QWidget *scrollWidget = new QWidget;
  scrollWidget->setLayout(new QVBoxLayout);

  // Load metrics
  // std::string metrics_combobox =
  // paper_selection->currentText().toStdString();

  // removeMetrics(metrics_layout);

  for (const auto &m : metrics_) {
    QCheckBox *check = new QCheckBox(QString::fromStdString(m.first),
                                     this); // fix_typo(m.first)
    check->setChecked(m.second);
    checkboxes.push_back(check);
    metrics_layout->addStretch();
    scrollWidget->layout()->addWidget(check);
  }

  metrics_layout->setSpacing(0);
  scrollArea->setWidget(scrollWidget);
  metrics_layout->addWidget(scrollArea);
  metrics_layout->insertStretch(-1, 1);

  connect(save_metrics, SIGNAL(clicked()), this, SLOT(saveMetricsYaml()));
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

  // updateMetricsVector();

  // update metrics selection
  for (int i = 0; i < (int)checkboxes.size(); i++) {
    std::string key = checkboxes[i]->text().toStdString();
    metrics_[key] = checkboxes[i]->isChecked();
  }

  try {
    pkg_shared_dir =
        ament_index_cpp::get_package_share_directory("hunav_evaluator");

  } catch (const char *msg) {
    RCLCPP_ERROR(this->get_logger(),
                 "Package hunav_evaluator not found in dir: %s!!!",
                 pkg_shared_dir.c_str());
  }
  pkg_shared_dir = pkg_shared_dir + "/config/metrics.yaml";

  YAML::Node output_yaml = YAML::LoadFile(pkg_shared_dir);

  // update the values
  for (YAML::iterator it =
           output_yaml["hunav_evaluator_node"]["ros__parameters"]["metrics"]
               .begin();
       it !=
       output_yaml["hunav_evaluator_node"]["ros__parameters"]["metrics"].end();
       ++it) {
    std::string key = it->first.as<std::string>();
    output_yaml["hunav_evaluator_node"]["ros__parameters"]["metrics"][key] =
        metrics_[key];
    // bool value = it->second.as<bool>();
  }

  // Open file to save metrics
  std::ofstream file;
  file.open(pkg_shared_dir, std::ofstream::trunc);
  if (file.is_open()) {
    file << output_yaml;
    file.close();
    RCLCPP_INFO(this->get_logger(), "Metric file (%s) successfully written!",
                pkg_shared_dir.c_str());
  }

  // removeMetrics(metrics_layout);

  // Clears the metrics array once the user has saved the metrics for the first
  // time
  // if (initial) {
  //   metrics_selected_array.clear();
  // }

  // initial = false;
  // metricsSelectionWindow();
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
} // namespace hunav_rviz2_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hunav_rviz2_panel::MetricsPanel, rviz_common::Panel)