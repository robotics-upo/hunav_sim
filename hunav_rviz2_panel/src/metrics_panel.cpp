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
#include <ament_index_cpp/get_resource.hpp>

#include "headers/metrics_panel.hpp"


using std::placeholders::_1;

namespace hunav_rviz2_panel
{
    MetricsPanel::MetricsPanel(QWidget *parent)
        : rviz_common::Panel(parent), rclcpp::Node("hunav_agents")
    {
        window = new QWidget(this);
        metrics_layout = new QVBoxLayout(window);
        QPushButton *show_metrics = new QPushButton("Show metrics");
        save_metrics = new QPushButton("Save metrics");
        
        // Layout style
        metrics_layout->setSpacing(20);
        //metrics_layout->insertStretch(1, -1);

        //Logic
        loadMetrics();
        selectMetrics(metrics_layout);
        metrics_layout->addWidget(show_metrics);
        metrics_layout->addWidget(save_metrics);
        metrics_layout->addStretch(1);
        connect(show_metrics, SIGNAL(clicked()), this, SLOT(metricsSelectionWindow()));
        metrics_layout->setSpacing(0);
        setLayout(metrics_layout);
    }

    MetricsPanel::~MetricsPanel(){

    }

    void MetricsPanel::selectMetrics(QVBoxLayout *metrics_layout){
        paper_selection = new QComboBox();

        for(auto i : papers_parsed){
            paper_selection->addItem(QString::fromStdString(i));
        }

        metrics_layout->addWidget(paper_selection);
    }

    void MetricsPanel::loadMetrics(){
        
        YAML::Node metrics_file;
        YAML::Node papers;
        YAML::Node metrics;
        
        try {
        pkg_shared_dir = ament_index_cpp::get_package_prefix("hunav_evaluator");
        std::string toReplace("install/hunav_evaluator");
        size_t pos = pkg_shared_dir.find(toReplace);
        pkg_shared_dir.replace(pos, toReplace.length(), "src/hunav_sim/hunav_evaluator");

        } catch (const char* msg) {
            RCLCPP_ERROR(this->get_logger(),
                        "Package hunav_evaluator not found in dir: %s!!!",
                        pkg_shared_dir.c_str());
        }

        pkg_shared_dir = pkg_shared_dir + "/config/metrics.yaml";


        RCLCPP_INFO(this->get_logger(),
                        "RUTA: %s",
                    pkg_shared_dir.c_str());

        metrics_file = YAML::LoadFile(pkg_shared_dir);
        
        // Get papers
        papers = metrics_file["hunav_evaluator_node"]["ros__parameters"]["papers"];

        for(int i = 0; i < static_cast<int>(papers.size()); i++){
            papers_parsed.push_back(metrics_file["hunav_evaluator_node"]["ros__parameters"]["papers"][i].as<std::string>());
            
            // Get metrics
            metrics = metrics_file["hunav_evaluator_node"]["ros__parameters"][papers_parsed.back()];

            for(int i = 0; i < static_cast<int>(metrics.size()); i++){
                std::string m = metrics_file["hunav_evaluator_node"]["ros__parameters"][papers_parsed.back()][i].as<std::string>();
                metrics_parsed.insert(make_pair(papers_parsed.back(), m));
            }
        }

        for(auto const& it : metrics_parsed){
            metrics_bool.insert(make_pair(it.second, metrics_file["hunav_evaluator_node"]["ros__parameters"][it.second].as<std::string>()));
        }

        for(auto const& it : metrics_bool){
            RCLCPP_INFO(this->get_logger(), "%s/%s", it.first.c_str(), it.second.c_str());
        }
    }

    void MetricsPanel::metricsSelectionWindow(){

        QScrollArea *scrollArea = new QScrollArea(this);
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        QWidget *scrollWidget = new QWidget;
        scrollWidget->setLayout(new QVBoxLayout);

        // Load metrics
        std::string metrics_combobox = paper_selection->currentText().toStdString();

        removeMetrics(metrics_layout);

        for(auto paper : papers_parsed){
            if(metrics_combobox.compare(paper) == 0){
                current_paper = paper;

                for(auto const& it : metrics_parsed){
                    if(it.first.compare(paper) == 0){
                        QCheckBox *check = new QCheckBox(QString::fromStdString(fix_typo(it.second)), this);
                        if(initial){
                            for(auto const& m_bool : metrics_bool){
                                if(it.second.compare(m_bool.first) == 0){
                                    if(m_bool.second.compare("True") == 0){
                                        check->setChecked(true);
                                    }
                                }
                            }
                        }
                    
                        checkboxes.push_back(check);
                        metrics_layout->addStretch();
                        scrollWidget->layout()->addWidget(check);
                    }
                }
            }
        }

        metrics_layout->setSpacing(0);
        scrollArea->setWidget(scrollWidget);
        metrics_layout->addWidget(scrollArea);
        metrics_layout->insertStretch(-1, 1);

        connect(save_metrics, SIGNAL(clicked()), this, SLOT(generateMetricsYaml()));
    }

    void MetricsPanel::updateMetricsVector(){
        
        bool found = false; 
        for(int i = 0; i < static_cast<int>(checkboxes.size()); i++){
            if(checkboxes[i]->isChecked()){
                std::string selected = checkboxes[i]->text().toStdString();
                std::string selected_modified = selected;
                std::for_each(selected_modified.begin(), selected_modified.end(), [](char & c){
                    c = ::tolower(c);
                });
                std::replace(selected_modified.begin(), selected_modified.end(), ' ', '_');
                
                // Check if chose metric is not in the multimap.
                for(const auto &it : metrics_selected_array){
                    if(selected_modified.compare(it.second) == 0){
                        found = true;
                    }
                }

                // If not found, insert it.
                if(found == false){
                    metrics_selected_array.insert(make_pair(current_paper, selected_modified));
                }
            }
        }

    }

    void MetricsPanel::removeMetrics(QLayout *layout){
        QLayoutItem *item;
        while((item = layout->takeAt(3))) {
            if (item->widget()) {
            delete item->widget();
            }
            delete item;
        }
        
        checkboxes.clear();
    }

    void MetricsPanel::generateMetricsYaml(){

        updateMetricsVector();
        
        std::ofstream file;
        YAML::Node hunav_evaluator_node;

        try {
            pkg_shared_dir = ament_index_cpp::get_package_share_directory("hunav_evaluator");

        } catch (const char* msg) {
            RCLCPP_ERROR(this->get_logger(),
                        "Package hunav_evaluator not found in dir: %s!!!",
                    pkg_shared_dir.c_str());
        }
        pkg_shared_dir = pkg_shared_dir + "/config/metrics.yaml";
        
        // Open file to save metrics
        file.open(pkg_shared_dir , std::ofstream::trunc);

        hunav_evaluator_node["hunav_evaluator_node"]["ros__parameters"]["frequency"] = 1.0;
        hunav_evaluator_node["hunav_evaluator_node"]["ros__parameters"]["experiment_tag"] = '1';
        hunav_evaluator_node["hunav_evaluator_node"]["ros__parameters"]["result_file"] = "test.txt";

        for(auto i = papers_parsed.begin(); i != papers_parsed.end(); ++i){
            hunav_evaluator_node["hunav_evaluator_node"]["ros__parameters"]["papers"].push_back(*i);
        }

        for(const auto &it : metrics_selected_array){
            hunav_evaluator_node["hunav_evaluator_node"]["ros__parameters"][it.first].push_back(it.second);
        }

        for(const auto &it : metrics_selected_array){
             hunav_evaluator_node["hunav_evaluator_node"]["ros__parameters"][it.second] = "True";
        }

        file << hunav_evaluator_node;
        file.close();

        removeMetrics(metrics_layout);

        // Clears the metrics array once the user has saved the metrics for the first time
        if(initial){
            metrics_selected_array.clear();
        }

        initial = false;
        metricsSelectionWindow();
    }

    std::string MetricsPanel::fix_typo(std::string metric){
    
        std::for_each(metric.begin(), metric.end(), [](char & c){
            c = ::tolower(c);
        });
        std::replace(metric.begin(), metric.end(), '_', ' ');
        metric[0] = toupper(metric[0]);
        return metric;

    }

    void MetricsPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        config.mapSetValue("Topic", output_topic_);
    }

    // Load all configuration data for this panel from the given Config object.
    void MetricsPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hunav_rviz2_panel::MetricsPanel, rviz_common::Panel)