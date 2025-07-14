#include <QBasicTimer>
#include <QtWidgets>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>
#include <map>

//#include "nav2_msgs/action/navigate_to_pose.hpp"
//#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/tool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
//#include "visualization_msgs/msg/marker_array.hpp"
//#include "nav2_util/geometry_utils.hpp"
#include "rviz_common/display_context.hpp"

#include "yaml-cpp/yaml.h"

#include "std_msgs/msg/string.hpp"

#include "rclcpp/qos.hpp"

#include <rviz_common/message_filter_display.hpp>

#include <QTreeWidget>
#include <tf2/LinearMath/Quaternion.h>

class QLineEdit;

namespace hunav_rviz2_panel {

class MetricsPanel : public rviz_common::Panel, public rclcpp::Node {

  Q_OBJECT
public:
  MetricsPanel(QWidget *parent = 0);
  ~MetricsPanel();

  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:

  // void selectMetrics(QVBoxLayout *metrics_layout);
  void loadMetrics();
  void metricsSelectionWindow();
  // void updateMetricsVector();
  void saveMetricsYaml();
  void onSearchTextChanged();
  void selectAllMetrics();
  void deselectAllMetrics();
  void toggleCategory(const QString& categoryName);
  QString categorizeMetric(const QString& metricName);
  QString formatMetricName(const QString& metricName);
  QString getMetricTooltip(const QString& metricName);
  void updateStatusLabel();
  // void removeMetrics(QLayout *layout);
  // std::string fix_typo(std::string metric);

public:
  // QString output_topic_;
  // Initial panel
  // QComboBox *paper_selection;
  // QComboBox *metrics_selection;

  // Metrics window
  QWidget *window;
  QVBoxLayout *metrics_layout;
  // QWidget *metrics_window = nullptr;
  // QGridLayout *grid_load_metrics_layout;
  // QVBoxLayout *load_metrics_layout;
  // QList<QLabel *> labels;
  QPushButton *save_metrics;
  
  // New UI components
  QLineEdit *search_box;
  QPushButton *select_all_btn;
  QPushButton *deselect_all_btn;
  QLabel *status_label;
  QScrollArea *scroll_area;
  QWidget *scroll_widget;

  // Logic
  std::unordered_map<std::string, bool> metrics_;
  // std::vector<std::string> papers_parsed;
  std::string pkg_shared_dir;
  // std::multimap<std::string, std::string> metrics_parsed;
  // std::multimap<std::string, std::string> metrics_selected_array;
  std::vector<QCheckBox *> checkboxes;
  std::map<QString, QGroupBox*> category_groups;
  std::map<QString, std::vector<QCheckBox*>> category_checkboxes;
  // std::string current_paper;
  // bool initial = true;
  // std::multimap<std::string, std::string> metrics_bool;
};

} // namespace hunav_rviz2_panel