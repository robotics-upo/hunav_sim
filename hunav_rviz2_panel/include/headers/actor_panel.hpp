#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/tool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rviz_common/display_context.hpp"

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/qos.hpp"

#include <rviz_common/message_filter_display.hpp>

class QLineEdit;

namespace hunav_rviz2_panel
{

class ActorPanel: public rviz_common::Panel, public rclcpp::Node/*public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PointStamped>*/
{

Q_OBJECT
public:

  ActorPanel( QWidget* parent = 0 );

  virtual void load(const rviz_common::Config& config );
  virtual void save(rviz_common::Config config ) const;

public Q_SLOTS:

  void setTopic( const QString& topic );
  
protected Q_SLOTS:

  void addActor();
  void exitWindow();
  void updateTopic();
  void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  int processMouseEvent(rviz_common::ViewportMouseEvent &event);
  void onInitialPose(double x, double y, double theta, QString frame);
  void onNewGoal(double x, double y, double theta, QString frame);
  void getCoordinates();
  void closeGoalsWindow();
  void setInitialPose();


public:

  QLineEdit *actors;
  QLineEdit* output_topic_editor_;
  QLineEdit* output_topic_editor_1;
  QLineEdit* output_topic_editor_2;
  QLineEdit* coordinates;
  QLineEdit* coordinates1;
  QLineEdit* coordinates2;

  std::vector<YAML::Node> actors_info;
  std::vector<std::string> names;
  std::vector<std::string> point;

  // The current name of the output topic.
  QString output_topic_;

  // The current name of the output topic.
  QString output_topic_1;

  // The current name of the output topic.
  QString output_topic_2;

  QWidget *window = nullptr;
  QWidget *window1 = nullptr;
  QWidget *window2 = nullptr;
  
  bool first_actor = true;
  int num_actors;
  int iterate_actors = 1;
  int num_coordinates = 0;

  geometry_msgs::msg::PoseStamped initial_pose;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  geometry_msgs::msg::PoseStamped pose;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
  rclcpp::Node::SharedPtr g_node = nullptr;
  
  rclcpp::Node::SharedPtr client_node_;

  std::string flag_resource;

};

}

