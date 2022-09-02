#include "headers/goal_tool.hpp"

#include <memory>
#include <string>

#include "headers/goal_common.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"

namespace hunav_rviz2_panel
{

GoalTool::GoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';
}

GoalTool::~GoalTool()
{
}

void GoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("HunavGoals");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));
}

void
GoalTool::onPoseSet(double x, double y, double theta)
{
  // Set goal pose on global object GoalUpdater to update nav2 Panel
  GoalUpdater.setGoal(x, y, theta, context_->getFixedFrame());
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(hunav_rviz2_panel::GoalTool, rviz_common::Tool)