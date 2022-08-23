#ifndef HUNAV_RVIZ2_PANEL__GOAL_POSE_UPDATER_HPP_
#define HUNAV_RVIZ2_PANEL__GOAL_POSE_UPDATER_HPP_

#include <QObject>

namespace hunav_rviz2_panel
{

class GoalPoseUpdater : public QObject
{
  Q_OBJECT

public:
  GoalPoseUpdater() {}
  ~GoalPoseUpdater() {}

  void setGoal(double x, double y, double theta, QString frame)
  {
    emit updateGoal(x, y, theta, frame);
  }

signals:
  void updateGoal(double x, double y, double theta, QString frame);
};

} 

#endif 
