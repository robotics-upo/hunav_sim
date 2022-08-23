/*#include <QObject>

#include <memory>

#include <rviz_common/tool.hpp>

namespace rviz_panels
{

class GoalTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  GoalTool();
  ~GoalTool() override;

  void onInitialize() ;

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz_common::ViewportMouseEvent& event );

  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;

protected:
  void onPoseSet(double x, double y, double theta);
};

}  // namespace nav2_rviz_plugins
*/

#include <QObject>

#include <memory>

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/tools/point/point_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{

class DisplayContext;

namespace properties
{
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace hunav_rviz2_panel
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC GoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  GoalTool();
  ~GoalTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;
};

} 
