
#include "hunav_msgs/msg/agents.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hunav {

class HunavLoader : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Hunav Loader object
   *
   */
  HunavLoader();
  /**
   * @brief Destroy the Hunav Loader object
   *
   */
  ~HunavLoader();
};
} // namespace hunav