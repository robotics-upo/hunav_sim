
#include "hunav_msgs/msg/agents.hpp"
#include "hunav_msgs/srv/get_parameters.hpp"
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

private:
  /**
   * @brief Service callback to provide parameters to other nodes
   * 
   * @param request Empty request
   * @param response Parameters response
   */
  void getParametersService(const std::shared_ptr<hunav_msgs::srv::GetParameters::Request> request,
                           std::shared_ptr<hunav_msgs::srv::GetParameters::Response> response);

  // Service server
  rclcpp::Service<hunav_msgs::srv::GetParameters>::SharedPtr get_parameters_srv_;
};
} // namespace hunav