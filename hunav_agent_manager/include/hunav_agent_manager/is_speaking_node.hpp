#ifndef IS_SPEAKING_NODE_HPP_
#define IS_SPEAKING_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <mutex>
#include <string>

namespace hunav
{

    extern rclcpp::Node::SharedPtr hunav_manager_node;

    class IsSpeakingNode : public BT::StatefulActionNode
    {
    public:
        IsSpeakingNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config),
              agent_manager_(nullptr)
        {
        }

        IsSpeakingNode() = delete;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("agent_id"),
                BT::InputPort<double>("time_step"),
                BT::InputPort<int>("target_id"),
                BT::InputPort<double>("distance_threshold", 5.0, "Maximum distance for speech to be audible"),
                BT::InputPort<double>("duration", 10.0, "Listening duration in seconds")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int listener_id_;
        double dt_;
        int target_id_;
        double distance_threshold_;
        double duration_;
        std::chrono::steady_clock::time_point start_time_;
        AgentManager *agent_manager_;

    };

} // namespace hunav

#endif // IS_SPEAKING_NODE_HPP_
