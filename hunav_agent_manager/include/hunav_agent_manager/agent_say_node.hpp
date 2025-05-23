#ifndef AGENT_SAY_NODE_HPP_
#define AGENT_SAY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>
#include <string>
#include <memory>

namespace hunav {

class AgentSayNode : public rclcpp::Node, public std::enable_shared_from_this<AgentSayNode>
{
public:
    AgentSayNode() : Node("agent_say_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("agent_say", rclcpp::QoS(10));
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "agent_say", rclcpp::QoS(10),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_message_ = msg->data;
            });

    }

    // Static setter to assign the instance after construction
    static void setInstance(std::shared_ptr<AgentSayNode> instance)
    {
        instance_ = instance;
    }

    void publishMessage(const std::string & msg_text)
    {
        std_msgs::msg::String msg;
        msg.data = msg_text;
        publisher_->publish(msg);
    }

    std::string getLastMessage()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_message_;
    }
    
    // Provide a static accessor to the unique instance
    static std::shared_ptr<AgentSayNode> getInstance()
    {
        return instance_;
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::mutex mutex_;
    std::string last_message_;

    // Static instance pointer
    static std::shared_ptr<AgentSayNode> instance_;
};

} // namespace hunav

#endif  // AGENT_SAY_NODE_HPP_
