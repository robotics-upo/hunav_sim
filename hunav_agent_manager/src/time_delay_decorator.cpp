#include "hunav_agent_manager/time_delay_decorator.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include <cmath>
#include <iostream>

namespace hunav {

BT::NodeStatus TimeDelayDecorator::tick()
{
    // Get the delay value from the input port.
    if (!getInput<double>("delay", delay_))
    {
         // Default to 1.0 second if not provided.
         delay_ = 1.0;
    }

    // On the first tick, record the start time.
    if (start_time_ == std::chrono::steady_clock::time_point())
    {
         start_time_ = std::chrono::steady_clock::now();
         return BT::NodeStatus::FAILURE; // Still waiting.
    }
    
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
    
    if (elapsed < delay_)
    {
         // Delay not yet elapsed: do not tick the child.
         return BT::NodeStatus::FAILURE;
    }
    else
    {
         // Delay elapsed: tick the child and return its status.
         return child_node_->executeTick();
    }
}

void TimeDelayDecorator::halt()
{
    // Reset the timer so that when restarted, it begins anew.
    start_time_ = std::chrono::steady_clock::time_point();
    BT::DecoratorNode::halt();
}

} // namespace hunav
