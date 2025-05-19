#ifndef TIME_DELAY_DECORATOR_HPP_
#define TIME_DELAY_DECORATOR_HPP_

#include "behaviortree_cpp/decorator_node.h"
#include <chrono>

namespace hunav {

class TimeDelayDecorator : public BT::DecoratorNode
{
public:
    TimeDelayDecorator(const std::string& name, const BT::NodeConfig& config)
      : BT::DecoratorNode(name, config),
        delay_(0.0),
        start_time_()
    {}

    // Input port for delay (in seconds). Default is 1.0 second.
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("delay", 1.0, "Delay time in seconds before ticking the child") };
    }

    // The tick function: while the delay hasn’t elapsed, return FAILURE.
    // Once the delay has passed, tick the child and return its status.
    BT::NodeStatus tick() override;

    // When halted, reset the timer.
    void halt() override;

private:
    double delay_;
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace hunav

#endif  // TIME_DELAY_DECORATOR_HPP_
