#ifndef MINIMAL_PUBLISHER_HPP_
#define MINIMAL_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher();

    int get_counter() const { return counter_; }

    
    // void test_tick() { timer_callback(); }

private:
    void timer_callback();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

#endif  // MINIMAL_PUBLISHER_HPP_
