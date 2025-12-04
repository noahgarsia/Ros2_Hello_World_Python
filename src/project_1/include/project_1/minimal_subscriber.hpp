#ifndef MINIMAL_SUBSCRIBER_HPP
#define MINIMAL_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber();
    std::string get_last_message() const { return last_print_; }


private:
    void topic_callback(const std_msgs::msg::String & msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    std::string last_print_;
};

#endif // MINIMAL_SUBSCRIBER_HPP
