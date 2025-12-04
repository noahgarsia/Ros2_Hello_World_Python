#include "project_1/minimal_subscriber.hpp"


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

MinimalSubscriber::MinimalSubscriber()
: Node(
    "minimal_cpp_subscriber",
    rclcpp::NodeOptions().use_intra_process_comms(false)
)
{
    // Disable topic statistics to avoid unnecessary overhead
    rclcpp::SubscriptionOptions options;
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;

    // Create the subscriber
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/project_1_example_topic",
        rclcpp::QoS(10),
        std::bind(&MinimalSubscriber::topic_callback, this, _1),
        options
    );
}

void MinimalSubscriber::topic_callback(const std_msgs::msg::String & msg) 
{
    // Build the exact same string that RCLCPP_INFO prints
    last_print_ = "I heard: '" + msg.data + "'";

    // Print it
    RCLCPP_INFO(this->get_logger(), "%s", last_print_.c_str());
}
