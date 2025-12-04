#include "project_1/minimal_publisher.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Constructor implementation
MinimalPublisher::MinimalPublisher()
: Node("minimal_cpp_publisher"), counter_(0)
{
    publisher_ =
        create_publisher<std_msgs::msg::String>("/project_1_example_topic", 10);

    timer_ = create_wall_timer(
        500ms,
        std::bind(&MinimalPublisher::timer_callback, this)
    );

    RCLCPP_INFO(get_logger(), "Publishing at 2HZ");
}

// Timer callback implementation
void MinimalPublisher::timer_callback()
{
    counter_++;

    auto message = std_msgs::msg::String();
    message.data = "Hello, World!" + std::to_string(counter_);


    publisher_->publish(message);
}

