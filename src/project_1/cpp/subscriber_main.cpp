#include "rclcpp/rclcpp.hpp"
#include "project_1/minimal_subscriber.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MinimalSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
