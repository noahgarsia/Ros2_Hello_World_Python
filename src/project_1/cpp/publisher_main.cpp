#include "rclcpp/rclcpp.hpp"
#include "project_1/minimal_publisher.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
