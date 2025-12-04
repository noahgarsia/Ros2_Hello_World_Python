/**
 * @file test_subscriber.cpp
 * @brief Unit tests for the MinimalSubscriber class using Google Test.
 * Test 1: Verify subscriber callback is triggered when a message is received.
 * Test 2: Verify the subscriber correctly processes message content.
 * @date 2025-11-25
 * @author Noah
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "project_1/minimal_subscriber.hpp" 

// ---------------------------------------------------------------------------
// Test Fixture
// ---------------------------------------------------------------------------

class TestSubscriberNode : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node = std::make_shared<MinimalSubscriber>();
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    std::shared_ptr<MinimalSubscriber> node;
};


// ---------------------------------------------------------------------------
// TEST 1 — Verify subscriber callback is triggered
// ---------------------------------------------------------------------------

TEST_F(TestSubscriberNode, SubscriberCallbackTriggered)
{
    // Create a publisher to send data to the subscriber
    auto test_pub = node->create_publisher<std_msgs::msg::String>(
        "/project_1_example_topic",   
        10
    );

    // Publish a test message
    std_msgs::msg::String msg;
    msg.data = "3";
    test_pub->publish(msg);

    // Allow the subscriber to process the message
    rclcpp::spin_some(node);

    // Verify the subscriber stored the processed output
    EXPECT_EQ(node->get_last_message(), "I heard: '3'");
}
