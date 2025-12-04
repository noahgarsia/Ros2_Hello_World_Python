//
// @__FILE__ test_publisher.cpp
//
// @brief Simple unit test for the MinimalPublisher node
//
// Description:
// This test suite initialises ROS2 before each test and shuts it down
// afterwards (handled by the GTest fixture). The fixture creates a fresh
// MinimalPublisher node for every test to ensure clean isolation.
//
// The tests check:
// 1. Node name is correct.
// 2. Only ONE publisher exists on the topic.
// 3. The publisher actually publishes "Hello, World!" messages.
//
// ---------------------------------------------------------------------------
// Includes
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <chrono>

#include "project_1/minimal_publisher.hpp"

// Enable 100ms / 600ms chrono literals
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class TestPublisherNode : public ::testing::Test 
{
protected:
    void SetUp() override 
    {
        rclcpp::init(0, nullptr);
        node = std::make_shared<MinimalPublisher>();
    }

    void TearDown() override 
    {
        rclcpp::shutdown();
    }

    std::shared_ptr<MinimalPublisher> node;
};

// ---------------------------------------------------------------------------
// TEST 1: Node name + number of publishers
// ---------------------------------------------------------------------------

TEST_F(TestPublisherNode, SimplePublisherCheck)
{
    EXPECT_EQ(node->get_name(), std::string("minimal_cpp_publisher"));

    auto pub_info =
        node->get_publishers_info_by_topic("/project_1_example_topic");

    EXPECT_EQ(pub_info.size(), 1)
        << "Expected exactly one publisher on '/project_1_example_topic'.";
}

// ---------------------------------------------------------------------------
// TEST 2: Publisher actually sends the message
// ---------------------------------------------------------------------------

TEST_F(TestPublisherNode, TestMessageContent)
{
    std::shared_ptr<std_msgs::msg::String> received_msg = nullptr;

    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "/project_1_example_topic",
        10,
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            received_msg = msg;
        }
    );

    auto start = std::chrono::steady_clock::now();
    while (!received_msg &&
           (std::chrono::steady_clock::now() - start) < std::chrono::seconds(1))
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    ASSERT_NE(received_msg, nullptr) << "No message received after 1 second.";

    EXPECT_TRUE(received_msg->data.rfind("Hello, World!", 0) == 0)
        << "Message does not begin with 'Hello, World!'";
}

// ---------------------------------------------------------------------------
// TEST 3: Counter increments using the timer callback
// ---------------------------------------------------------------------------

TEST_F(TestPublisherNode, MessageCounterIncrement)
{
    EXPECT_EQ(node->get_counter(), 0);

    for (int expected = 1; expected <= 10; ++expected)
    {
        std::this_thread::sleep_for(600ms); 

        rclcpp::spin_some(node);

        EXPECT_EQ(node->get_counter(), expected)
            << "The subscriber counter did not increment as expected.";
    }
}

