#!/bin/bash

# Load workspace overlay
if [ -f ~/ros2_ws_cpp/install/setup.bash ]; then
    source ~/ros2_ws_cpp/install/setup.bash
fi

cleanup() {
    echo "Stopping ROS2 nodes..."
    pkill -f "cpp_minimal_publisher"
    pkill -f "cpp_minimal_subscriber"

    echo "Restarting ROS2 daemon..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
}

# Ensure cleanup runs if user presses Ctrl+C
trap cleanup SIGINT

echo "Launching publisher in background..."
setsid ros2 run project_1 cpp_minimal_publisher >/dev/null 2>&1 &

echo "Launching subscriber in background..."
ros2 run project_1 cpp_minimal_subscriber &
pid_sub=$!

echo "Killing subscriber after 5 seconds..."
sleep 5
kill $pid_sub
sleep 1

##########################################################
# CLEANUP BEFORE TESTS
##########################################################
echo "Performing cleanup before tests..."
cleanup
sleep 1

##########################################################
# RUN C++ TESTS USING COLCON
##########################################################
echo "-----------------------------------"
echo "Running C++ tests with colcon test"
echo "-----------------------------------"

colcon test --packages-select project_1
echo "-----------------------------------"
echo "Test results:"
colcon test-result --all --verbose
echo "-----------------------------------"

cleanup
echo "All operations completed."
