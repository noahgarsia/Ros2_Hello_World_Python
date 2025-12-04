FROM ros:jazzy

# Install required build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace inside the container
RUN mkdir -p /ros2_ws_ccp/src
WORKDIR /ros2_ws_ccp

# Copy ONLY your src directory into the container
COPY src/ /ros2_ws_ccp/src/

# Build the workspace inside the container
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Automatically source ROS2 + workspace when container starts
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws_ccp/install/setup.bash && exec bash"]
