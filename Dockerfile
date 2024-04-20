# Use a base image with ROS2 already installed
FROM ros:humble

# Install ROS2
RUN apt-get update && \
    apt-get install -y \
        ros-dev-tools \
        python3-colcon-common-extensions

# Install Turtlebot3 and Nav2
RUN apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-turtlebot3-gazebo

# Clean up
RUN rm -rf /var/lib/apt/lists/*

# Set up entry point
CMD ["bash"]

