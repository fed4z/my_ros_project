# Use an official ROS base image
FROM ros:foxy

# Set the working directory in the container
WORKDIR /workspace

# Copy the entire project into the container
COPY . .

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup.bash
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash"

# Install rosdep and update
RUN apt-get update && apt-get install -y python3-rosdep
RUN rosdep init && rosdep update

# Install dependencies from package.xml
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

# Define default command
CMD ["bash"]