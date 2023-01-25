FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    pkg-config \
    python3-pip

# Install additional Python modules
RUN pip3 install pyserial

# Install additional ROS packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
# ros-foxy-cartographer \

# Create Colcon workspace with external dependencies
RUN mkdir -p /lola2/src
# WORKDIR /lola2/src
# COPY dependencies.repos ./
# RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing any extra missed dependencies.
WORKDIR /lola2

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
  && apt-get update -y \
  && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# WORKDIR /lola2
# RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build