FROM ros:noetic-ros-base-focal

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libyaml-cpp-dev \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-pip \
    ros-noetic-costmap-2d \
    ros-noetic-geometry-msgs \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-nav-core \
    ros-noetic-nav-msgs \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-stage-ros \
    ros-noetic-turtlebot3-navigation \
    ros-noetic-base-local-planner \
    ros-noetic-pluginlib \
    ros-noetic-roslint \
    ros-noetic-roslaunch \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /catkin_ws/src && chown -R $USERNAME:$USERNAME /catkin_ws

# Switch to non-root user
USER $USERNAME
WORKDIR /catkin_ws

# Clone and build jps3d library
RUN cd /catkin_ws/src && \
    git clone https://github.com/KumarRobotics/jps3d.git && \
    cd jps3d && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/home/$USERNAME/.local && \
    make -j$(nproc) && \
    make install

# Set environment variables for jps3d
ENV CMAKE_PREFIX_PATH=/home/$USERNAME/.local:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=/home/$USERNAME/.local/lib:$LD_LIBRARY_PATH

# Setup bash
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /catkin_ws/devel/setup.bash ]; then source /catkin_ws/devel/setup.bash; fi" >> /home/$USERNAME/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc

# Set the entrypoint
COPY --chown=$USERNAME:$USERNAME .devcontainer/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
