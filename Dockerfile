# ===============================
# ROS 2 Humble + MoveIt2 + Gazebo Classic + BT.CPP
# ===============================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# -------------------------------
# 1. Install base + ROS 2 Humble
# -------------------------------
RUN apt update && apt install -y locales curl gnupg lsb-release && \
    locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt install -y software-properties-common && add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y \
    ros-humble-desktop-full \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-servo \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-msgs \
    ros-humble-ros2-control \
    ros-humble-controller-manager \
    ros-humble-control-toolbox \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control-test-assets \
    ros-humble-joint-trajectory-controller \
    ros-humble-ur-msgs \
    ros-humble-ur-client-library \
    gazebo \
    libgazebo-dev \
    python3-colcon-common-extensions \
    git build-essential cmake

# -------------------------------
# 2. Install BehaviorTree.CPP v4
# -------------------------------
RUN source /opt/ros/humble/setup.bash && \
    mkdir -p /root/libs && cd /root/libs && \
    git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
    cd BehaviorTree.CPP && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/humble && \
    make -j$(nproc) && make install

# -------------------------------
# 3. Workspace + sources
# -------------------------------
WORKDIR /root/ros2_ws
RUN mkdir -p src

COPY src/custom_ros_messages/ src/custom_ros_messages/
COPY src/arpa_control/ src/arpa_control/
COPY src/arpa_bt_executor/ src/arpa_bt_executor/
COPY src/arpa_description/ src/arpa_description/
COPY src/Universal_Robots_ROS2_Driver/ src/Universal_Robots_ROS2_Driver/
COPY src/Universal_Robots_ROS2_Gazebo_Simulation/ src/Universal_Robots_ROS2_Gazebo_Simulation/

# -------------------------------
# Runtime setup
# -------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENV ROS_DISTRO=humble

# -------------------------------
# Install deps
# -------------------------------
RUN apt update && apt install -y python3-rosdep python3-pip
RUN rosdep init || true
RUN rosdep update
RUN cd /root/ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Python deps for arpa_helper_tools (scan_battery TSP)
RUN python3 -m pip install --no-cache-dir ortools

# -------------------------------
# Build workspace
# -------------------------------
RUN source /opt/ros/humble/setup.bash && \
    cd /root/ros2_ws && colcon build --symlink-install

RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]