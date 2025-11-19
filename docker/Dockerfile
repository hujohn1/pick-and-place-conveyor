FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash","-lc"]

# --- Base utilities & ROS deps (minimo utile) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake python3-pip nano curl wget \
    ros-dev-tools python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros gazebo libgazebo-dev \
    ros-humble-gazebo-ros2-control \
    ros-humble-moveit ros-humble-moveit-ros-visualization \
    ros-humble-xacro ros-humble-joint-state-publisher-gui \
    mesa-utils libgl1-mesa-dri libxkbcommon0 \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-realtime-tools ros-humble-control-toolbox ros-humble-control-msgs \
    mesa-utils \
    x11-apps \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# --- Workspace layout ---
RUN mkdir -p /workspaces/app_ws/src
WORKDIR /workspaces/app_ws

# --- Import external repository ---
COPY ./deps.repos /workspaces/app_ws/deps.repos
RUN vcs import src < deps.repos

# Copy your package 
COPY ./src/ur5_simulation /workspaces/app_ws/src/ur5_simulation
COPY ./src/ur5_camera_gripper_moveit_config /workspaces/app_ws/src/ur5_camera_gripper_moveit_config

# rosdep: install all the dependencies (from package.xml)
RUN rosdep update || true && \
	rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true

# compile all the overlay
RUN source /opt/ros/humble/setup.bash && \
	colcon build --packages-skip robotiq_driver robotiq_hardware_tests

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Base environment
ENV ROS_DOMAIN_ID=0
ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/entrypoint.sh"]
