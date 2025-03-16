ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE}

#environment variables
ENV ROS_WS=/root/ros2_ws
ENV DEBIAN_FRONTEND=noninteractive

#ros2 DDS implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#use bash
SHELL ["/bin/bash", "-c"]

#source ros stuff
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Install RMW
RUN apt update \
    && apt install -y \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Install Python deps (pip, etc.), audio libs, and Vosk
RUN apt update \
 && apt install -y python3-pip python3-dev build-essential portaudio19-dev libasound2-dev \
 && python3 -m pip install --no-cache-dir vosk \
 && rm -rf /var/lib/apt/lists/*

# Install dependencies
COPY vosk_ros2_interfaces/package.xml ${ROS_WS}/src/vosk_ros2_interfaces/package.xml

RUN apt update \
    && rosdep update \
    && rosdep install --from-paths ${ROS_WS}/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build
COPY vosk_ros2_interfaces ${ROS_WS}/src/vosk_ros2_interfaces

WORKDIR ${ROS_WS}
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc
