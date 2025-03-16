ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE}

# Environment variables
ENV ROS_WS=/root/ros2_ws
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Use bash
SHELL ["/bin/bash", "-c"]

# Source ROS entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Install Cyclone DDS
RUN apt update \
 && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Install Python deps (pip, etc.), audio libs, Vosk,
# and also wget/unzip for downloading models
RUN apt update \
 && apt install -y \
    python3-pip python3-dev build-essential portaudio19-dev libasound2-dev \
    wget unzip \
 && python3 -m pip install --no-cache-dir vosk sounddevice \
 && rm -rf /var/lib/apt/lists/*

# Download and unzip a small English speech model
RUN mkdir -p /opt/vosk_model \
 && cd /opt/vosk_model \
 && wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip \
 && unzip vosk-model-small-en-us-0.15.zip \
 && mv vosk-model-small-en-us-0.15 speech_model \
 && rm vosk-model-small-en-us-0.15.zip

# Download and unzip a speaker recognition model
RUN cd /opt/vosk_model \
 && wget https://alphacephei.com/vosk/models/vosk-model-spk-0.4.zip \
 && unzip vosk-model-spk-0.4.zip \
 && mv vosk-model-spk-0.4 speaker_model \
 && rm vosk-model-spk-0.4.zip

# Copy package.xml files to trigger dependency installation
COPY vosk_ros2_interfaces/package.xml ${ROS_WS}/src/vosk_ros2_interfaces/package.xml
COPY vosk_ros2/package.xml ${ROS_WS}/src/vosk_ros2/package.xml

RUN apt update \
 && rosdep update \
 && rosdep install --from-paths ${ROS_WS}/src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*

# Copy source and build
COPY vosk_ros2_interfaces ${ROS_WS}/src/vosk_ros2_interfaces
COPY vosk_ros2 ${ROS_WS}/src/vosk_ros2

WORKDIR ${ROS_WS}
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc
