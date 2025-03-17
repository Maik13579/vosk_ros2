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

# Install Python deps, audio libs, Vosk, wget/unzip, and required tools
RUN apt update \
 && apt install -y software-properties-common \
 && apt-add-repository universe \
 && apt update \
 && apt install -y \
    python3-pip python3-dev build-essential portaudio19-dev libasound2-dev \
    sphinxbase-utils \
    automake autoconf libtool \
    wget unzip \
    dos2unix \
 && python3 -m pip install --no-cache-dir vosk sounddevice \
 && rm -rf /var/lib/apt/lists/*

# Download, build, and install OpenFST
RUN wget https://www.openfst.org/twiki/pub/FST/FstDownload/openfst-1.8.2.tar.gz \
 && tar -xzvf openfst-1.8.2.tar.gz \
 && cd openfst-1.8.2 \
 && ./configure --enable-far --enable-static --enable-shared \
 && make -j4 \
 && make install \
 && cd .. \
 && rm -rf openfst-1.8.2 openfst-1.8.2.tar.gz

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

# Download and unzip a larger English speech model
RUN cd /opt/vosk_model \
 && wget https://alphacephei.com/vosk/models/vosk-model-en-us-0.22-lgraph.zip \
 && unzip vosk-model-en-us-0.22-lgraph.zip \
 && mv vosk-model-en-us-0.22-lgraph big_speech_model \
 && rm vosk-model-en-us-0.22-lgraph.zip

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

COPY scripts /scripts
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
RUN ldconfig

# Create new grammar
RUN /scripts/build_fst.sh /scripts/demo.jsgf /opt/vosk_model/big_speech_model
RUN /scripts/build_fst.sh /scripts/gpsr.jsgf /opt/vosk_model/big_speech_model