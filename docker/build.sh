#!/bin/bash
set -e

# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

ROS_DISTRO=humble
IMAGE_TAG="coqui_tts_ros2"
DOWNLOAD_MODELS=true
BUILD_CUSTOM_MODELS=true

# Build Base image
docker build -t ${IMAGE_TAG} --target base \
  --build-arg BASE_IMAGE=ros:${ROS_DISTRO} \
  -f $PARENT_DIR/docker/Dockerfile \
  $PARENT_DIR

if $DOWNLOAD_MODELS; then
  docker build -t ${IMAGE_TAG} --target download_models\
    --build-arg BASE_IMAGE=${IMAGE_TAG} \
    -f $PARENT_DIR/docker/Dockerfile \
    $PARENT_DIR
fi

if $BUILD_CUSTOM_MODELS; then
  docker build -t ${IMAGE_TAG} --target build_custom_models\
    --build-arg BASE_IMAGE=${IMAGE_TAG} \
    -f $PARENT_DIR/docker/Dockerfile \
    $PARENT_DIR
fi

#Build vosk and ros wrapper
docker build -t ${IMAGE_TAG} --target build\
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f $PARENT_DIR/docker/Dockerfile \
  $PARENT_DIR