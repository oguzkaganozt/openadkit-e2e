# Domain bridge image for the E2E rig: the ROS 2 domain_bridge needs the
# workspace message types installed in its own environment (it resolves
# topic types by import), and this rig bridges visionpilot_msgs
# (driving_command 1->2) besides the Autoware types. The adapter image
# already carries visionpilot_msgs + safety_island_msgs on top of the
# Autoware runtime, so base on it and only add the bridge package (same
# ROS expired-key fix the Safety Island demo Dockerfile uses).
#
# Build (deploy/build.sh does this, after the adapter image exists):
#   docker build -f deploy/images/domain-bridge.Dockerfile \
#     -t "${BRIDGE_IMAGE}" \
#     --build-arg "ADAPTER_IMAGE=${ADAPTER_IMAGE}" deploy/images
ARG ADAPTER_IMAGE=openadkit-e2e-adapter:latest

FROM ${ADAPTER_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO=humble
USER root

# Fix the expired ROS GPG key issue comprehensively.
# See: https://github.com/osrf/docker_images/issues/535
RUN apt-get update --allow-unauthenticated || true && \
    apt-get install -y --allow-unauthenticated curl gnupg2 && \
    apt-key del F42ED6FBAB17C654 || true && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key && \
    cat /tmp/ros.key | gpg --dearmor --batch --yes -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    rm /tmp/ros.key

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y "ros-${ROS_DISTRO}-domain-bridge"