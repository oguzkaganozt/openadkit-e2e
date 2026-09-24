# Autoware runtime + VP/SI message packages for the E2E adapter (and the
# SI probes / future CARLA actuator).
#
# The adapter consumes /vehicle/driving_reference (visionpilot_msgs) and
# later the actuator reads /control/safety_island/approved_request
# (safety_island_msgs); the Autoware universe image ships neither and its
# pruned toolchain (no cc1/Scrt1.o/crti.o, no fastcdr headers) cannot build
# them. safety_island_msgs additionally depends on autoware_control_msgs,
# which only exists in the Autoware install tree, so that tree is brought
# into the builder stage and sourced there. Only the generated artifacts are
# copied into the runtime image.
#
# Build with the message package directories as named contexts
# (deploy/build.sh does this):
#   docker build -f deploy/images/adapter.Dockerfile \
#     -t "${ADAPTER_IMAGE}" \
#     --build-arg "AUTOWARE_IMAGE=${AUTOWARE_IMAGE}" \
#     --build-context "vpmsgs=upstream/vision_pilot/VisionPilot/modules/middleware_interfaces/ros2_interface/visionpilot_msgs" \
#     --build-context "simsgs=safety_island_msgs" deploy/images
ARG AUTOWARE_IMAGE=ghcr.io/autowarefoundation/autoware:universe-20250207

FROM ${AUTOWARE_IMAGE} AS autoware

FROM ros:humble-ros-base AS msgs-builder
# The Autoware setup script is bash-only (bashisms inside local_setup.bash
# die under dash with "Bad substitution"), so run every step with bash.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=autoware /opt/autoware /opt/autoware
COPY --from=vpmsgs / /src/visionpilot_msgs
COPY --from=simsgs / /src/safety_island_msgs
RUN . /opt/ros/humble/setup.sh \
 && . /opt/autoware/setup.bash \
 && for pkg in visionpilot_msgs safety_island_msgs; do \
      cmake -S "/src/$pkg" -B "/tmp/build/$pkg" \
            -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
            -DBUILD_TESTING=OFF \
     && cmake --build "/tmp/build/$pkg" --parallel "$(nproc)" \
     && cmake --install "/tmp/build/$pkg"; \
    done

FROM ${AUTOWARE_IMAGE}
USER root
COPY --from=msgs-builder /opt/ros/humble/include/visionpilot_msgs /opt/ros/humble/include/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/include/safety_island_msgs /opt/ros/humble/include/safety_island_msgs
COPY --from=msgs-builder /opt/ros/humble/lib/libvisionpilot_msgs__* /opt/ros/humble/lib/
COPY --from=msgs-builder /opt/ros/humble/lib/libsafety_island_msgs__* /opt/ros/humble/lib/
COPY --from=msgs-builder /opt/ros/humble/local/lib/python3.10/dist-packages/visionpilot_msgs /opt/ros/humble/local/lib/python3.10/dist-packages/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/local/lib/python3.10/dist-packages/safety_island_msgs /opt/ros/humble/local/lib/python3.10/dist-packages/safety_island_msgs
COPY --from=msgs-builder /opt/ros/humble/share/visionpilot_msgs /opt/ros/humble/share/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/safety_island_msgs /opt/ros/humble/share/safety_island_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/packages/visionpilot_msgs /opt/ros/humble/share/ament_index/resource_index/packages/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/packages/safety_island_msgs /opt/ros/humble/share/ament_index/resource_index/packages/safety_island_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/rosidl_interfaces/visionpilot_msgs /opt/ros/humble/share/ament_index/resource_index/rosidl_interfaces/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/rosidl_interfaces/safety_island_msgs /opt/ros/humble/share/ament_index/resource_index/rosidl_interfaces/safety_island_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/package_run_dependencies/visionpilot_msgs /opt/ros/humble/share/ament_index/resource_index/package_run_dependencies/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/package_run_dependencies/safety_island_msgs /opt/ros/humble/share/ament_index/resource_index/package_run_dependencies/safety_island_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/parent_prefix_path/visionpilot_msgs /opt/ros/humble/share/ament_index/resource_index/parent_prefix_path/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/parent_prefix_path/safety_island_msgs /opt/ros/humble/share/ament_index/resource_index/parent_prefix_path/safety_island_msgs