# Autoware runtime + visionpilot_msgs for the E2E adapter.
#
# The adapter consumes /vehicle/driving_reference (visionpilot_msgs), which
# the Autoware universe image does not ship. That image has a pruned
# toolchain (no cc1/Scrt1.o/crti.o, no fastcdr headers), so the messages
# are built in a clean ros:humble-ros-base stage and only the generated
# artifacts are copied into the runtime image. The runtime image keeps
# Autoware msgs/env, so the adapter can publish autoware_planning_msgs.
#
# Build with the visionpilot_msgs package as the context (build.sh does
# this):
#   docker build -f deploy/images/adapter.Dockerfile \
#     -t "${ADAPTER_IMAGE}" \
#     --build-arg "AUTOWARE_IMAGE=${AUTOWARE_IMAGE}" \
#     upstream/vision_pilot/VisionPilot/modules/middleware_interfaces/ros2_interface/visionpilot_msgs
ARG AUTOWARE_IMAGE=ghcr.io/autowarefoundation/autoware:universe-20250207

FROM ros:humble-ros-base AS msgs-builder
COPY . /src
RUN . /opt/ros/humble/setup.sh \
 && cmake -S /src -B /tmp/msg-build \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
      -DBUILD_TESTING=OFF \
 && cmake --build /tmp/msg-build --parallel "$(nproc)" \
 && cmake --install /tmp/msg-build

FROM ${AUTOWARE_IMAGE}
USER root
COPY --from=msgs-builder /opt/ros/humble/include/visionpilot_msgs /opt/ros/humble/include/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/lib/libvisionpilot_msgs__* /opt/ros/humble/lib/
COPY --from=msgs-builder /opt/ros/humble/local/lib/python3.10/dist-packages/visionpilot_msgs /opt/ros/humble/local/lib/python3.10/dist-packages/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/visionpilot_msgs /opt/ros/humble/share/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/packages/visionpilot_msgs /opt/ros/humble/share/ament_index/resource_index/packages/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/rosidl_interfaces/visionpilot_msgs /opt/ros/humble/share/ament_index/resource_index/rosidl_interfaces/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/package_run_dependencies/visionpilot_msgs /opt/ros/humble/share/ament_index/resource_index/package_run_dependencies/visionpilot_msgs
COPY --from=msgs-builder /opt/ros/humble/share/ament_index/resource_index/parent_prefix_path/visionpilot_msgs /opt/ros/humble/share/ament_index/resource_index/parent_prefix_path/visionpilot_msgs