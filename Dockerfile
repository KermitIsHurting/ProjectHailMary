# CPU-only build image for ProjectHailMary (CI / x86_64 dev box).
# TensorRT / CUDA are absent from ros:humble, so the detector library and
# inference_node are excluded via -DCUAS_BUILD_GPU=OFF; every other node and
# every GoogleTest binary builds and runs here.
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /ws

# Copy manifests first so the dependency layer is cached across source edits.
COPY msgs/cuas_msgs/package.xml src/cuas_msgs/package.xml
COPY src/cuas_fusion/package.xml src/cuas_fusion/package.xml

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      libopencv-dev libeigen3-dev libyaml-cpp-dev \
 && (rosdep update --rosdistro humble || rosdep update) \
 && rosdep install --from-paths src --ignore-src -y -r \
      --skip-keys "rviz2 gstreamer1.0 libopencv-dev" \
 && rm -rf /var/lib/apt/lists/*

# Full sources. The workspace is flat (msgs/ + src/ both become src/<pkg>).
COPY msgs/cuas_msgs src/cuas_msgs
COPY src/cuas_fusion src/cuas_fusion
COPY test test
COPY scripts scripts

RUN source /opt/ros/humble/setup.bash \
 && colcon build --packages-select cuas_msgs cuas_fusion \
      --event-handlers console_direct+ \
      --cmake-args -DCUAS_WERROR=ON -DCUAS_BUILD_GPU=OFF

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && colcon test --packages-select cuas_fusion --event-handlers console_direct+ && colcon test-result --verbose"]
