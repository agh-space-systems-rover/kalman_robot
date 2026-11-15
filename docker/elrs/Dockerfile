FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
WORKDIR /ws

COPY kalman_interfaces src/kalman_interfaces
COPY kalman_master src/kalman_master
COPY kalman_wheels src/kalman_wheels

RUN apt-get update \
    && rosdep update --rosdistro "$ROS_DISTRO" \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

RUN sed -i "s|^exec \"\$@\"|source /ws/install/setup.bash\nexec \"\$@\"|" /ros_entrypoint.sh
