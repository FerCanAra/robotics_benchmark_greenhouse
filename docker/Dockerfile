# Greenhouse benchmark: ROS 2 Humble + Nav2 + MVSim + this repository.
#
# Build from the repository root, with its submodules checked out:
#   docker build -f docker/Dockerfile -t greenhouse-benchmark .
# Run a campaign with docker/run_campaign.sh (see the README).
FROM ros:humble-ros-base

# MVSim version (a tag, branch or commit of https://github.com/MRPT/mvsim),
# the one this version of the benchmark was validated with:
ARG MVSIM_REF=2cc54561

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Nav2 from the main ROS 2 repository, held so that it is not upgraded below.
# RViz (with the Nav2 plugins) and foxglove_bridge are only used for visualization:
RUN apt-get update && \
    apt-get install -y git ros-humble-navigation2 ros-humble-nav2-bringup python3-scipy \
      ros-humble-rviz2 ros-humble-nav2-rviz-plugins ros-humble-foxglove-bridge && \
    apt-mark hold $(dpkg-query -W -f='${Package}\n' 'ros-humble-nav2*' 'ros-humble-navigation2') && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN git clone https://github.com/MRPT/mvsim.git src/mvsim && \
    git -C src/mvsim checkout ${MVSIM_REF} && \
    git -C src/mvsim submodule update --init --recursive
COPY . src/robotics_benchmark_greenhouse

# MRPT 3 (required by MVSim) is only in the ros2-testing repository for Humble:
RUN sed 's|/ros2/ubuntu|/ros2-testing/ubuntu|' /etc/apt/sources.list.d/ros2.sources \
      > /etc/apt/sources.list.d/ros2-testing.sources && \
    apt-get update && rosdep update --rosdistro humble && \
    rosdep install --from-paths src --ignore-src -y -r --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

# --symlink-install keeps the installed packages linked to their git sources,
# whose commits run_batch.py records in the manifest of every campaign:
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf log && \
    git config --system --add safe.directory '*'

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
