FROM ros:foxy-ros-base-focal

RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN apt install -y python3-pykdl

# install ros2 packages
RUN apt-get install -y ros-foxy-sensor-msgs-py

RUN mkdir -p ~/ros2_ws/src
RUN cd ~/ros2_ws
WORKDIR /root/ros2_ws
COPY src src
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
colcon build --symlink-install"

# Source the workspace setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc
