ARG ROS_DISTRO=humble
 
########################################
# Base Image for TurtleBot3 Simulation #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

 # Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Copy ros workspace
COPY ros2_ws ros2_ws/ 
 
# Build the base Colcon workspace, installing dependencies first.
WORKDIR /ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install \
 && sudo apt install python3-pykdl -y
 

CMD ["bash","-c","source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0; ros2 run era_5g_network_signal_mapper_ros2 signal_mapper;" ]