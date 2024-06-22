FROM osrf/ros:humble-desktop-full-jammy

#setup
ENV DEBIAN_FRONTEND="noninteractive"

COPY ./root_dir/shell_scripts/basic_util.sh /root/
RUN cd /root/ && sudo chmod +x * && ./basic_util.sh && rm -rf basic_util.sh
COPY ./root_dir/shell_scripts/colcon.sh /root/
RUN cd /root/ && sudo chmod +x * && ./colcon.sh && rm -rf colcon.sh
COPY ./root_dir/shell_scripts/ros_package_install_essential.sh /root
RUN cd /root/ && sudo chmod +x * && ./ros_package_install_essential.sh && rm -rf ros_package_install_essential.sh
RUN apt-get update && apt-get upgrade -y
COPY ./root_dir/ros2_ws/ /root/ros2_ws/
RUN cd /root/ros2_ws/ && rosdep install --from-paths src --ignore-src -r -y