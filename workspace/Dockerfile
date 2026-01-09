from ros:humble
run apt-get update && apt-get install -y neovim tmux ripgrep fd-find
run apt-get update && apt-get install -y \
    ros-humble-octomap-ros ros-humble-ament-cmake \
    ros-humble-octomap-msgs qtbase5-dev qt5-qmake \
    python3-opencv libopencv-dev
workdir /root/ros2_ws
copy ./src /root/ros2_ws/src
run . /opt/ros/humble/setup.sh && colcon build --symlink-install
run rm -rf /root/ros2_ws/src
cmd [ "bash" ]
