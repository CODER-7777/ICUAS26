#!/bin/bash
podman run -it --net host -e DISPLAY=host.containers.internal:0 -v "$(pwd)/src:/root/ros2_ws/src" icuas tmux
