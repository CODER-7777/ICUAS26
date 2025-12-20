#!/bin/bash

cd ~/ros2_ws
colcon build --merge-install --packages-select octomap_map_generator
source install/setup.bash
./src/icuas26_competition/startup/start.sh
