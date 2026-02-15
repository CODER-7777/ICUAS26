#!/bin/bash
cd ~/workspace
colcon build --packages-select aruco_mission_cpp graph_traversal
source install/setup.bash 
cd ~/ros2_ws/src/icuas26_competition/startup
source _setup.sh
# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln session.yml .tmuxinator.yml

SETUP_NAME=$1
[ -z "$SETUP_NAME" ] && SETUP_NAME=_setup.sh 

# staxrt tmuxinator
tmuxinator icuas_competition_example.yml setup_name=$SETUP_NAME