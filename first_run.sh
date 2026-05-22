#!/bin/bash

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
  if [ ! -z "$xauth_list" ]; then
    echo $xauth_list | xauth -f $XAUTH nmerge -
  else
    touch $XAUTH
  fi
  chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

REPO_ROOT=$(pwd)

# Hook to the current SSH_AUTH_LOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

# --env SSH_AUTH_SOCK=/ssh-agent \
# --gpus '"all","capabilities=compute,utility,graphics"' \

docker run -it \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="TERM=xterm-256color" \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -v /tmp/.docker.xauth:/tmp/.docker.xauth:ro \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev:/dev" \
  --volume="$HOME/.Xauthority:/root/.Xauthority" \
  --volume="/var/run/dbus/:/var/run/dbus/:z" \
  --volume="$PWD/workspace:/root/workspace" \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  --net=host \
  --privileged \
  --gpus all \
  --device /dev/dri \
  --name crazysim_icuas_cont \
  crazysim_icuas_img
