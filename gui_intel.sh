#!/bin/bash
IMG=localhost/crazysim_icuas_img
CMD=bash
USER_NAME=shivang

podman run --security-opt label=disable \
  --device=/dev/dri \
  -e XDG_RUNTIME_DIR=/tmp \
  -e DISPLAY \
  -e XAUTHORITY=$XAUTHORITY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.local/share/JetBrains/:/home/$USER_NAME/.local/share/JetBrains \
  -v $HOME/.config/JetBrains/:/root/.config/JetBrains \
  -v $HOME/.cache/JetBrains/:/root/.cache/JetBrains \
  -v $PWD:/root/icuas26 \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e MESA_LOADER_DRIVER_OVERRIDE=iris \
  -e __GLX_VENDOR_LIBRARY_NAME=mesa \
  -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY:ro \
  --userns=keep-id \
  --group-add keep-groups \
  --net host \
  --name icuas26 \
  -it $IMG $CMD
