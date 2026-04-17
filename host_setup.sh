#!/bin/bash
# host_setup.sh - Wayland/Xwayland Setup

echo "Opening Xwayland bridge for local container..."
# You may need to run 'sudo pacman -S xorg-xhost' if you don't have this installed
xhost +local:

# Hook to the current SSH_AUTH_LOCK
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
