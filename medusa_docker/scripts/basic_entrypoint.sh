#!/bin/sh
set -e

# Setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"