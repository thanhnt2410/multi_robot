#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
# System setuptools matches Humble's colcon; user-site setuptools may not.
PYTHONNOUSERSITE=1 colcon build --symlink-install "$@"
