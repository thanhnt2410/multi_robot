#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec .venv/bin/python /opt/ros/humble/bin/ros2 launch multi_robot_bringup patrol.launch.py "$@"
