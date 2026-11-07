#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ -d tests ]]; then
  .venv/bin/python -m unittest discover -s tests -v
else
  echo 'Không có bộ tests local; chỉ kiểm tra cú pháp Python.'
fi
.venv/bin/python -m compileall -q src scripts
