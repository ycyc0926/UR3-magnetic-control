#!/usr/bin/env bash
set -eo pipefail
source /home/yc/UR3/ros2_env.sh
set -u
if pgrep -x arv-viewer-0.8 >/dev/null || pgrep -x SpinView_QT >/dev/null; then
  echo '请先关闭占用相机的 arv-viewer 或 SpinView，然后重新运行。' >&2
  exit 1
fi
# Full sensor FOV is required for the saved plane calibration. These are
# volatile acquisition settings; exposure/gain and saved UserSets are untouched.
arv-tool-0.8 control TriggerMode=Off Width=8 Height=6 OffsetX=0 OffsetY=0 Width=2448 Height=2048 PixelFormat=BayerRG8
exec ros2 launch /home/yc/UR3/camera/h_vision.launch.py
