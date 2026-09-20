#!/usr/bin/env bash

# 用法：source /home/yc/UR3/ros2_env.sh
source /opt/ros/humble/setup.bash
if [[ -f /home/yc/UR3/ros2_ws/install/setup.bash ]]; then
  source /home/yc/UR3/ros2_ws/install/setup.bash
fi
# 当前工作区使用隔离安装布局，显式加入自建包索引。
for project_prefix in /home/yc/UR3/ros2_ws/install/*; do
  if [[ -d "${project_prefix}/share/ament_index" ]]; then
    export AMENT_PREFIX_PATH="${project_prefix}:${AMENT_PREFIX_PATH}"
  fi
done
unset project_prefix
export ROS_DOMAIN_ID=33
export ROS_LOCALHOST_ONLY=0
