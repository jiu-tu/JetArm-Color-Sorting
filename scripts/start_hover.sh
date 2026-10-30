#!/usr/bin/env bash
# 悬停标定：检测到目标后仅悬停于其上方 6cm（不闭爪），
# 保存 /tmp/hover.jpg 并打印 pose_t / HOVER actual 后退出
# 用法: bash scripts/start_hover.sh
source /root/ros2_ws/install/setup.bash
ros2 run example sort_grab --ros-args -p hover:=true -p start:=true
