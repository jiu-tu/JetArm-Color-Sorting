#!/usr/bin/env bash
# 功能二：颜色分拣 —— 识别正前方红/绿/蓝积木，放置到左侧对应色标区
# 用法: bash scripts/start_color.sh
source /root/ros2_ws/install/setup.bash
ros2 run example sort_grab --ros-args -p mode:=color -p start:=true
