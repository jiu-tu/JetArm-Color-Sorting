#!/usr/bin/env bash
# 功能一：定位夹取 —— 识别正前方任意颜色物块，放置到机械臂右侧指定区
# 用法: bash scripts/start_right.sh
source /root/ros2_ws/install/setup.bash
ros2 run example sort_grab --ros-args -p mode:=right -p start:=true
