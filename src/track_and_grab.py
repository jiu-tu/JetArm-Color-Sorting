# -*- coding: utf-8 -*-
"""
track_and_grab.py — 视觉跟踪与抓取父类（重建版）
=================================================
项目：基于深度相机的 JetArm 六轴机械臂颜色分拣与定位夹取系统
平台：OrangePi AIPro（昇腾）+ ROS2 Humble + Orbbec Gemini 深度相机

说明
----
本文件依据补丁链（尤其 patch_track8：ColorTracker.proc 排除画面上方区域）
与板载代码片段整理，包含：
  - ColorTracker：基于 LAB 色彩空间阈值分割的颜色跟踪器（免训练），
    输出目标像素中心与伺服偏转量，供主循环做稳定判定与深度反投影；
  - 抓取参考实现（pick）：高位到位->下降->闭爪->抬升->回中->放置。
主程序见 sort_grab.py。
"""
import time
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from example.rgbd_function.include import common


class PID:
    """简易位置式 PID，用于云台（1/4 号舵机）伺服跟踪。"""

    def __init__(self, kp=0.5, ki=0.0, kd=0.0, limit=60.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.sum_err = 0.0
        self.last_err = 0.0

    def reset(self):
        self.sum_err = 0.0
        self.last_err = 0.0

    def update(self, err):
        self.sum_err += err
        self.sum_err = max(-self.limit, min(self.limit, self.sum_err))
        out = self.kp * err + self.ki * self.sum_err + \
            self.kd * (err - self.last_err)
        self.last_err = err
        return out


class ColorTracker:
    """LAB 色彩空间阈值分割跟踪器（免训练）。

    处理流程：
      RGB->LAB 颜色空间 → 按目标颜色阈值二值化 → 形态学开闭运算去噪 →
      找最大外轮廓 → 最小外接圆得到中心 → 排除画面上方干扰区 →
      依据中心与画面中心偏差计算云台偏转量（p_y = [pitch, yaw]）。
    """

    def __init__(self, color):
        self.color = color
        self.yaw = 500          # 云台舵机初始脉宽（1号）
        self.pitch = 150        # 俯仰舵机初始脉宽（4号）
        self.pid_yaw = PID()
        self.pid_pitch = PID()

    def proc(self, rgb_image, result_image, lab_data):
        """处理一帧：返回 (p_y, center, radius)；无目标时 p_y=None。"""
        h, w = rgb_image.shape[:2]
        lab = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB)
        lower, upper = lab_data.get(self.color, ((0, 0, 0), (255, 255, 255)))
        mask = cv2.inRange(lab, np.array(lower, dtype=np.uint8),
                           np.array(upper, dtype=np.uint8))
        # 形态学去噪：先开后闭
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        min_c = None            # (cx, cy, area)
        for c in contours:
            (center_x, center_y), radius = cv2.minEnclosingCircle(c)
            # patch_track8: 排除画面上方（色标板/背景绿在 y<0.35h 区域）
            if (center_y * 2) < 0.35 * h:
                continue
            if min_c is None or math.fabs(cv2.contourArea(c)) > min_c[2]:
                min_c = (int(center_x), int(center_y),
                         math.fabs(cv2.contourArea(c)))
        if min_c is None:
            return None, None, None
        center = (min_c[0], min_c[1])
        # 云台偏转：目标偏离画面中心越远, 云台角修正越大
        err_yaw = (center[0] - w / 2) / (w / 2)          # 水平偏差
        err_pitch = (center[1] - h * 0.6) / (h * 0.4)    # 垂直偏差（目标在下半区）
        yaw_out = self.pid_yaw.update(err_yaw)
        pitch_out = self.pid_pitch.update(err_pitch)
        p_y = [max(0, min(250, self.pitch - pitch_out)),
               max(0, min(1000, self.yaw + yaw_out))]
        return p_y, center, min_c[2]


class TrackAndGrab(Node):
    """跟踪与抓取父节点（参考实现）。"""

    def __init__(self):
        super().__init__('track_and_grab')
        self.running = True
        self.moving = False
        self.stamp = time.time()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_info = None
        self.joints_pub = self.create_publisher(Int32MultiArray, 'joints_pub', 10)
        self.set_pose_target_client = self.create_client(
            common.SetPoseTarget, 'set_pose_target')
        # 手眼标定矩阵
        self.hand2cam_tf_matrix = np.array([
            [0, 0, 1, -0.101],
            [-1, 0, 0, 0.011],
            [0, -1, 0, 0.045],
            [0, 0, 0, 1]], dtype=np.float64)

    def get_endpoint(self):
        """正运动学求解末端位姿（占位，板载通过服务获得）。"""
        self.endpoint = np.eye(4)
        return self.endpoint

    def send_request(self, client, msg):
        if not client.service_is_ready():
            return None
        return client.call(msg)

    def pick(self, position):
        """完整抓放动作（参考实现）：高位->下降->闭爪->抬升->回中->放置。"""
        if position[2] < 0.2:
            yaw = 80
        else:
            yaw = 30
        msg = common.set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res and res.pulse:
            servo_data = res.pulse
            # 1) 高位到达
            set_servo_position(self.joints_pub, 1, ((1, servo_data[0]),))
            time.sleep(1)
            # 2) 下降到位
            set_servo_position(self.joints_pub, 1.5,
                               ((1, servo_data[0]), (2, servo_data[1]),
                                (3, servo_data[2]), (4, servo_data[3]),
                                (5, servo_data[4])))
            time.sleep(1.5)
            # 3) 闭爪（10号=600）
            set_servo_position(self.joints_pub, 0.5, ((10, 600),))
            time.sleep(1)
            # 4) 抬升 6cm 再抓稳
            position[2] += 0.06
            msg = common.set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
            res = self.send_request(self.set_pose_target_client, msg)
            if res and res.pulse:
                servo_data = res.pulse
                set_servo_position(self.joints_pub, 1,
                                   ((1, servo_data[0]), (2, servo_data[1]),
                                    (3, servo_data[2]), (4, servo_data[3]),
                                    (5, servo_data[4])))
                time.sleep(1)
            # 5) 回中（放置前基准）
            set_servo_position(self.joints_pub, 1,
                               ((1, 500), (2, 720), (3, 100), (4, 120),
                                (5, 500), (10, 600)))
            time.sleep(1)
            # 6) 放置（示例：按目标颜色选择左侧/右侧固定关节角）
            place_j1 = 125 if self.target_color == 'red' else 350
            set_servo_position(self.joints_pub, 1,
                               ((1, place_j1), (2, 635), (3, 120), (4, 200), (5, 500)))
            time.sleep(1)
            set_servo_position(self.joints_pub, 1.5,
                               ((1, place_j1), (2, 550), (3, 120), (4, 150), (5, 500)))
            time.sleep(1.5)
            set_servo_position(self.joints_pub, 1,
                               ((1, place_j1), (2, 550), (3, 120), (4, 170),
                                (5, 500), (10, 200)))
            time.sleep(1.5)
            # 7) 复位
            set_servo_position(self.joints_pub, 1,
                               ((1, 500), (2, 720), (3, 100), (4, 150),
                                (5, 500), (10, 200)))
            time.sleep(2)
        else:
            self.get_logger().info('\033[1;31m%s\033[0m' % "Out of gripping range")
        self.moving = False


def set_servo_position(pub, duration, data):
    """发布舵机位置消息（1=舵机号, value=脉宽）。"""
    msg = Int32MultiArray()
    msg.data = [int(duration * 1000)] + [v for pair in data for v in pair]
    pub.publish(msg)
