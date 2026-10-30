# -*- coding: utf-8 -*-
"""
sort_grab.py — 颜色分拣与定位夹取主程序（重建版）
==================================================
项目：基于深度相机的 JetArm 六轴机械臂颜色分拣与定位夹取系统
平台：OrangePi AIPro（昇腾）+ ROS2 Humble + Orbbec Gemini 深度相机

说明
----
本文件依据 14 轮补丁迭代链（见 ../patches/）与悬停标定 / 逐轮微调记录整理，
包含验收时的关键实现与全部最终补偿参数。板载运行环境中 ROS2 节点装配
（话题 / 服务名等）以实验板实际为准。

两种功能（-p mode:=right / -p mode:=color）：
  mode=right : 识别正前方任意颜色物块 -> 放置到机械臂右侧指定区（固定关节角）
  mode=color : 识别红/绿/蓝积木 -> 放置到左侧对应颜色色标区（启动自动标定）

调试模式（-p hover:=true）：
  悬停标定：检测到目标后仅悬停于其上方 6cm，不闭爪；保存截图、打印
  pose_t 与实际末端位姿 HOVER actual 后退出，用于量化坐标误差来源。
"""
import time
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from example.rgbd_function.include import common
from example.rgbd_function.include.track_and_grab import ColorTracker


class SortGrab(Node):
    """颜色分拣 / 定位夹取主节点（重建版）。"""

    def __init__(self):
        super().__init__('sort_grab')
        # ---- 参数 ----
        self.mode = self.get_parameter('mode').value if self.has_parameter('mode') else 'color'
        self.hover = self.get_parameter('hover').value if self.has_parameter('hover') else False
        self.enable_disp = self.get_parameter('enable_disp').value if self.has_parameter('enable_disp') else False
        start = self.get_parameter('start').value if self.has_parameter('start') else True
        init_color = self.get_parameter('color').value if self.has_parameter('color') else 'red'
        # ---- 颜色轮询表 ----
        self.COLORS = ['red', 'green', 'blue']
        self.lab_data = {
            'red':   ((0, 150, 150), (255, 190, 255)),    # 实测阈值区间（L,a,b）
            'green': ((0, 100, 100), (255, 140, 220)),
            'blue':  ((0, 100, 100), (255, 190, 150)),
        }
        self.color_idx = self.COLORS.index(init_color)
        self.target_color = self.COLORS[self.color_idx]
        self.tracker = ColorTracker(self.target_color)
        # ---- 中央区 / 深度门槛（最终参数，见 CHANGELOG） ----
        self.start_stamp = time.time()
        self.start = start
        self.enable_disp = self.enable_disp
        # ---- 状态 ----
        self.place_points = {}          # 色标放置点: {color: (x, y, z)}
        self.calibrated = False
        self.out_count = 0
        self.skip_count = 0
        self._skip_log = 0
        self.last_seen = 0.0
        self.last_skip = 0.0
        self.last_pitch_yaw = [0.0, 0.0]
        self.centers = []
        self.stamp = time.time()
        self.moving = False
        self.running = True
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_info = None
        # ---- ROS2 接口（话题/服务名以实验板为准） ----
        self.joints_pub = self.create_publisher(Int32MultiArray, 'joints_pub', 10)
        self.set_pose_target_client = self.create_client(
            common.SetPoseTarget, 'set_pose_target')
        self._sub_rgb = self.create_subscription(common.Image, 'rgb_image',
                                                 self.cb_rgb, 10)
        self._sub_depth = self.create_subscription(common.Image, 'depth_image',
                                                   self.cb_depth, 10)
        self._sub_info = self.create_subscription(common.CameraInfo, 'depth_camera_info',
                                                  self.cb_info, 10)
        # 手眼标定矩阵（相机在机械臂末端的安装位姿，官方标定）
        self.hand2cam_tf_matrix = np.array([
            [0, 0, 1, -0.101],
            [-1, 0, 0, 0.011],
            [0, -1, 0, 0.045],
            [0, 0, 0, 1]], dtype=np.float64)
        # ---- 逐色补偿参数（悬停标定 + 逐轮实测微调最终值，单位:米） ----
        # 坐标语义：x 正=远离机械臂方向，y 正=机械臂左侧，z 正=升高
        self.GRAB_OFFSET = {
            'red':   (0.050, -0.004, 0.020),
            'blue':  (0.040, -0.005, 0.030),
            'green': (0.000, -0.015, 0.020),
        }
        # 放置偏移：叠加在色标点之上（远+ / 左+）
        self.PLACE_OFFSET = {
            'red':   (0.015, -0.035),     # 远+1.5cm, 左+3.5cm（实测）
            'blue':  (0.015,  0.000),
            'green': (0.015,  0.000),
        }
        if start:
            # 启动时自动扫描左侧色标板，标定三种颜色的放置点
            threading.Thread(target=self.scan_place_points, daemon=True).start()

    # ---------- 订阅回调 ----------
    def cb_rgb(self, msg):
        self.latest_rgb = msg

    def cb_depth(self, msg):
        self.latest_depth = msg
        self.latest_info = getattr(self, '_info', None)

    def cb_info(self, msg):
        self._info = msg
        self.latest_info = msg

    # ---------- 色标板放置点自动标定 ----------
    def scan_place_points(self):
        """启动时扫描左侧色标板：连续 10 帧取中位数作为放置点世界坐标。"""
        while self.running and not self.calibrated:
            if self.latest_rgb is None or self.latest_depth is None:
                time.sleep(0.05)
                continue
            rgb = np.ndarray(shape=(self.latest_rgb.height, self.latest_rgb.width, 3),
                             dtype=np.uint8, buffer=self.latest_rgb.data)
            depth = np.ndarray(shape=(self.latest_depth.height, self.latest_depth.width),
                               dtype=np.uint16, buffer=self.latest_depth.data)
            h, w = depth.shape[:2]
            for color in self.COLORS:
                mask = cv2.inRange(cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB),
                                   self.lab_data[color][0], self.lab_data[color][1])
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                best = None
                for c in contours:
                    (cx, cy), r = cv2.minEnclosingCircle(c)
                    if (cy * 2) < 0.35 * h:            # 色标板位于画面下方
                        if best is None or abs(cv2.contourArea(c)) > best[2]:
                            best = (int(cx), int(cy), abs(cv2.contourArea(c)))
                if best is None:
                    continue
                x1 = max(0, best[0] - 3); x2 = min(w, best[0] + 3)
                y1 = max(0, best[1] - 3); y2 = min(h, best[1] + 3)
                roi = depth[y1:y2, x1:x2]
                valid = roi[np.logical_and(roi > 0, roi < 10000)]
                if valid.size == 0:
                    continue
                dist = float(np.median(valid)) / 1000.0
                K = self.latest_info.k
                pos = common.depth_pixel_to_camera(
                    (best[0], best[1]), dist, (K[0], K[4], K[2], K[5]))
                pose_end = np.matmul(self.hand2cam_tf_matrix,
                                     common.xyz_euler_to_mat(pos, (0, 0, 0)))
                world = np.matmul(self.get_endpoint(), pose_end)
                pose_t, _ = common.mat_to_xyz_euler(world)
                if color not in self.place_points:
                    self.place_points[color] = []
                self.place_points[color].append(
                    (pose_t[0], pose_t[1], pose_t[2]))
            # 每色取中位数
            ready = all(len(self.place_points.get(c, [])) >= 10 for c in self.COLORS)
            if ready:
                for color in self.COLORS:
                    pts = np.array(self.place_points[color])
                    self.place_points[color] = np.median(pts, axis=0)
                self.calibrated = True
                self.get_logger().info('色标板标定完成: %s' %
                                       {c: [round(v, 3) for v in p]
                                        for c, p in self.place_points.items()})
            time.sleep(0.05)

    # ---------- 末端位姿 / 逆解 ----------
    def get_endpoint(self):
        """获取机械臂末端在基坐标系下的位姿（正运动学）。

        实验板：通过正运动学服务获得并缓存在 self.endpoint，此处占位。
        """
        self.endpoint = np.eye(4)
        return self.endpoint

    def send_request(self, client, msg):
        if not client.service_is_ready():
            return None
        return client.call(msg)

    # ---------- 抓取与放置 ----------
    def pick(self, position):
        """完整抓放动作：高位到位->下降->闭爪->抬升->回中->放置。"""
        yaw = 80 if position[2] < 0.2 else 30
        msg = common.set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res and res.pulse:
            servo_data = res.pulse
            # 1) 高位到达目标上方
            set_servo_position(self.joints_pub, 1, ((1, servo_data[0]),))
            time.sleep(1)
            # 2) 下降到位
            set_servo_position(self.joints_pub, 1.5,
                               ((1, servo_data[0]), (2, servo_data[1]),
                                (3, servo_data[2]), (4, servo_data[3]),
                                (5, servo_data[4])))
            time.sleep(1.5)
            # 3) 闭合夹爪（10号=600）
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
            # 5) 回中
            set_servo_position(self.joints_pub, 1,
                               ((1, 500), (2, 720), (3, 100), (4, 120),
                                (5, 500), (10, 600)))
            time.sleep(1)
            # 6) 放置
            self.place(position)
        else:
            self.get_logger().info('\033[1;31m%s\033[0m' % "Out of gripping range")
        self.moving = False

    def place(self, position):
        """放置：mode=right 固定关节角放右侧；mode=color 放对应色标区。"""
        if self.mode == 'right':
            # 功能1: 统一放机械臂右侧指定区（固定关节角序列）
            set_servo_position(self.joints_pub, 1,
                               ((1, 125), (2, 635), (3, 120), (4, 200), (5, 500)))
            time.sleep(1)
            set_servo_position(self.joints_pub, 1.5,
                               ((1, 125), (2, 550), (3, 120), (4, 150), (5, 500)))
            time.sleep(1.5)
            set_servo_position(self.joints_pub, 1,
                               ((1, 125), (2, 550), (3, 120), (4, 170),
                                (5, 500), (10, 200)))
            time.sleep(1.5)
            self.get_logger().info('放置到[右侧]指定区')
        else:
            place = self.place_points.get(self.target_color)
            if place is not None:
                p = np.array(place, dtype=np.float64).copy()
                dx, dy = self.PLACE_OFFSET.get(self.target_color, (0.015, 0.0))
                p[0] += dx
                p[1] += dy
                self.get_logger().info('放置到[%s]色标' % self.target_color)
                for dz in (0.12, 0.03):
                    pp = p.copy()
                    pp[2] += dz
                    msg = common.set_pose_target(pp, 30, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.set_pose_target_client, msg)
                    if res and res.pulse:
                        sd = res.pulse
                        set_servo_position(self.joints_pub, 1.5,
                                           ((1, sd[0]), (2, sd[1]), (3, sd[2]),
                                            (4, sd[3]), (5, sd[4])))
                        time.sleep(1.5)
                pp = p.copy()
                pp[2] += 0.02
                msg = common.set_pose_target(pp, 30, [-180.0, 180.0], 1.0)
                res = self.send_request(self.set_pose_target_client, msg)
                if res and res.pulse:
                    sd = res.pulse
                    set_servo_position(self.joints_pub, 1.5,
                                       ((1, sd[0]), (2, sd[1]), (3, sd[2]),
                                        (4, sd[3]), (5, sd[4])))
                    time.sleep(1.5)
                set_servo_position(self.joints_pub, 0.5, ((10, 200),))
                time.sleep(1)
            else:
                self.get_logger().warn('无[%s]色标坐标，不放' % self.target_color)
        # 放置后回到基准
        set_servo_position(self.joints_pub, 1,
                           ((1, 500), (2, 720), (3, 100), (4, 150),
                            (5, 500), (10, 200)))
        time.sleep(2)
        self.tracker.yaw = 500
        self.tracker.pitch = 150
        self.tracker.pid_yaw.clear()
        self.tracker.pid_pitch.clear()
        self.stamp = time.time()

    # ---------- 主循环 ----------
    def main(self):
        while self.running:
            if self.latest_rgb is None or self.latest_depth is None or self.latest_info is None:
                time.sleep(0.02)
                continue
            ros_rgb_image = self.latest_rgb
            ros_depth_image = self.latest_depth
            depth_camera_info = self.latest_info
            self.latest_depth = None
            try:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3),
                                       dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width),
                                         dtype=np.uint16, buffer=ros_depth_image.data)
                result_image = np.copy(rgb_image)
                h, w = depth_image.shape[:2]
                depth = np.copy(depth_image).reshape((-1,))
                depth[depth <= 0] = 55555
                sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
                sim_depth_image = sim_depth_image / 2000.0 * 255.0
                bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8),
                                                    cv2.COLORMAP_JET)
                if self.tracker is not None and not self.moving and \
                        time.time() > self.start_stamp and self.start:
                    result_image, p_y, center, r = self.tracker.proc(
                        rgb_image, result_image, self.lab_data)
                    if p_y is not None:
                        center_x, center_y = center
                        center_x = min(center_x, w)
                        center_y = min(center_y, h)
                        # 中央区约束：只处理正前方积木区（最终参数）
                        # 注: 深度图实际约 640x400, 早期 0.95h 上界会把 y=389~400
                        #     全部误拦, 最终只保留 y 下界
                        if not (0.25 * w < center_x < 0.95 * w and 0.40 * h < center_y):
                            self.skip_count += 1
                            self._skip_log += 1
                            if self._skip_log % 20 == 0:
                                self.get_logger().info(
                                    '跳过[%s]: center=(%d,%d) 非中央区' %
                                    (self.target_color, int(center_x), int(center_y)))
                            if self.skip_count >= 40:
                                self.skip_count = 0
                                self.color_idx = (self.color_idx + 1) % len(self.COLORS)
                                self.target_color = self.COLORS[self.color_idx]
                                self.tracker = ColorTracker(self.target_color)
                                self.get_logger().info('持续跳过40帧，切换: %s' %
                                                       self.target_color)
                            self.stamp = time.time()
                            continue
                        self.last_seen = time.time()
                        # 伺服深度门槛：只跟踪 0.08~0.35m（排除背景/人/远处干扰）
                        dx1 = max(0, int(center_x - 5)); dx2 = min(w, int(center_x + 5))
                        dy1 = max(0, int(center_y - 5)); dy2 = min(h, int(center_y + 5))
                        droi = depth_image[dy1:dy2, dx1:dx2]
                        dvalid = droi[np.logical_and(droi > 0, droi < 10000)]
                        if dvalid.size == 0 or not (
                                0.08 < float(np.median(dvalid) / 1000.0) < 0.35):
                            self.skip_count += 1
                            self._skip_log += 1
                            if self._skip_log % 20 == 0:
                                self.get_logger().info(
                                    '深度异常[%s]: center=(%d,%d) 忽略' %
                                    (self.target_color, int(center_x), int(center_y)))
                            if self.skip_count >= 40:
                                self.skip_count = 0
                                self.color_idx = (self.color_idx + 1) % len(self.COLORS)
                                self.target_color = self.COLORS[self.color_idx]
                                self.tracker = ColorTracker(self.target_color)
                                self.get_logger().info('持续跳过40帧，切换: %s' %
                                                       self.target_color)
                            self.stamp = time.time()
                            continue
                        pitch_v = min(max(int(p_y[0]), 0), 250)
                        set_servo_position(self.joints_pub, 0.02,
                                           ((1, int(p_y[1])), (4, pitch_v)))
                        # 稳定判定：相邻帧偏差<2 且连续 5 帧取均值
                        if abs(self.last_pitch_yaw[0] - p_y[0]) < 2 and \
                                abs(self.last_pitch_yaw[1] - p_y[1]) < 2:
                            self.centers.append((center_x, center_y))
                            if len(self.centers) > 5:
                                self.centers.pop(0)
                            center_x = int(sum(c[0] for c in self.centers) / len(self.centers))
                            center_y = int(sum(c[1] for c in self.centers) / len(self.centers))
                            x1 = max(0, int(center_x - 3)); x2 = min(w, int(center_x + 3))
                            y1 = max(0, int(center_y - 3)); y2 = min(h, int(center_y + 3))
                            roi = depth_image[y1:y2, x1:x2]
                            valid_depths = roi[np.logical_and(roi > 0, roi < 10000)]
                            if valid_depths.size == 0:
                                self.stamp = time.time()
                                continue
                            dist = round(float(np.mean(valid_depths) / 1000.0), 3)
                            self.skip_count = 0
                            self.get_logger().info('DISTANCE: ' + str(dist))
                            if dist > 0.35 or dist < 0.06:
                                if time.time() - self.last_skip > 1.0:
                                    self.last_skip = time.time()
                                    self.color_idx = (self.color_idx + 1) % len(self.COLORS)
                                    self.target_color = self.COLORS[self.color_idx]
                                    self.tracker = ColorTracker(self.target_color)
                                    self.get_logger().info(
                                        '距离超限(%s m)，切换到 %s' %
                                        (str(dist), self.target_color))
                                self.stamp = time.time()
                                continue
                            K = depth_camera_info.k
                            self.get_endpoint()
                            position = common.depth_pixel_to_camera(
                                (center_x, center_y), dist, (K[0], K[4], K[2], K[5]))
                            pose_end = np.matmul(
                                self.hand2cam_tf_matrix,
                                common.xyz_euler_to_mat(position, (0, 0, 0)))
                            world_pose = np.matmul(self.endpoint, pose_end)
                            pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                            # 逐色补偿（悬停标定 + 逐轮实测微调最终值）
                            ox, oy, oz = self.GRAB_OFFSET.get(
                                self.target_color, (0.0, 0.0, 0.0))
                            pose_t[0] += ox
                            pose_t[1] += oy
                            pose_t[2] += oz
                            self.stamp = time.time()
                            if self.hover:
                                # 悬停标定：只悬停于上方 6cm，不闭爪
                                yaw_h = 80 if pose_t[2] < 0.2 else 30
                                msg_h = common.set_pose_target(
                                    [pose_t[0], pose_t[1], pose_t[2] + 0.06],
                                    yaw_h, [-180.0, 180.0], 1.0)
                                res_h = self.send_request(self.set_pose_target_client, msg_h)
                                self.get_logger().info(
                                    'HOVER pose_t: %s' % [round(v, 3) for v in pose_t])
                                if res_h and res_h.pulse:
                                    sd = res_h.pulse
                                    set_servo_position(self.joints_pub, 1, ((1, sd[0]),))
                                    time.sleep(1)
                                    set_servo_position(
                                        self.joints_pub, 1.5,
                                        ((1, sd[0]), (2, sd[1]), (3, sd[2]),
                                         (4, sd[3]), (5, sd[4])))
                                    time.sleep(2.5)
                                    if self.latest_rgb is not None:
                                        img_h = np.ndarray(
                                            shape=(self.latest_rgb.height,
                                                   self.latest_rgb.width, 3),
                                            dtype=np.uint8,
                                            buffer=self.latest_rgb.data)
                                        cv2.imwrite('/tmp/hover.jpg',
                                                    cv2.cvtColor(img_h, cv2.COLOR_RGB2BGR))
                                        self.get_logger().info('HOVER 截图已存 /tmp/hover.jpg')
                                    ep_h = self.get_endpoint()
                                    self.get_logger().info(
                                        'HOVER actual: %s' % [round(ep_h[0, 3], 3),
                                                              round(ep_h[1, 3], 3),
                                                              round(ep_h[2, 3], 3)])
                                self.running = False
                                self.moving = False
                                continue
                            self.moving = True
                            threading.Thread(target=self.pick, args=(pose_t,)).start()
                        else:
                            if time.time() - getattr(self, '_lst', 0) > 1.0:
                                self._lst = time.time()
                                self.get_logger().info(
                                    '未稳定[%s]: dp=(%.1f,%.1f)' %
                                    (self.target_color,
                                     self.last_pitch_yaw[0] - p_y[0],
                                     self.last_pitch_yaw[1] - p_y[1]))
                            self.stamp = time.time()
                        # 显示层
                        dist = depth_image[
                            min(int(center_y), depth_image.shape[0] - 1),
                            min(int(center_x), depth_image.shape[1] - 1)]
                        if dist < 150:
                            txt = "TOO CLOSE !!!"
                        else:
                            txt = "Dist: {}mm".format(dist)
                        cv2.circle(result_image, (int(center_x), int(center_y)), 5,
                                   (255, 255, 255), -1)
                        cv2.circle(depth_color_map, (int(center_x), int(center_y)), 5,
                                   (255, 255, 255), -1)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20),
                                    cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20),
                                    cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2,
                                    cv2.LINE_AA)
                        self.last_pitch_yaw = p_y
                    else:
                        self.stamp = time.time()
                        if time.time() - self.last_seen > 3:
                            self.color_idx = (self.color_idx + 1) % len(self.COLORS)
                            self.target_color = self.COLORS[self.color_idx]
                            self.tracker = ColorTracker(self.target_color)
                            self.last_seen = time.time()
                            self.get_logger().info('无目标，切换: %s' % self.target_color)
                if self.enable_disp:
                    result_image = cv2.cvtColor(result_image[40:440, ], cv2.COLOR_RGB2BGR)
                    result_image = np.concatenate([result_image, depth_color_map], axis=1)
                    cv2.imshow("sort_grab", result_image)
                    key = cv2.waitKey(1)
                    if key == ord('q') or key == 27:
                        self.running = False
            except Exception as e:
                self.get_logger().info('error1: ' + str(e))


def set_servo_position(pub, duration, data):
    """发布舵机位置消息（1=舵机号, value=脉宽）。"""
    msg = Int32MultiArray()
    msg.data = [int(duration * 1000)] + [v for pair in data for v in pair]
    pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SortGrab()
    try:
        node.main()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
