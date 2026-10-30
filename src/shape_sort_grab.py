#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
shape_sort_grab.py — JetArm 三维形状分拣（圆柱 / 长方体）
=============================================================
任务：
    正前方桌面上立着两个物块：一个圆柱、一个长方体。
    用 Gemini 深度相机识别形状，分别抓取：
        圆柱   -> 放到左边
        长方体 -> 放到右边

视觉思路（不依赖颜色，靠深度差分割立起的物块）：
    1) 在正前方 ROI 内取深度图，中位数作为"桌面深度"。
    2) 物块顶面比桌面近 BLOCK_NEAR_MM 以上 -> 二值 mask。
    3) mask 上找轮廓，按圆度 circularity = 4π·A / P² 分类：
           circularity > CIRC_THRESH  -> 圆柱
           否则                       -> 长方体
    4) 轮廓质心 + 该点深度 -> common.pixels_to_world -> 机械臂 xyz。

部署（在 Orange Pi / A200I 板上）：
    scp shape_sort_grab.py root@<板子IP>:/root/ros2_ws/
    cd /root/ros2_ws
    source install/setup.bash
    ros2 run example shape_sort_grab_node     # 或直接 python3 shape_sort_grab.py

首次运行必须先标定两个放置点（见下方 LEFT_PLACE / RIGHT_PLACE 注释）。
"""

import os
import sys
import time
import queue
import threading
import argparse

import cv2
import numpy as np

# 安全开关：python3 shape_sort_grab.py --dry-run  -> 只识别不动机械臂
DRY_RUN = "--dry-run" in sys.argv

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from servo_controller_msgs.msg import ServosPosition

# —— 以下三个 import 与板上官方例程保持一致（sdk / kinematics / servo_controller）——
from sdk.common import common
from kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position


# ============================================================
# 可调参数（按你的物块和桌面微调）
# ============================================================

# --- 视觉 ROI：正前方桌面区域（像素坐标，相对 RGB 图）---
# 只在这个窗口里找物块，排除色标板 / 手臂自身 / 背景。
ROI_X1, ROI_X2 = 120, 520          # 左右裁剪
ROI_Y1, ROI_Y2 = 100, 360          # 上下裁剪

# --- 深度分割（mm）---
BLOCK_NEAR_MM   = 18.0    # 物块顶面比桌面近多少 mm 才算"立起"（积木高约 30~60mm，取 18 留余量）
DEPTH_MIN_MM    = 80.0    # 比这个更近的点直接忽略（手臂/桌面边缘噪点）
DEPTH_MAX_MM    = 600.0   # 比这个更远的点直接忽略（背景）

# --- 轮廓面积过滤（像素²）---
MIN_AREA = 300
MAX_AREA = 8000

# --- 形状分类阈值 ---
# circularity: 正圆≈1.0，正方形≈0.785，长方形≈0.6~0.7，越扁越小
CIRC_THRESH = 0.78

# --- 抓取高度补偿（m）---
PICK_Z_ABOVE = 0.03       # 预抓取在物体上方 3cm
PICK_Z_DOWN  = 0.03       # 再下降 3cm 到物块顶面

PICK_PITCH   = -10.0      # 抓取时末端俯仰角（与官方例程一致，按实际夹爪角度微调）

# --- 左右放置点（m, 机械臂基坐标系）---
# !!! 首次使用必须标定 !!!
# 标定方法：手动让机械臂夹着物块到目标左/右位置，保持夹爪闭合，
#           调用 set_pose_target(pose, PICK_PITCH, [-180,180], 1.0) 看返回的 res.pulse 反推，
#           或用官方 "pixel_coordinate_calculation" 节点点一下桌面取坐标。
# 下面是经验默认值（JetArm 工作区：前方 ~0.15m，左右 ±0.12m，桌面 z≈0.02m）。
# 如果发现左右反了，把两个 y 坐标符号对调即可。
LEFT_PLACE_POSE  = [0.16,  0.13, 0.02]   # 圆柱 -> 左
RIGHT_PLACE_POSE = [0.16, -0.13, 0.02]   # 长方 -> 右

# --- 关节初始位（与官方 color_sorting 例程一致）---
HOME_POSE = ((1, 500), (2, 610), (3, 70), (4, 140), (5, 500), (10, 200))

# --- 配置文件路径（板上 root 用户，工作空间在 /root/ros2_ws）---
CONFIG_CANDIDATES = [
    "/root/ros2_ws/src/app/config/arm_config.yaml",
    "/root/ros2_ws/src/app/config/positions.yaml",
    "/home/ubuntu/ros2_ws/src/app/config/arm_config.yaml",
]


# ============================================================
# 工具函数
# ============================================================

def classify_shape(cnt):
    """返回 ('cylinder'|'cuboid', circularity)；面积不合规返回 (None, 0)"""
    area = cv2.contourArea(cnt)
    if area < MIN_AREA or area > MAX_AREA:
        return None, 0.0
    peri = cv2.arcLength(cnt, True)
    if peri < 1e-3:
        return None, 0.0
    circ = 4.0 * np.pi * area / (peri * peri)
    if circ > CIRC_THRESH:
        return "cylinder", circ
    return "cuboid", circ


# ============================================================
# 主节点
# ============================================================

class ShapeSortNode(Node):

    def __init__(self, name="shape_sort_grab"):
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.running = True
        self.cb_group = ReentrantCallbackGroup()

        # 图像队列
        self.rgb_q = queue.Queue(maxsize=2)
        self.depth_q = queue.Queue(maxsize=2)
        self.K = None                 # 相机内参 3x3

        # ---------- 加载外参 / 标定 ----------
        self._load_extrinsic()

        # ---------- 订阅 ----------
        self.create_subscription(
            Image, "/depth_cam/rgb/image_raw", self._rgb_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            Image, "/depth_cam/depth/image_raw", self._depth_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            CameraInfo, "/depth_cam/depth/camera_info", self._caminfo_cb, 10,
            callback_group=self.cb_group)

        # ---------- 发布 / 服务客户端 ----------
        self.joints_pub = self.create_publisher(
            ServosPosition, "/servo_controller/servo_position", 10,
            callback_group=self.cb_group)

        # 等运动学服务起来
        self.kinematics_client = self.create_client(
            Trigger, "/kinematics/set_pose_target",
            callback_group=self.cb_group) if False else None
        # 真正的 set_pose_target service 类型是 kinematics_msgs.srv.SetRobotPose
        from kinematics_msgs.srv import SetRobotPose
        self.SetRobotPose = SetRobotPose
        self.kinematics_client = self.create_client(
            SetRobotPose, "/kinematics/set_pose_target",
            callback_group=self.cb_group)

        self.get_logger().info("等待运动学服务...")
        self.kinematics_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("shape_sort_grab 节点就绪")

    # --------------------------------------------------------
    # 配置 / 外参
    # --------------------------------------------------------
    def _load_extrinsic(self):
        """从 YAML 读外参 extristric 与 white_area_pose_world，构造投影矩阵"""
        cfg_path = None
        for p in CONFIG_CANDIDATES:
            if os.path.exists(p):
                cfg_path = p
                break
        if cfg_path is None:
            self.get_logger().warn("未找到外参 YAML，用单位阵占位——必须标定！")
            self.extristric = (np.eye(3), np.zeros(3))
            self.white_area_center = np.eye(4)
            self.offset = (0.0, 0.0, 0.0)
            self.scale = (1.0, 1.0, 1.0)
            return

        cfg = common.get_yaml_data(cfg_path)
        ext = cfg.get("extristric", cfg.get("extrinsic"))
        if ext is not None:
            tvec, rmat = common.extristric_plane_shift(
                np.array(ext["translation"]), np.array(ext["rotation"]))
            self.extristric = (rmat, tvec)
        else:
            self.extristric = (np.eye(3), np.zeros(3))

        wa = cfg.get("white_area_pose_world", [0, 0, 0, 0, 0, 0])
        self.white_area_center = common.xyz_euler_to_mat(
            np.array(wa[:3], dtype=float), tuple(wa[3:6]))

        # offset / scale 标定（与官方 color_sorting 一致）
        sort_cfg = cfg.get("shape_sorting", cfg.get("color_sortting", {}))
        self.offset = tuple(sort_cfg.get("offset", (0.0, 0.0, 0.0)))
        self.scale = tuple(sort_cfg.get("scale", (1.0, 1.0, 1.0)))

        self.get_logger().info("外参加载自: " + cfg_path)

    # --------------------------------------------------------
    # ROS 回调
    # --------------------------------------------------------
    def _caminfo_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)

    def _rgb_cb(self, msg: Image):
        img = np.ndarray(shape=(msg.height, msg.width, 3),
                         dtype=np.uint8, buffer=msg.data)
        # 相机一般是 RGB，OpenCV 要 BGR
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if self.rgb_q.full():
            try: self.rgb_q.get_nowait()
            except queue.Empty: pass
        self.rgb_q.put(img)

    def _depth_cb(self, msg: Image):
        depth = np.ndarray(shape=(msg.height, msg.width),
                            dtype=np.uint16, buffer=msg.data)
        if self.depth_q.full():
            try: self.depth_q.get_nowait()
            except queue.Empty: pass
        self.depth_q.put(depth.copy())

    # --------------------------------------------------------
    # 视觉识别
    # --------------------------------------------------------
    def detect_shapes(self, rgb, depth):
        """
        返回 [{'shape','cx','cy','depth_mm','world_xyz'}, ...]
        只返回恰好一个 cylinder 和一个 cuboid（各取面积最大的）。
        """
        h, w = depth.shape
        x1, x2 = max(ROI_X1, 0), min(ROI_X2, w)
        y1, y2 = max(ROI_Y1, 0), min(ROI_Y2, h)

        roi_d = depth[y1:y2, x1:x2].astype(np.float32)
        roi_rgb = rgb[y1:y2, x1:x2]

        # 1) 桌面深度 = ROI 内有效深度中位数
        valid = roi_d[(roi_d > DEPTH_MIN_MM) & (roi_d < DEPTH_MAX_MM)]
        if valid.size < 50:
            return [], roi_rgb
        table_d = float(np.median(valid))

        # 2) 物块 mask：比桌面近 BLOCK_NEAR_MM 以上
        block_mask = ((roi_d > DEPTH_MIN_MM) &
                      (roi_d < table_d - BLOCK_NEAR_MM)).astype(np.uint8) * 255

        # 形态学去噪
        block_mask = cv2.morphologyEx(
            block_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        block_mask = cv2.morphologyEx(
            block_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # 3) 找轮廓
        contours, _ = cv2.findContours(
            block_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = {"cylinder": [], "cuboid": []}
        for cnt in contours:
            shape, circ = classify_shape(cnt)
            if shape is None:
                continue
            M = cv2.moments(cnt)
            if M["m00"] < 1e-3:
                continue
            cx = int(M["m10"] / M["m00"]) + x1   # 转回全图坐标
            cy = int(M["m01"] / M["m00"]) + y1

            # 该点深度（取 5x5 中位数抗噪）
            dx1, dx2 = max(0, cx-3), min(w, cx+3)
            dy1, dy2 = max(0, cy-3), min(h, cy+3)
            patch = depth[dy1:dy2, dx1:dx2].astype(np.float32)
            pv = patch[(patch > DEPTH_MIN_MM) & (patch < table_d - 5)]
            d_mm = float(np.median(pv)) if pv.size > 0 else table_d - BLOCK_NEAR_MM

            # 画框调试
            color = (0, 255, 0) if shape == "cylinder" else (255, 0, 0)
            cv2.drawContours(roi_rgb, [cnt - np.array([[[x1, y1]]])], -1, color, 2)
            cv2.putText(roi_rgb, "%s(%.2f)" % (shape, circ),
                        (cx - x1, cy - y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            candidates[shape].append({
                "shape": shape,
                "cx": cx, "cy": cy,
                "depth_mm": d_mm,
                "area": cv2.contourArea(cnt),
            })

        # 每类取面积最大的一个
        result = []
        for shape in ("cylinder", "cuboid"):
            if candidates[shape]:
                best = max(candidates[shape], key=lambda c: c["area"])
                result.append(best)

        return result, roi_rgb

    # --------------------------------------------------------
    # 像素 + 深度 -> 机械臂 xyz（与官方 color_sorting 一致）
    # --------------------------------------------------------
    def pixel_to_arm(self, cx, cy):
        proj = np.row_stack((
            np.column_stack((self.extristric[1], self.extristric[0])),
            np.array([[0, 0, 0, 1]])))
        world_pose = common.pixels_to_world(
            [(cx, cy)], self.K, proj)[0]
        world_pose[1] = -world_pose[1]
        world_pose[2] = 0.02
        world_pose = np.matmul(
            self.white_area_center,
            common.xyz_euler_to_mat(world_pose, (0, 0, 0)))
        world_pose[2] = 0.02
        pose_t, _ = common.mat_to_xyz_euler(world_pose)
        for i in range(3):
            pose_t[i] = pose_t[i] + self.offset[i]
            pose_t[i] = pose_t[i] * self.scale[i]
        return [float(pose_t[0]), float(pose_t[1]), float(pose_t[2])]

    # --------------------------------------------------------
    # 动作：抓取 / 放置
    # --------------------------------------------------------
    def _send_pose(self, pose, yaw=0.0):
        """笛卡尔位姿请求 -> 返回舵机脉冲列表 [s1,s2,s3,s4] 或 None"""
        msg = set_pose_target(pose, PICK_PITCH, [-180.0, 180.0], 1.0)
        res = self._call_kinematics(msg)
        if res is None or not res.pulse:
            return None
        return res.pulse

    def _call_kinematics(self, msg):
        import threading as _t
        ev = _t.Event()
        out = {}
        def _do():
            fut = self.kinematics_client.call_async(msg)
            fut.add_done_callback(lambda f: (out.update(r=f.result()), ev.set()))
        _t.Thread(target=_do, daemon=True).start()
        ev.wait(timeout=5.0)
        return out.get("r")

    def pick(self, pose_xyz, label=""):
        """抓一个物块。pose_xyz 是物块顶面中心在机械臂坐标系下的 xyz。"""
        self.get_logger().info("抓取 %s @ (%.3f, %.3f, %.3f)" %
                               (label, pose_xyz[0], pose_xyz[1], pose_xyz[2]))
        if DRY_RUN:
            self.get_logger().info("[DRY-RUN] 跳过抓取动作")
            time.sleep(0.5)
            return True
        pose = list(pose_xyz)

        # 1) 先到物块上方 PICK_Z_ABOVE
        pose[2] += PICK_Z_ABOVE
        sd = self._send_pose(pose)
        if sd is None:
            self.get_logger().warn("预抓取位不可达")
            return False
        set_servo_position(self.joints_pub, 1.0,
                           ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3])))
        time.sleep(1.0)

        # 2) 下降到物块顶面
        pose[2] -= PICK_Z_ABOVE
        sd = self._send_pose(pose)
        if sd is None:
            return False
        set_servo_position(self.joints_pub, 1.0,
                           ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3])))
        time.sleep(1.0)

        # 3) 夹爪闭合
        set_servo_position(self.joints_pub, 1.0, ((10, 600),))
        time.sleep(1.0)

        # 4) 抬起回 home 上方
        set_servo_position(self.joints_pub, 1.5, HOME_POSE[:5])
        time.sleep(1.5)
        return True

    def place(self, place_xyz, label=""):
        """把夹着的物块放到 place_xyz。"""
        self.get_logger().info("放置 %s @ (%.3f, %.3f, %.3f)" %
                               (label, place_xyz[0], place_xyz[1], place_xyz[2]))
        if DRY_RUN:
            self.get_logger().info("[DRY-RUN] 跳过放置动作")
            return
        pose = list(place_xyz)

        # 1) 先到放置点上方
        pose[2] += PICK_Z_ABOVE
        sd = self._send_pose(pose)
        if sd is None:
            self.get_logger().warn("放置上位不可达")
            return
        set_servo_position(self.joints_pub, 1.0,
                           ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3])))
        time.sleep(1.0)

        # 2) 下降
        pose[2] -= PICK_Z_ABOVE
        sd = self._send_pose(pose)
        if sd is None:
            return
        set_servo_position(self.joints_pub, 2.0,
                           ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3])))
        time.sleep(2.0)

        # 3) 松夹爪
        set_servo_position(self.joints_pub, 1.0, ((10, 200),))
        time.sleep(1.0)

        # 4) 抬起回家
        set_servo_position(self.joints_pub, 1.0, HOME_POSE)
        time.sleep(1.0)

    # --------------------------------------------------------
    # 主流程
    # --------------------------------------------------------
    def run(self):
        # 回家
        set_servo_position(self.joints_pub, 1.0, HOME_POSE)
        time.sleep(1.0)

        # 1) 等一帧稳定图像，识别两个物块
        self.get_logger().info("等待视觉稳定...")
        detected = []
        for _ in range(30):
            try:
                rgb = self.rgb_q.get(timeout=1.0)
                depth = self.depth_q.get(timeout=1.0)
            except queue.Empty:
                continue
            detected, dbg = self.detect_shapes(rgb, depth)
            cv2.imshow("shape_debug", dbg)
            cv2.waitKey(1)
            shapes = [d["shape"] for d in detected]
            if "cylinder" in shapes and "cuboid" in shapes:
                break
            time.sleep(0.1)

        cv2.destroyAllWindows()

        have = {d["shape"] for d in detected}
        self.get_logger().info("识别结果: %s" % str(have))
        if "cylinder" not in have or "cuboid" not in have:
            self.get_logger().error("未同时识别到圆柱和长方体，请调整摆放/阈值")
            return

        # 2) 逐个抓取放置：圆柱->左，长方->右
        plan = {
            "cylinder": ("圆柱", LEFT_PLACE_POSE),
            "cuboid":   ("长方体", RIGHT_PLACE_POSE),
        }
        for d in detected:
            shape = d["shape"]
            label, place_xyz = plan[shape]
            arm_xyz = self.pixel_to_arm(d["cx"], d["cy"])
            self.get_logger().info("%s 像素=(%d,%d) depth=%.0fmm -> 机械臂=(%.3f,%.3f,%.3f)"
                                   % (label, d["cx"], d["cy"], d["depth_mm"],
                                      arm_xyz[0], arm_xyz[1], arm_xyz[2]))
            if not self.pick(arm_xyz, label):
                continue
            self.place(place_xyz, label)

        self.get_logger().info("分拣完成")


def main():
    rclpy.init()
    node = ShapeSortNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
