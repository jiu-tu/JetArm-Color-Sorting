# -*- coding: utf-8 -*-
# sort_grab.py 双份同步补丁：
# 1) 中央区约束去掉 y 下界 -> 绿色(center_y≈76)放行
# 2) 新增 mode:=right 功能1: 统一放机械臂右侧指定区(固定关节角)；默认 color 功能2: 放对应色标
import io, sys

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

old_constr = "if not (0.25 * w < center_x < 0.75 * w and center_y > 0.2 * h):"
new_constr = "if not (0.25 * w < center_x < 0.75 * w):"

old_init = "        self.calibrated = False"
new_init = ("        self.calibrated = False\n"
            "        self.mode = self.get_parameter('mode').value if self.has_parameter('mode') else 'color'")

old_place = """        place = self.place_points.get(self.target_color)
        if place is not None:
            place = place.copy()
            place[0] += 0.015
            self.get_logger().info('放置到[%s]色标' % self.target_color)
            for dz in (0.12, 0.03):
                p = place.copy()
                p[2] += dz
                msg = set_pose_target(p, yaw, [-180.0, 180.0], 1.0)
                res = self.send_request(self.set_pose_target_client, msg)
                if res.pulse:
                    sd = res.pulse
                    set_servo_position(self.joints_pub, 1.5, ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3]), (5, sd[4])))
                    time.sleep(1.5)
            p = place.copy()
            p[2] += 0.02
            msg = set_pose_target(p, yaw, [-180.0, 180.0], 1.0)
            res = self.send_request(self.set_pose_target_client, msg)
            if res.pulse:
                sd = res.pulse
                set_servo_position(self.joints_pub, 1.5, ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3]), (5, sd[4])))
                time.sleep(1.5)
            set_servo_position(self.joints_pub, 0.5, ((10, 200), ))
            time.sleep(1)
        else:
            self.get_logger().warn('无[%s]色标坐标，不放' % self.target_color)"""

new_place = """        if self.mode == 'right':
            # 功能1: 识别正前方物块后统一放机械臂右侧指定区（固定关节角序列）
            set_servo_position(self.joints_pub, 1, ((1, 125), (2, 635), (3, 120), (4, 200), (5, 500)))
            time.sleep(1)
            set_servo_position(self.joints_pub, 1.5, ((1, 125), (2, 550), (3, 120), (4, 150), (5, 500)))
            time.sleep(1.5)
            set_servo_position(self.joints_pub, 1, ((1, 125), (2, 550), (3, 120), (4, 170), (5, 500), (10, 200)))
            time.sleep(1.5)
            self.get_logger().info('放置到[右侧]指定区')
        else:
            place = self.place_points.get(self.target_color)
            if place is not None:
                place = place.copy()
                place[0] += 0.015
                self.get_logger().info('放置到[%s]色标' % self.target_color)
                for dz in (0.12, 0.03):
                    p = place.copy()
                    p[2] += dz
                    msg = set_pose_target(p, yaw, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.set_pose_target_client, msg)
                    if res.pulse:
                        sd = res.pulse
                        set_servo_position(self.joints_pub, 1.5, ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3]), (5, sd[4])))
                        time.sleep(1.5)
                p = place.copy()
                p[2] += 0.02
                msg = set_pose_target(p, yaw, [-180.0, 180.0], 1.0)
                res = self.send_request(self.set_pose_target_client, msg)
                if res.pulse:
                    sd = res.pulse
                    set_servo_position(self.joints_pub, 1.5, ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3]), (5, sd[4])))
                    time.sleep(1.5)
                set_servo_position(self.joints_pub, 0.5, ((10, 200), ))
                time.sleep(1)
            else:
                self.get_logger().warn('无[%s]色标坐标，不放' % self.target_color)"""

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    assert old_constr in s, 'constr not found: ' + p
    assert old_init in s, 'init not found: ' + p
    assert old_place in s, 'place not found: ' + p
    s = s.replace(old_constr, new_constr)
    s = s.replace(old_init, new_init)
    s = s.replace(old_place, new_place)
    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED OK:', p)

# 语法校验
import py_compile
py_compile.compile(paths[0], doraise=True)
print('SYNTAX OK')
