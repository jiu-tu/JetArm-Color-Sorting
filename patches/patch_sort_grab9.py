# -*- coding: utf-8 -*-
# sort_grab.py 补丁9：hover 悬停标定模式
# -p hover:=true 时: 检测到积木后只移动到上方6cm悬停(不闭爪), 截图存/tmp/hover.jpg, 打印pose_t后退出
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

old1 = "        self.mode = self.get_parameter('mode').value if self.has_parameter('mode') else 'color'"
new1 = "        self.mode = self.get_parameter('mode').value if self.has_parameter('mode') else 'color'\n        self.hover = self.get_parameter('hover').value if self.has_parameter('hover') else False"

old2 = """                            pose_t[0] += 0.025  # 2026-09-23 悬停标定: TCP->夹爪中心 x 偏移 2.2~2.5cm
                            pose_t[2] += 0.010
                            self.stamp = time.time()
                            self.moving = True
                            threading.Thread(target=self.pick, args=(pose_t,)).start()"""

new2 = """                            pose_t[0] += 0.025  # 2026-09-23 悬停标定: TCP->夹爪中心 x 偏移 2.2~2.5cm
                            pose_t[2] += 0.010
                            self.stamp = time.time()
                            if self.hover:
                                yaw_h = 80 if pose_t[2] < 0.2 else 30
                                msg_h = set_pose_target([pose_t[0], pose_t[1], pose_t[2] + 0.06], yaw_h, [-180.0, 180.0], 1.0)
                                res_h = self.send_request(self.set_pose_target_client, msg_h)
                                self.get_logger().info('HOVER pose_t: %s' % [round(v, 3) for v in pose_t])
                                if res_h.pulse:
                                    sd = res_h.pulse
                                    set_servo_position(self.joints_pub, 1, ((1, sd[0]), ))
                                    time.sleep(1)
                                    set_servo_position(self.joints_pub, 1.5, ((1, sd[0]), (2, sd[1]), (3, sd[2]), (4, sd[3]), (5, sd[4])))
                                    time.sleep(2.5)
                                    if self.latest_rgb is not None:
                                        img_h = np.ndarray(shape=(self.latest_rgb.height, self.latest_rgb.width, 3), dtype=np.uint8, buffer=self.latest_rgb.data)
                                        cv2.imwrite('/tmp/hover.jpg', cv2.cvtColor(img_h, cv2.COLOR_RGB2BGR))
                                        self.get_logger().info('HOVER 截图已存 /tmp/hover.jpg')
                                self.running = False
                                self.moving = False
                                continue
                            self.moving = True
                            threading.Thread(target=self.pick, args=(pose_t,)).start()"""

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    if 'self.hover =' in s:
        print('ALREADY PATCHED:', p)
        continue
    assert old1 in s, 'anchor1 not found: ' + p
    assert old2 in s, 'anchor2 not found: ' + p
    s = s.replace(old1, new1)
    s = s.replace(old2, new2)
    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED:', p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
