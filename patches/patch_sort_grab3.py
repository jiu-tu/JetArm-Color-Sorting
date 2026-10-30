# -*- coding: utf-8 -*-
# sort_grab.py 补丁3：伺服跟踪加深度门槛(0.15~0.5m)，排除背景/人干扰
# 未命中深度范围 -> 按跳过处理(计数, 40帧切色)，机械臂保持静止
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

old = """                        self.last_seen = time.time()
                        pitch_v = min(max(int(p_y[0]), 0), 250)
                        set_servo_position(self.joints_pub, 0.02, ((1, int(p_y[1])), (4, pitch_v)))"""

new = """                        self.last_seen = time.time()
                        # 深度门槛: 只跟踪 0.15~0.5m 的物块, 排除背景/人/远处干扰
                        dx1 = max(0, int(center_x - 5)); dx2 = min(w, int(center_x + 5))
                        dy1 = max(0, int(center_y - 5)); dy2 = min(h, int(center_y + 5))
                        droi = depth_image[dy1:dy2, dx1:dx2]
                        dvalid = droi[np.logical_and(droi > 0, droi < 10000)]
                        if dvalid.size == 0 or not (0.15 < float(np.median(dvalid) / 1000.0) < 0.5):
                            self.skip_count += 1
                            self._skip_log += 1
                            if self._skip_log % 20 == 0:
                                self.get_logger().info('深度异常[%s]: center=(%d,%d) 忽略' % (self.target_color, int(center_x), int(center_y)))
                            if self.skip_count >= 40:
                                self.skip_count = 0
                                self.color_idx = (self.color_idx + 1) % len(self.COLORS)
                                self.target_color = self.COLORS[self.color_idx]
                                self.tracker = ColorTracker(self.target_color)
                                self.get_logger().info('持续跳过40帧，切换: %s' % self.target_color)
                            self.stamp = time.time()
                            continue
                        pitch_v = min(max(int(p_y[0]), 0), 250)
                        set_servo_position(self.joints_pub, 0.02, ((1, int(p_y[1])), (4, pitch_v)))"""

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    if '深度门槛' in s:
        print('ALREADY PATCHED:', p)
        continue
    assert old in s, 'anchor not found: ' + p
    s = s.replace(old, new)
    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED:', p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
