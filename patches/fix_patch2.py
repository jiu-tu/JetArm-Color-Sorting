# -*- coding: utf-8 -*-
# 修复 patch_sort_grab2 的误伤 + 补上跳过段计数切色
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

# A) 修复 pick 里 out_count 重置处误伤（16空格版被拆行）
old_broken = """                self.out_count = 0
        self.skip_count = 0
        self._skip_log = 0
                self.color_idx = (self.color_idx + 1) % len(self.COLORS)"""
new_fixed = """                self.out_count = 0
                self.color_idx = (self.color_idx + 1) % len(self.COLORS)"""

# B) 跳过段补计数切色（当前为 0.3 约束 + 原样日志）
old_skip = """                        if not (0.3 * w < center_x < 0.7 * w):
                            self.get_logger().info('跳过[%s]: center=(%d,%d) 非中央区' % (self.target_color, int(center_x), int(center_y)))
                            self.stamp = time.time()
                            continue"""
new_skip = """                        if not (0.3 * w < center_x < 0.7 * w):
                            self.skip_count += 1
                            self._skip_log += 1
                            if self._skip_log % 20 == 0:
                                self.get_logger().info('跳过[%s]: center=(%d,%d) 非中央区' % (self.target_color, int(center_x), int(center_y)))
                            if self.skip_count >= 40:
                                self.skip_count = 0
                                self.color_idx = (self.color_idx + 1) % len(self.COLORS)
                                self.target_color = self.COLORS[self.color_idx]
                                self.tracker = ColorTracker(self.target_color)
                                self.get_logger().info('持续跳过40帧，切换: %s' % self.target_color)
                            self.stamp = time.time()
                            continue"""

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    if old_broken in s:
        s = s.replace(old_broken, new_fixed)
        print('FIXED broken out_count:', p)
    else:
        print('broken not found (ok if already fixed):', p)
    if old_skip in s:
        s = s.replace(old_skip, new_skip)
        print('PATCHED skip block:', p)
    else:
        print('skip block not found:', p)
    io.open(p, 'w', encoding='utf-8').write(s)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
