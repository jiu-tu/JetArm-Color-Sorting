# -*- coding: utf-8 -*-
# sort_grab.py 补丁2：
# 1) 中央区收紧 x in [0.3w, 0.7w] -> 色标板(x≈150-160)彻底排除
# 2) 连续跳过40帧强制切色 -> 不耗在色标板上
# 3) main finally 的 destroy_node/rclpy.shutdown 加保护 -> 消除崩溃
# 4) 跳过日志限频(每20帧打一次) -> 减少刷屏
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

# 1) 约束
old_constr = "if not (0.3 * w < center_x < 0.7 * w):"      # 可能已是补丁1的0.25版
for cand in ("if not (0.25 * w < center_x < 0.75 * w):",
             "if not (0.3 * w < center_x < 0.7 * w):"):
    pass

new_constr = "if not (0.3 * w < center_x < 0.7 * w):"

# 2) init 加 skip_count
old_init = "        self.out_count = 0"
new_init = ("        self.out_count = 0\n"
            "        self.skip_count = 0\n"
            "        self._skip_log = 0")

# 3) 跳过处理：计数+限频+40帧切色
old_skip = """                        if not (0.25 * w < center_x < 0.75 * w):
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

# 4) 稳定后重置 skip_count（在 DISTANCE 计算处）
old_dist = """                            dist = round(float(np.median(valid_depths) / 1000.0), 3)  # 中位数抗背景污染
                            self.get_logger().info('DISTANCE: ' + str(dist))"""
new_dist = """                            dist = round(float(np.median(valid_depths) / 1000.0), 3)
                            self.skip_count = 0
                            self.get_logger().info('DISTANCE: ' + str(dist))"""

# 5) main finally 保护
old_fin = """    finally:
        node.destroy_node()
        rclpy.shutdown()"""
new_fin = """    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass"""

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    if old_constr in s:
        # 已经打过补丁2（防重复）
        print('ALREADY PATCHED:', p)
        continue
    assert old_init in s, 'init not found: ' + p
    assert old_skip in s, 'skip not found: ' + p
    assert old_dist in s, 'dist not found: ' + p
    assert old_fin in s, 'fin not found: ' + p
    s = s.replace('if not (0.25 * w < center_x < 0.75 * w):', new_constr)
    s = s.replace(old_init, new_init)
    s = s.replace(old_skip, new_skip)
    s = s.replace(old_dist, new_dist)
    s = s.replace(old_fin, new_fin)
    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED OK:', p)

import py_compile
py_compile.compile(paths[0], doraise=True)
print('SYNTAX OK')
