# -*- coding: utf-8 -*-
# track_and_grab.py 补丁8：ColorTracker.proc 排除画面上方(y<0.35h)区域
# 色标板红块(128,62)/背景绿(319,161)在上方 -> 排除; 积木(305~399)在下方 -> 保留
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/track_and_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/track_and_grab.py',
]

old = """            (center_x, center_y), radius = cv2.minEnclosingCircle(c) # 最小外接圆
            if min_c is None or math.fabs(cv2.contourArea(c)) > min_c[2]:"""

new = """            (center_x, center_y), radius = cv2.minEnclosingCircle(c) # 最小外接圆
            if (center_y * 2) < 0.35 * h:  # 排除画面上方(色标板/背景), 只认积木区
                continue
            if min_c is None or math.fabs(cv2.contourArea(c)) > min_c[2]:"""

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    assert old in s, 'anchor not found: ' + p
    s = s.replace(old, new)
    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED:', p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
