# -*- coding: utf-8 -*-
# sort_grab.py 补丁10：悬停标定修正 (2026-09-24 实测)
# 夹爪偏远5~6cm -> x 净补偿 +0.025 改 -0.030
# 夹爪偏左2~3cm(画面右=机械臂左) -> 新增 y 补偿 -0.025
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

old = """                            pose_t[0] += 0.025  # 2026-09-23 悬停标定: TCP->夹爪中心 x 偏移 2.2~2.5cm
                            pose_t[2] += 0.010"""

new = """                            pose_t[0] += -0.030  # 2026-09-24 悬停标定: 夹爪偏远5~6cm->靠近(x净-0.030)
                            pose_t[1] += -0.025  # 2026-09-24 悬停标定: 夹爪偏左2~3cm->向右(y-0.025)
                            pose_t[2] += 0.010"""

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
