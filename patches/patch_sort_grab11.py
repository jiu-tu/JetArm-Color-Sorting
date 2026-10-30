# -*- coding: utf-8 -*-
# sort_grab.py 补丁11：用户实测(第三次悬停)机械臂偏近3cm -> x补偿 -0.030 改 0.000
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

old = """                            pose_t[0] += -0.030  # 2026-09-24 悬停标定: 夹爪偏远5~6cm->靠近(x净-0.030)
                            pose_t[1] += -0.025  # 2026-09-24 悬停标定: 夹爪偏左2~3cm->向右(y-0.025)
                            pose_t[2] += 0.010"""

new = """                            pose_t[0] += 0.000  # 2026-09-24 悬停标定: 用户实测偏近3cm->x补偿回0
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
