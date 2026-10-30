# -*- coding: utf-8 -*-
# sort_grab.py 补丁7：中央区按实测积木位置放宽
# 积木(60°视角实测): red(395,305) green(498,339) blue(602,399)
# 色标板: red块(128,62)+左下(36,429); 背景绿(319,161)
# 新约束: x in [0.25w,0.95w]=[160,608], y in [0.45h,0.95h]=[216,456]
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    old = "if not (0.3 * w < center_x < 0.7 * w):"
    new = "if not (0.25 * w < center_x < 0.95 * w and 0.45 * h < center_y < 0.95 * h):"
    assert old in s, 'anchor not found: ' + p
    s = s.replace(old, new)
    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED:', p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
