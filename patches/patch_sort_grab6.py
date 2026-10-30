# -*- coding: utf-8 -*-
# sort_grab.py 补丁6：回退 4号 25 -> 60（恢复手眼标定匹配；积木摆25~35cm最佳距离）
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    n = s.count('(4, 25)')
    assert n >= 6, 'expect >=6, got %d: %s' % (n, p)
    s = s.replace('(4, 25)', '(4, 60)')
    io.open(p, 'w', encoding='utf-8').write(s)
    print('REVERTED %d occurrences:' % n, p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
