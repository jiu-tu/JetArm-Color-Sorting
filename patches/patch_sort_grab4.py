# -*- coding: utf-8 -*-
# sort_grab.py 补丁4：摄像头基准角 4号 60 -> 25（适配积木摆在正前方10~20cm）
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    n = s.count('(4, 60)')
    assert n >= 6, 'expect >=6, got %d: %s' % (n, p)
    s = s.replace('(4, 60)', '(4, 25)')
    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED %d occurrences:' % n, p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
