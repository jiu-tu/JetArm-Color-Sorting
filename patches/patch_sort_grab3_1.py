# -*- coding: utf-8 -*-
# sort_grab.py 补丁3.1：适配积木摆放在 10~20cm
# 1) 伺服深度门槛下限 0.15 -> 0.08
# 2) 主循环距离超限下限 0.1 -> 0.06
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    old_gate = "if dvalid.size == 0 or not (0.15 < float(np.median(dvalid) / 1000.0) < 0.5):"
    new_gate = "if dvalid.size == 0 or not (0.08 < float(np.median(dvalid) / 1000.0) < 0.5):"
    assert old_gate in s, 'gate anchor not found: ' + p
    s = s.replace(old_gate, new_gate)

    old_dist = "if dist > 1.5 or dist < 0.1:"
    new_dist = "if dist > 1.5 or dist < 0.06:"
    assert old_dist in s, 'dist anchor not found: ' + p
    s = s.replace(old_dist, new_dist)

    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED:', p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
