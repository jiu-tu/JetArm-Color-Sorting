# -*- coding: utf-8 -*-
# sort_grab.py 补丁5：深度上限 0.5 -> 0.35，排除远处背景误检(如色标板/背景绿)
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    old_gate = "if dvalid.size == 0 or not (0.08 < float(np.median(dvalid) / 1000.0) < 0.5):"
    new_gate = "if dvalid.size == 0 or not (0.08 < float(np.median(dvalid) / 1000.0) < 0.35):"
    assert old_gate in s, 'gate anchor not found: ' + p
    s = s.replace(old_gate, new_gate)

    old_dist = "if dist > 1.5 or dist < 0.06:"
    new_dist = "if dist > 0.35 or dist < 0.06:"
    assert old_dist in s, 'dist anchor not found: ' + p
    s = s.replace(old_dist, new_dist)

    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED:', p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
