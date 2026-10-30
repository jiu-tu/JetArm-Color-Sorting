# -*- coding: utf-8 -*-
# sort_grab.py 补丁12：第四次悬停实测(偏近3cm偏右2cm) + z反投影为负(抓深根源)
# x: 0.000->+0.030(往远3cm); y: -0.025->-0.045(往左2cm); z: +0.010->+0.050(抬高4cm)
# hover分支追加: 打印实际末端位姿, 验证 pose_t 是否真实执行
import io

paths = [
    '/root/ros2_ws/install/example/lib/python3.10/site-packages/example/rgbd_function/include/sort_grab.py',
    '/root/ros2_ws/src/example/example/rgbd_function/include/sort_grab.py',
]

old1 = """                            pose_t[0] += 0.000  # 2026-09-24 悬停标定: 用户实测偏近3cm->x补偿回0
                            pose_t[1] += -0.025  # 2026-09-24 悬停标定: 夹爪偏左2~3cm->向右(y-0.025)
                            pose_t[2] += 0.010"""

new1 = """                            pose_t[0] += 0.030  # 2026-09-24 悬停标定: 偏近3cm->往远(x+0.030)
                            pose_t[1] += -0.045  # 2026-09-24 悬停标定: 偏右2cm->往左(y-0.045)
                            pose_t[2] += 0.050  # 2026-09-24 悬停标定: z反投影为负(抓深根源)->抬高5cm"""

old2 = """                                    if self.latest_rgb is not None:
                                        img_h = np.ndarray(shape=(self.latest_rgb.height, self.latest_rgb.width, 3), dtype=np.uint8, buffer=self.latest_rgb.data)
                                        cv2.imwrite('/tmp/hover.jpg', cv2.cvtColor(img_h, cv2.COLOR_RGB2BGR))
                                        self.get_logger().info('HOVER 截图已存 /tmp/hover.jpg')"""

new2 = """                                    if self.latest_rgb is not None:
                                        img_h = np.ndarray(shape=(self.latest_rgb.height, self.latest_rgb.width, 3), dtype=np.uint8, buffer=self.latest_rgb.data)
                                        cv2.imwrite('/tmp/hover.jpg', cv2.cvtColor(img_h, cv2.COLOR_RGB2BGR))
                                        self.get_logger().info('HOVER 截图已存 /tmp/hover.jpg')
                                    ep_h = self.get_endpoint()
                                    self.get_logger().info('HOVER actual: %s' % [round(ep_h[0, 3], 3), round(ep_h[1, 3], 3), round(ep_h[2, 3], 3)])"""

for p in paths:
    s = io.open(p, encoding='utf-8').read()
    assert old1 in s, 'anchor1 not found: ' + p
    assert old2 in s, 'anchor2 not found: ' + p
    s = s.replace(old1, new1)
    s = s.replace(old2, new2)
    io.open(p, 'w', encoding='utf-8').write(s)
    print('PATCHED:', p)

import py_compile
for p in paths:
    py_compile.compile(p, doraise=True)
print('SYNTAX OK')
