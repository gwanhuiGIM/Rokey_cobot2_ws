"""합성 장면(테이블 평면 + 상자 2개)으로 segment()/write_scene() 검증.

실행:  python3 scripts/test_capture_graspgenx_scene.py [출력디렉토리]
       (시스템 파이썬. 카메라·로봇·GPU 불필요)

2단계 검증의 1단계다. 2단계는 여기서 쓴 디렉토리를 GraspGenX venv 의 loader 로
실제로 읽어보는 것 — scripts/test_scene_roundtrip.py 참고.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capture_graspgenx_scene import DEFAULTS, segment, write_scene  # noqa: E402

H, W = 240, 320
K = np.array([[300., 0., 160.], [0., 300., 120.], [0., 0., 1.]])
# 카메라가 (0.5, 0, 1.0)에서 수직 아래를 본다: x_c->+x, y_c->-y, z_c->-z
T = np.eye(4)
T[:3, :3] = np.array([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])
T[:3, 3] = [0.5, 0., 1.0]

depth = np.full((H, W), 1.00, np.float32)      # 테이블 -> base z = 0.0
depth[100:140, 130:170] = 0.95                 # 물체1: 5cm
depth[100:140, 200:240] = 0.94                 # 물체2: 6cm

p = dict(DEFAULTS)
p['table_z'] = float('nan')
seg, label_map, diag = segment(depth, K, T, p)
print(diag)
print('label_map =', label_map)
u = sorted(np.unique(seg).tolist())
print('unique(seg) =', u)
assert u == [0, 2, 101, 102], f'라벨이 배경0+테이블2+물체2개가 아니다: {u}'
assert label_map == {'ground': 0, 'table': 2, 'obj_1': 101, 'obj_2': 102}, label_map
assert (seg[110:130, 140:160] == 101).all() and (seg[110:130, 210:230] == 102).all()

# 붙어 있는 두 물체는 하나로 뭉쳐야 한다(설계상 한계를 고정한다)
d2 = np.full((H, W), 1.00, np.float32)
d2[100:140, 130:240] = 0.95
_, lm2, _ = segment(d2, K, T, dict(p))
assert lm2 == {'ground': 0, 'table': 2, 'obj_1': 101}, lm2

# 파일 4개를 실제로 쓴다 — 2단계에서 GraspGenX loader 가 이 디렉토리를 읽는다
out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/graspgenx_scene_test/00'
rgb = np.zeros((H, W, 3), np.uint8)
rgb[..., 0] = 200                              # 빨간 톤 — RGB/BGR 뒤바뀜을 눈으로 잡으려고
write_scene(out, depth, rgb, seg, K, T, label_map,
            [p['x_min'], p['y_min'], p['z_min'], p['x_max'], p['y_max'], p['z_max']])
for f in ('depth.npy', 'rgb.png', 'seg.png', 'meta_data.json'):
    assert os.path.exists(os.path.join(out, f)), f
print(f'wrote {out}')
print('PASS')
