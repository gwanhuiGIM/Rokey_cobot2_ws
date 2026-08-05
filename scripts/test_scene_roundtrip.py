"""2단계: capture_graspgenx_scene.py 가 쓴 디렉토리를 GraspGenX **자체 loader**로 읽는다.

왜 필요한가: 스키마를 소스 읽어서 맞췄을 뿐 실제로 로드해본 적이 없다.
카메라가 오기 전에 포맷 오류를 여기서 잡는다 — 실기 시간에 디버깅하지 않으려고.

실행 (GraspGenX venv 안에서):
    cd ~/cobot2_ws/isaac_ros-dev/src/GraspGenX
    uv run python ~/cobot2_ws/scripts/test_scene_roundtrip.py /tmp/graspgenx_scene_test/00
"""
import sys

import numpy as np
from graspgenx.utils.scene_loaders import (
    build_scene_pc_excluding_object,
    collect_scene_items,
    load_realworld_scene,
)

scene_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/graspgenx_scene_test/00'
parent = scene_dir.rsplit('/', 1)[0]

# ① demo_scene_pc.py 가 부모 디렉토리에서 씬을 찾는 경로와 같은 함수
items = collect_scene_items(parent)
print('collect_scene_items ->', items)
assert items and items[0][0] == 'realworld', f'realworld 포맷으로 인식되지 않았다: {items}'

# ② 실제 로드
scene = load_realworld_scene(scene_dir)
print('scene keys :', sorted(k for k in scene if not k.startswith('_')))
print('objects    :', {k: len(v["pc"]) for k, v in scene['objects'].items()})
print('scene_xyz  :', scene['scene_xyz'].shape)
assert set(scene['objects']) == {'obj_1', 'obj_2'}, scene['objects'].keys()

# ③ 좌표가 base 프레임으로 제대로 왔는지 — 합성값과 대조
for name, expect_z in (('obj_1', 0.05), ('obj_2', 0.06)):
    pc = scene['objects'][name]['pc']
    z = float(np.median(pc[:, 2]))
    print(f'  {name}: {len(pc)} pts, median z = {z:.4f} m (기대 {expect_z})')
    assert abs(z - expect_z) < 1e-3, f'{name} z 가 {z}, 기대 {expect_z} — 변환이 틀렸다'
    cx, cy = float(np.median(pc[:, 0])), float(np.median(pc[:, 1]))
    assert 0.10 < cx < 0.90 and -0.5 < cy < 0.5, f'{name} 이 작업공간 밖: ({cx}, {cy})'

# ④ 충돌 필터가 쓰는 "대상 제외 씬" — 대상 점이 빠지고 나머지(테이블·다른 물체)는 남아야 한다
full = len(scene['scene_xyz'])
for name in ('obj_1', 'obj_2'):
    rest = build_scene_pc_excluding_object(scene, name)
    removed = full - len(rest)
    print(f'  scene minus {name}: {len(rest)} pts (제거 {removed})')
    assert removed == len(scene['objects'][name]['pc']), (removed, len(scene['objects'][name]['pc']))

# ⑤ 라벨 없는 픽셀(배경 0)도 씬 점군에 남는지 = 장애물이 라벨 없이도 충돌에 잡히는 근거
labeled = sum(len(v['pc']) for v in scene['objects'].values())
print(f'  전체 {full} pts 중 라벨된 물체 {labeled} pts → 나머지 {full - labeled} pts 가 장애물 역할')
assert full - labeled > 0

print('PASS')
