#!/usr/bin/env python3
"""GraspGenX GPU 워커 — 모델을 한 번만 올려놓고 stdin 으로 씬을 받는다.

실행 (반드시 GraspGenX venv 안에서. 보통은 grasp_bridge_node 가 자식 프로세스로 띄운다):
    cd ~/cobot2_ws/isaac_ros-dev/src/GraspGenX
    uv run python \
      ~/cobot2_ws/src/graspgenx_perception/graspgenx_perception/graspgen_worker.py \
      --gripper onrobot_RG2

이 파일은 패키지 디렉토리에 있지만 **rclpy 를 import 하지 않는다.** 형제 모듈이 아니라
`grasp_bridge_node` 가 경로로 실행하는 별도 프로세스다(`worker_script` 파라미터).

프로토콜 (한 줄 = 한 요청):
    stdin  <- 씬 디렉토리 경로 한 줄
    stdout -> JSON 한 줄  {"ok":true,"objects":{"obj_1":{"grasps":[[4x4]...],
                                                 "scores":[...],"n_pts":N}}}
             실패하면 {"ok":false,"error":"..."}
    ⚠️ 모델 로딩이 끝나면 stdout 에 {"ok":true,"ready":true} 한 줄을 먼저 뱉는다.
       진행 로그는 전부 **stderr** 로 간다 — stdout 은 JSON 전용이다.

왜 ZMQ 가 아닌가: 서버 모드는 `uv sync --extra serve`(pyzmq·msgpack·msgpack-numpy)를,
클라이언트 쪽은 시스템 파이썬에 같은 것을 요구한다. 양끝을 우리가 다 만드는 지금은
파이프 한 줄이면 충분하고 **새 의존성이 0개다**. GPU 를 다른 PC 로 뺄 때 ZMQ 로 갈아탄다
(그때 바뀌는 건 이 파일과 노드의 전송 계층뿐이고, 씬 포맷·필터는 그대로다).

grasp 는 **씬의 camera_pose 가 적용된 프레임**(= 우리 캡처에선 base_link)으로 나온다.
scene_loaders.load_realworld_scene:86 이 점을 world 로 보내기 때문이다.
"""

import argparse
import json
import os
import sys

# ⚠️ import 보다 먼저 해야 한다. GraspGenX 와 그 의존성들이 stdout 으로 print 하는데
#    (체크포인트 경로, 그리퍼 로딩, OpenGL 초기화 …) 그러면 JSON 파서가 첫 줄에서 죽는다.
#    stdout 은 우리 JSON 전용으로 잠그고 나머지는 전부 stderr 로 보낸다.
_STDOUT = sys.stdout
sys.stdout = sys.stderr

import numpy as np  # noqa: E402
import trimesh  # noqa: E402

from graspgenx import get_checkpoints_version_dir  # noqa: E402
from graspgenx.grasp_server import GraspGenXSampler  # noqa: E402
from graspgenx.samplers import run_planner_on_batch  # noqa: E402
from graspgenx.utils.checkpoint_io import load_model_cfg  # noqa: E402
from graspgenx.utils.collision_filter import filter_colliding_grasps  # noqa: E402
from graspgenx.utils.scene_loaders import (  # noqa: E402
    build_scene_pc_excluding_object,
    load_realworld_scene,
)


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def emit(obj):
    print(json.dumps(obj), file=_STDOUT, flush=True)


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gripper', default='onrobot_RG2')
    ap.add_argument('--checkpoints', default=None)
    ap.add_argument('--assets_dir', default=None)
    ap.add_argument('--planner', default='graspmoe', choices=['diffusion', 'graspmoe'])
    ap.add_argument('--num_grasps', type=int, default=64)      # 8GB VRAM 기준
    ap.add_argument('--grasp_threshold', type=float, default=0.5)
    # 상판 위 물건은 옆에서 못 잡는다 — 옆면 후보를 만들면 73%가 테이블과 충돌한다(2026-08-05 실측)
    ap.add_argument('--moe_obb_density', default='dense',
                    choices=['sparse', 'dense', 'dense-topandside'])
    ap.add_argument('--collision_threshold', type=float, default=0.02)
    ap.add_argument('--max_scene_points', type=int, default=8192)
    ap.add_argument('--num_collision_samples', type=int, default=2000)
    ap.add_argument('--min_obj_points', type=int, default=100)
    return ap.parse_args()


def main():
    args = parse_args()
    repo_root = os.environ.get('GRASPGENX_ROOT') or os.getcwd()
    ckpt = args.checkpoints or str(get_checkpoints_version_dir())
    log(f'[worker] checkpoints: {ckpt}')
    cfg = load_model_cfg(os.path.join(ckpt, 'gen'), os.path.join(ckpt, 'dis'), None, None)

    sampler = GraspGenXSampler(
        cfg, args.gripper,
        assets_dir=args.assets_dir or os.path.join(repo_root, 'assets'))
    gripper = sampler.get_gripper_info()
    surf, _ = trimesh.sample.sample_surface(
        gripper.collision_mesh, args.num_collision_samples)
    surf = np.asarray(surf, dtype=np.float32)
    log(f'[worker] gripper={args.gripper} 준비 완료')
    emit({'ok': True, 'ready': True})

    for line in sys.stdin:
        scene_dir = line.strip()
        if not scene_dir:
            continue
        try:
            emit(handle(scene_dir, sampler, surf, args))
        except Exception as e:                                   # noqa: BLE001
            log(f'[worker] 실패: {type(e).__name__}: {e}')
            emit({'ok': False, 'error': f'{type(e).__name__}: {e}'})


def handle(scene_dir, sampler, gripper_surface_points, args):
    scene = load_realworld_scene(scene_dir, min_obj_points=args.min_obj_points)
    labels = list(scene['objects'].keys())
    if not labels:
        return {'ok': True, 'objects': {}, 'note': 'segmented object 없음'}

    results = run_planner_on_batch(
        [scene['objects'][k]['pc'] for k in labels], sampler,
        planner=args.planner,
        grasp_threshold=args.grasp_threshold,
        num_grasps=args.num_grasps,
        topk_num_grasps=-1,
        moe_obb_density=args.moe_obb_density,
        # planner 기본값은 (-8,-6,-4,-2,-1,0) 인데 demo_scene_pc.py 는 "-2,0" 을 쓴다.
        # 2026-08-05 에 68 grasp 를 얻은 건 demo 쪽 값이라 그걸 따른다.
        moe_z_offsets_cm=(-2.0, 0.0),
    )

    out = {}
    for label, (grasps, conf, tags, _obb) in zip(labels, results):
        if len(grasps) == 0:
            out[label] = {'grasps': [], 'scores': [], 'n_pts': len(scene['objects'][label]['pc'])}
            continue
        scene_pc = build_scene_pc_excluding_object(scene, label)
        if len(scene_pc) > args.max_scene_points:
            idx = np.random.choice(len(scene_pc), args.max_scene_points, replace=False)
            scene_pc = scene_pc[idx]
        keep = filter_colliding_grasps(
            scene_pc=scene_pc, grasp_poses=grasps,
            collision_threshold=args.collision_threshold,
            gripper_surface_points=gripper_surface_points)
        grasps, conf = grasps[keep], conf[keep]
        tags = [t for t, k in zip(tags, keep) if k]
        log(f'[worker] {label}: {len(grasps)} free '
            f'(diff={tags.count("diff")}, obb={tags.count("obb")})')
        out[label] = {
            'grasps': np.asarray(grasps, dtype=float).tolist(),
            'scores': np.asarray(conf, dtype=float).tolist(),
            'n_pts': int(len(scene['objects'][label]['pc'])),
        }
    return {'ok': True, 'objects': out}


if __name__ == '__main__':
    main()
