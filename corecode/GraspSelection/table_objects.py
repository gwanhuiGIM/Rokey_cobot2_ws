#!/usr/bin/env python3
"""테이블 위 물체들 → 각각의 중심 좌표 + 크기 (base_link 기준).

메쉬도 6D pose도 학습도 없다. 고정 카메라 + 평평한 테이블이라는 두 조건만 쓴다.
extract_frame.py 가 물체 1개를 수동 ROI 로 잘라내는 것과 달리, 여기는 테이블 위
**전부**를 한 번에 찾는다. 6~7종을 다룰 거면 이쪽이 기본 경로다.

절차:
    depth → base_link 점군 → WORKSPACE 로 자름 → 테이블 z 추정 → 그 위 점만 남김
    → DBSCAN 클러스터링 → 클러스터마다 중심·크기

【중심 계산이 평균이 아닌 이유】 카메라는 물체의 **앞면만** 본다. 점군 평균은 그래서
카메라 쪽으로 물체 반지름의 절반쯤 치우친다(사과 6.6cm 면 ~1.5cm). 대신:
    XY 중심 = 실루엣 bbox 중심   (윗쪽 시점이라 좌우 경계는 양쪽 다 보인다)
    Z  중심 = (테이블면 + 최고점) / 2   (바닥은 테이블 높이로 이미 안다)
평균을 쓰면 그리퍼가 물체 앞쪽을 집으려다 스친다.

사용:
    python3 table_objects.py <bag> -t 30
    python3 table_objects.py --selftest        # bag 없이 로직 확인
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from extract_frame import (  # noqa: E402
    DEPTH,
    DEPTH_MAX_M,
    DEPTH_MIN_M,
    INFO,
    bag_t0,
    camera_pose_from_bag,
    nearest,
    open_db,
)

# 【보정 knob】 base_link 기준 작업 영역. 커튼·벽·바닥을 자르는 유일한 수단이다.
# 이 값은 d435i_0803_2149_apple bag(t=30)의 점 분포에서 역산한 것이지 **줄자 실측이 아니다.**
# 카메라를 옮기거나 재캘리브하면 반드시 다시 잡는다 — `--ws` 로 덮어쓸 수 있다.
WORKSPACE = dict(x=(-0.10, 0.98), y=(-0.10, 0.38), z=(-0.12, 0.40))

TABLE_MARGIN_M = 0.012  # 테이블면 위 몇 m 부터 물체로 볼지 (얇은 물체면 줄인다)
MAX_OBJ_H_M = 0.35  # 이보다 높은 건 물체가 아니다 (사람 손·기둥)
CLUSTER_EPS_M = 0.02  # DBSCAN 이웃 거리. 물체 간격보다 작아야 안 붙는다
MIN_PTS = 80  # 이보다 작은 클러스터는 노이즈
RG2_MAX_OPEN_M = 0.102


def to_base(depth: np.ndarray, K: np.ndarray, T: np.ndarray, stride: int = 2) -> np.ndarray:
    """depth 전체를 base_link 점군으로. stride 로 솎는다 — 중심 계산엔 충분하다."""
    d = depth[::stride, ::stride]
    v, u = np.nonzero((d > DEPTH_MIN_M) & (d < DEPTH_MAX_M))
    z = d[v, u]
    u, v = u * stride, v * stride
    pc = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)
    return pc @ T[:3, :3].T + T[:3, 3]


def find_objects(pc: np.ndarray, ws: dict | None = None) -> tuple[list[dict], np.ndarray]:
    """base 프레임 점군 → (물체 목록, 테이블 평면 [a,b,c,d])."""
    ws = ws or WORKSPACE
    for i, ax in enumerate("xyz"):
        lo, hi = ws[ax]
        pc = pc[(pc[:, i] > lo) & (pc[:, i] < hi)]
    if len(pc) < 500:
        sys.exit(f"작업영역 안 점이 {len(pc)}개뿐 — WORKSPACE/--ws 를 확인해라")

    # 【중요】 z 중앙값으로 테이블을 잡으면 안 된다. 실측 bag 에서 테이블 z 가 10 cm 폭으로
    # 퍼진다(평면이 기울었거나 캘리브 오차). 평면을 실제로 맞춰야 물체 높이가 의미를 갖는다.
    p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc))
    plane, inliers = p.segment_plane(distance_threshold=0.008, ransac_n=3, num_iterations=300)
    plane = np.asarray(plane)
    if len(inliers) < 0.15 * len(pc):
        print(f"⚠️ 평면 inlier 가 {len(inliers)}/{len(pc)} 뿐 — 작업영역에 테이블이 안 들어왔다")

    # RANSAC 은 법선 부호를 안 정해준다. base_link 에서 테이블 법선은 위(+z)다.
    if plane[2] < 0:
        plane = -plane
    h = (pc @ plane[:3] + plane[3]) / np.linalg.norm(plane[:3])  # 평면까지 부호 있는 거리

    above = pc[(h > TABLE_MARGIN_M) & (h < MAX_OBJ_H_M)]
    ha = h[(h > TABLE_MARGIN_M) & (h < MAX_OBJ_H_M)]
    if len(above) < MIN_PTS:
        return [], plane

    lab = np.array(
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(above)).cluster_dbscan(
            eps=CLUSTER_EPS_M, min_points=10
        )
    )

    objs = []
    for k in range(lab.max() + 1):
        m = lab == k
        q = above[m]
        if len(q) < MIN_PTS:
            continue
        lo, hi = q.min(0), q.max(0)
        top = float(ha[m].max())  # 테이블면 위 최고 높이
        objs.append(
            {
                # XY 는 실루엣 bbox 중심, Z 는 (테이블면 + 최고점)/2. 점군 평균이 아니다.
                "center": np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2] - top / 2]),
                "size": np.array([hi[0] - lo[0], hi[1] - lo[1], top]),
                "n": len(q),
            }
        )
    objs.sort(key=lambda o: -o["n"])
    return objs, plane


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("bag", type=pathlib.Path)
    p.add_argument("-t", type=float, default=10.0, help="bag 시작 후 몇 초 시점")
    p.add_argument("-o", "--out", type=pathlib.Path, default=pathlib.Path("gg_input"))
    p.add_argument("--ws", help="작업영역 덮어쓰기 'x1,x2,y1,y2,z1,z2' (m, base_link)")
    a = p.parse_args()
    ws = None
    if a.ws:
        v = [float(x) for x in a.ws.split(",")]
        ws = dict(x=tuple(v[0:2]), y=tuple(v[2:4]), z=tuple(v[4:6]))

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image

    a.out.mkdir(parents=True, exist_ok=True)
    con = open_db(a.bag, a.out)
    T = camera_pose_from_bag(con)

    dblob, dt = nearest(con, DEPTH, bag_t0(con) + int(a.t * 1e9))
    iblob, _ = nearest(con, INFO, dt)
    dmsg = deserialize_message(dblob, Image)
    assert dmsg.encoding == "16UC1", f"depth 인코딩이 {dmsg.encoding} 이다"
    depth = (
        np.frombuffer(dmsg.data, np.uint16).reshape(dmsg.height, dmsg.width).astype(np.float32)
        / 1000.0
    )
    K = np.array(deserialize_message(iblob, CameraInfo).k).reshape(3, 3)

    pc = to_base(depth, K, T)
    print(f"[pc] {len(pc)} pts, z 분포 p5/p50/p95 = "
          f"{np.percentile(pc[:, 2], [5, 50, 95]).round(3)}")

    objs, plane = find_objects(pc, ws)
    tilt = np.degrees(np.arccos(abs(plane[2]) / np.linalg.norm(plane[:3])))
    print(f"[table] 평면 {plane.round(3)} (수평 대비 {tilt:.1f}°) → 물체 {len(objs)}개\n")
    for i, o in enumerate(objs):
        c, s = o["center"], o["size"]
        w = min(s[0], s[1])
        flag = "" if w <= RG2_MAX_OPEN_M else "  ⚠️ RG2 개구(0.102) 초과"
        print(f"  #{i} 중심 {c.round(3)}  크기 {s.round(3)} m  ({o['n']} pts){flag}")


def _check() -> None:
    """합성 테이블 + 물체 2개로 분리·중심·크기를 확인. bag 없이 돈다."""
    rng = np.random.default_rng(0)
    table = np.column_stack(
        [rng.uniform(0.2, 0.9, 4000), rng.uniform(-0.4, 0.4, 4000), np.full(4000, 0.10)]
    )
    # 구 표면 중 **카메라를 향한 면만** 남긴다 — 실제로 depth 가 주는 것이 이것뿐이다.
    # 카메라는 물체보다 x 가 작고 위에 있다고 본다(비스듬한 시선).
    n = rng.normal(size=(2000, 3))
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    view = np.array([0.6, 0.0, -0.8])  # 카메라 → 물체 방향
    n = n[n @ view < 0]  # 법선이 카메라를 향하는 면
    ball = np.array([0.5, 0.0, 0.10 + 0.033]) + 0.033 * n
    box = np.column_stack(
        [rng.uniform(0.7, 0.75, 400), rng.uniform(0.2, 0.25, 400), rng.uniform(0.11, 0.16, 400)]
    )
    ws = dict(x=(0.1, 1.0), y=(-0.5, 0.5), z=(0.0, 0.5))
    objs, plane = find_objects(np.vstack([table, ball, box]), ws)
    assert abs(abs(plane[3] / plane[2]) - 0.10) < 0.005, plane  # 평면이 z=0.10 이어야
    assert len(objs) == 2, [o["size"] for o in objs]

    ball_o = min(objs, key=lambda o: abs(o["center"][0] - 0.5))
    truth = np.array([0.5, 0.0, 0.10 + 0.033])
    assert np.allclose(ball_o["center"], truth, atol=0.006), ball_o["center"]
    assert np.allclose(ball_o["size"], 0.066, atol=0.008), ball_o["size"]

    # 점군 평균은 카메라 쪽(-x)으로 치우친다. 보정 중심이 더 가까워야 이 코드가 의미 있다.
    naive = ball.mean(0)
    e_naive, e_fix = np.linalg.norm(naive - truth), np.linalg.norm(ball_o["center"] - truth)
    assert e_fix < e_naive / 2, f"편향 보정 실패 naive={e_naive:.4f} fix={e_fix:.4f}"
    print(f"self-check PASS (평균 오차 {e_naive * 1000:.1f} mm → 보정 {e_fix * 1000:.1f} mm)")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _check()
    else:
        main()
