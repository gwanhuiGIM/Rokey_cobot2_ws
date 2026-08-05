#!/usr/bin/env python3
"""d435i rosbag 프레임 1장 → GraspGenX 입력 4파일.

만드는 것 (md/detect_graspx.md §2 의 입력 표와 같은 모양):
    rgb.png         컬러
    depth.npy       float32, **미터**   (bag은 uint16 mm — 여기서 나눈다)
    seg.png         uint8 라벨맵. 0=배경, 1=대상
    meta_data.json  intrinsics + camera_pose(4x4)

camera_pose 는 **T_base <- camera_color_optical_frame** 이다. camera_link 가 아니다.
GraspGenX 는 컬러 intrinsics 로 역투영하므로 점군이 optical 프레임에서 나오고,
optical 은 camera_link 대비 (x우, y하, z전)로 **회전이 90°씩 두 번 들어가 있다.**
camera_link 변환을 넣으면 물체가 엉뚱한 축으로 눕는다 — 에러 없이.

이 값은 T_cam2base.npy 가 아니라 **bag 안의 /tf_static 에서** 뽑는다.
npy 와 값이 같다고 문서에 적혀 있지만(md/rosbag-d435i.md §2), npy 는 camera_link 까지고
optical 까지의 나머지 두 단은 어차피 bag 에만 있다. 출처를 하나로 두는 편이 안전하다.

마스크를 만드는 방법이 둘이다. **--box 가 기본 경로다.**

    --box    수동 ROI + depth 로 테이블 제거. 모델도 GPU 도 학습도 없다.
    --yolo   yolo11n-seg (COCO). 자동이지만 이 씬에서는 못 쓴다 — 아래 참고.

⚠️ COCO 사전학습 YOLO 는 `2149_apple` 의 사과를 **못 잡는다**(2026-08-05 실측).
   imgsz=640 에서 검출 0, imgsz=1280 에서 나온 'apple' 2개는 conf 0.22/0.18 에
   박스가 (695,384)-(725,392) — 컨베이어 위 사과가 아니라 **배경 과일상자**다.
   컨베이어 사과는 화면에서 ~35x40 px 라 n/m/x 모델 어느 것도 못 봤다.
   자동 검출이 필요하면 OD_Tutorial/YOLO 로 커스텀 학습해야 한다.

사용:
    # 사과 프레임(t=30) 추출. box 는 f30 에서 눈으로 잰 값이다.
    python3 extract_frame.py <bag> -t 30 --box 596,244,634,287

    python3 extract_frame.py <bag> -t 30 --dump-rgb   # box 잴 이미지만 뽑기
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sqlite3
import subprocess
import sys

import cv2
import numpy as np

# ROS 메시지 역직렬화 — source /opt/ros/humble/setup.bash 가 필요하다
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

COLOR = "/camera/camera/color/image_raw/compressed"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
# intrinsics 는 **aligned_depth 쪽**을 쓴다. align_depth 가 depth 를 컬러 해상도로
# 리샘플하므로 두 topic 의 K 는 같아야 하지만, depth.npy 와 짝이 맞는 건 이쪽이다.
INFO = "/camera/camera/aligned_depth_to_color/camera_info"

BASE_FRAME = "base_link"
OPTICAL_FRAME = "camera_color_optical_frame"

# 이 거리 밖의 depth 는 버린다. D435i 는 먼 거리에서 노이즈가 급격히 는다.
# UNVERIFIED: 테이블 작업 거리 가정. 실측으로 조일 것.
DEPTH_MIN_M, DEPTH_MAX_M = 0.15, 2.0


def open_db(path: pathlib.Path, workdir: pathlib.Path) -> sqlite3.Connection:
    """bag 폴더/.db3/.db3.zstd 아무거나 받아 sqlite 연결을 돌려준다.

    rosbag2_py.SequentialReader 는 압축 bag 을 못 연다(md/rosbag-d435i.md §3).
    그래서 zstd 로 풀고 sqlite 로 직접 읽는다.
    """
    if path.is_dir():
        cands = sorted(path.glob("*.db3")) or sorted(path.glob("*.db3.zstd"))
        if not cands:
            sys.exit(f"{path} 안에 .db3 가 없다")
        path = cands[0]
    if path.suffix == ".zstd":
        out = workdir / path.name.removesuffix(".zstd")
        if not out.exists():
            print(f"[zstd] {path.name} 압축 해제 중...")
            subprocess.run(["zstd", "-dqf", "-o", str(out), str(path)], check=True)
        path = out
    return sqlite3.connect(path)


def camera_pose_from_bag(con: sqlite3.Connection) -> np.ndarray:
    """/tf_static 을 tf2 버퍼에 넣고 base_link <- color_optical 을 조회한다.

    체인이 base_link -> camera_link -> camera_color_frame -> camera_color_optical_frame
    3단이라 손으로 곱하면 순서를 틀린다. tf2 가 이미 하는 일을 다시 짜지 않는다.
    """
    from scipy.spatial.transform import Rotation

    buf = Buffer()
    tid = {n: i for i, n in con.execute("select id,name from topics")}
    rows = con.execute(
        "select data from messages where topic_id=?", (tid["/tf_static"],)
    )
    for (blob,) in rows:
        for tf in deserialize_message(bytes(blob), TFMessage).transforms:
            buf.set_transform_static(tf, "bag")

    tf = buf.lookup_transform(BASE_FRAME, OPTICAL_FRAME, Time())
    t, q = tf.transform.translation, tf.transform.rotation

    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def nearest(con: sqlite3.Connection, topic: str, t_ns: int):
    """bag 수신 시각 기준으로 t_ns 에 가장 가까운 메시지 하나.

    ponytail: header.stamp 가 아니라 messages.timestamp 로 고른다. 전 프레임을
    역직렬화하지 않으려는 것이고, 실기 녹화라 둘의 차는 수 ms 다. 정적 장면에선
    무의미하다. 움직이는 장면에서 프레임 정합이 중요해지면 여기를 바꾼다.
    """
    tid = {n: i for i, n in con.execute("select id,name from topics")}
    if topic not in tid:
        sys.exit(f"bag 에 {topic} 이 없다")
    row = con.execute(
        "select data,timestamp from messages where topic_id=? "
        "order by abs(timestamp-?) limit 1",
        (tid[topic], t_ns),
    ).fetchone()
    return bytes(row[0]), row[1]


def bag_t0(con: sqlite3.Connection) -> int:
    return con.execute("select min(timestamp) from messages").fetchone()[0]


def segment_box(depth, K, T, box, margin: float, slab: float):
    """수동 ROI + 테이블 제거. 학습도 GPU 도 없고 결과가 결정적이다.

    테이블 높이를 ROI **바깥 테두리**에서 잰다. 안쪽에서 재면 물체가 ROI 를
    거의 채울 때 물체 자신을 테이블로 오인해 통째로 지운다.
    """
    x1, y1, x2, y2 = box
    valid = (depth > DEPTH_MIN_M) & (depth < DEPTH_MAX_M)

    roi = np.zeros(depth.shape, bool)
    roi[y1:y2, x1:x2] = True
    pad = max(6, (x2 - x1) // 3)
    ring = np.zeros(depth.shape, bool)
    ring[max(0, y1 - pad) : y2 + pad, max(0, x1 - pad) : x2 + pad] = True
    ring &= ~roi & valid
    if ring.sum() < 30:
        sys.exit("ROI 주변 depth 가 부족하다 — box 를 물체보다 조금 작게 잡아라")

    z = np.full(depth.shape, np.nan, np.float32)
    v, u = np.nonzero(valid)
    d = depth[v, u]
    x = (u - K[0, 2]) * d / K[0, 0]
    y = (v - K[1, 2]) * d / K[1, 1]
    z[v, u] = (np.stack([x, y, d], 1) @ T[:3, :3].T + T[:3, 3])[:, 2]

    table_z = float(np.nanmedian(z[ring]))
    mask = roi & valid & (z > table_z + margin)

    # 【중요】 높이만으로는 부족하다. ROI 모서리로 보이는 **배경**(커튼·벽)도
    # 테이블보다 높아서 그대로 통과한다. 실측: 이 필터가 없으면 지름 6.6cm 사과가
    # XY 14.4cm 로 부풀어 RG2 개구 한계(10.2cm)를 넘겨 grasp 가 전부 탈락한다.
    # 물체는 depth 상 하나의 얇은 덩어리다 — 가장 가까운 층만 남긴다.
    d0 = float(np.percentile(depth[mask], 10))
    mask &= (depth > d0 - 0.02) & (depth < d0 + slab)

    print(f"[box] 테이블 z={table_z:.3f} m, 물체 depth {d0:.3f}~{d0 + slab:.3f} m → {mask.sum()} px")
    if mask.sum() < 50:
        sys.exit("물체 픽셀이 거의 없다 — --margin 을 줄이거나 box/-t 를 확인해라")
    return mask, "manual"


def segment(rgb: np.ndarray, want: str | None, weights: str):
    """yolo11n-seg 로 마스크 하나. (mask bool HxW, 라벨명)

    SAM 을 쓰지 않는 이유: ultralytics 가 이미 깔려 있고 seg 모델이 검출+마스크를
    한 번에 준다. CPU 로 충분하다. 마스크 품질이 실측으로 부족하면 그때 얹는다.
    """
    from ultralytics import YOLO

    res = YOLO(weights)(rgb[:, :, ::-1], retina_masks=True, verbose=False)[0]
    if res.masks is None or len(res.masks) == 0:
        sys.exit("검출 0개 — -t 로 다른 시각을 보거나 --weights 를 바꿔라")

    names = [res.names[int(c)] for c in res.boxes.cls]
    conf = res.boxes.conf.cpu().numpy()
    masks = res.masks.data.cpu().numpy().astype(bool)
    print(f"[yolo] 검출: {list(zip(names, conf.round(2)))}")

    if want:
        idx = [i for i, n in enumerate(names) if n == want]
        if not idx:
            sys.exit(f"'{want}' 없음. 검출된 것: {sorted(set(names))}")
        i = max(idx, key=lambda i: conf[i])
    else:
        i = int(np.argmax([m.sum() for m in masks]))  # 가장 큰 것

    m = masks[i]
    if m.shape != rgb.shape[:2]:  # retina_masks 가 안 먹은 경우 대비
        m = cv2.resize(m.astype(np.uint8), rgb.shape[1::-1], cv2.INTER_NEAREST).astype(bool)
    return m, names[i]


def backproject(depth: np.ndarray, K: np.ndarray, mask: np.ndarray, T: np.ndarray):
    """마스크 픽셀 → 베이스 프레임 점군. 온전성 확인용이지 GraspGenX 입력은 아니다."""
    v, u = np.nonzero(mask & (depth > DEPTH_MIN_M) & (depth < DEPTH_MAX_M))
    z = depth[v, u]
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    pc = np.stack([x, y, z], 1)
    return pc @ T[:3, :3].T + T[:3, 3]


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("bag", type=pathlib.Path)
    p.add_argument("-t", type=float, default=10.0, help="bag 시작 후 몇 초 시점")
    p.add_argument("--box", help="수동 ROI 'x1,y1,x2,y2' (권장)")
    p.add_argument("--margin", type=float, default=0.010, help="테이블 위 몇 m 부터 물체로 볼지")
    p.add_argument("--slab", type=float, default=0.12, help="물체 depth 두께(m). 배경 제거용")
    p.add_argument("--yolo", action="store_true", help="COCO yolo11n-seg 자동 검출 (이 씬에선 실패)")
    p.add_argument("-c", "--cls", default=None, help="--yolo 용 클래스명")
    p.add_argument("--dump-rgb", action="store_true", help="컬러만 저장하고 종료 (box 재기용)")
    p.add_argument("-o", "--out", type=pathlib.Path, default=pathlib.Path("gg_input"))
    p.add_argument("--weights", default="yolo11n-seg.pt")
    a = p.parse_args()
    if not (a.box or a.yolo or a.dump_rgb):
        sys.exit("--box 'x1,y1,x2,y2' 또는 --yolo 중 하나가 필요하다 (--dump-rgb 로 box 를 먼저 재라)")

    a.out.mkdir(parents=True, exist_ok=True)
    con = open_db(a.bag, a.out)

    T_base_optical = camera_pose_from_bag(con)
    t_ns = bag_t0(con) + int(a.t * 1e9)

    cblob, ct = nearest(con, COLOR, t_ns)
    dblob, dt = nearest(con, DEPTH, ct)
    iblob, _ = nearest(con, INFO, ct)
    print(f"[sync] color↔depth 간격 {abs(ct - dt) / 1e6:.1f} ms")

    cmsg = deserialize_message(cblob, CompressedImage)
    rgb = cv2.imdecode(np.frombuffer(cmsg.data, np.uint8), cv2.IMREAD_COLOR)

    if a.dump_rgb:
        cv2.imwrite(str(a.out / "rgb.png"), rgb)
        print(f"→ {a.out}/rgb.png ({rgb.shape[1]}x{rgb.shape[0]}). 여기서 box 를 재라.")
        return

    dmsg = deserialize_message(dblob, Image)
    assert dmsg.encoding == "16UC1", f"depth 인코딩이 {dmsg.encoding} 이다 — mm 가정 재확인"
    # 【중요】 uint16 mm → float32 m. 이 나눗셈을 빼먹으면 물체가 1km 밖에 생긴다.
    depth = (
        np.frombuffer(dmsg.data, np.uint16).reshape(dmsg.height, dmsg.width).astype(np.float32)
        / 1000.0
    )

    K = np.array(deserialize_message(iblob, CameraInfo).k).reshape(3, 3)
    assert rgb.shape[:2] == depth.shape, f"해상도 불일치 {rgb.shape} vs {depth.shape}"

    if a.box:
        box = tuple(int(v) for v in a.box.split(","))
        mask, label = segment_box(depth, K, T_base_optical, box, a.margin, a.slab)
    else:
        mask, label = segment(rgb, a.cls, a.weights)

    # 마스크를 눈으로 확인할 오버레이. 여기가 틀리면 뒤가 전부 무의미하다.
    ov = rgb.copy()
    ov[mask] = (0.4 * ov[mask] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
    cv2.imwrite(str(a.out / "mask_overlay.png"), ov)

    cv2.imwrite(str(a.out / "rgb.png"), rgb)
    np.save(a.out / "depth.npy", depth)
    # UNVERIFIED: GraspGenX 가 seg.png 를 어떤 dtype 으로 읽는지 로컬에서 확인 못 했다
    # (graspgenx 미설치). 0=배경 / 1=대상 uint8 라벨맵으로 쓴다. 원격에서 로더가
    # int32 를 요구하면 여기를 고친다.
    cv2.imwrite(str(a.out / "seg.png"), mask.astype(np.uint8))
    (a.out / "meta_data.json").write_text(
        json.dumps(
            {
                "intrinsics": {
                    "fx": K[0, 0], "fy": K[1, 1], "cx": K[0, 2], "cy": K[1, 2],
                    "width": dmsg.width, "height": dmsg.height,
                },
                "camera_pose": T_base_optical.tolist(),
                "_source": {"bag": str(a.bag), "t_sec": a.t, "label": label},
            },
            indent=2,
        )
    )

    # --- 온전성 확인: 여기 숫자가 이상하면 GPU 를 돌려도 소용없다 ---
    pc = backproject(depth, K, mask, T_base_optical)
    if len(pc) < 50:
        sys.exit(f"유효 depth 점이 {len(pc)}개뿐 — 마스크나 depth 범위를 확인해라")
    c = pc.mean(0)
    print(f"\n[{label}] {len(pc)} pts, base_link 기준")
    print(f"  중심   {c.round(3)}")
    print(f"  z 범위 {pc[:, 2].min():.3f} ~ {pc[:, 2].max():.3f} m  (테이블 상면이 바닥값)")
    print(f"  원점거리 {np.linalg.norm(c):.3f} m")

    # 개구 폭 사전 점검. XY 폭이 RG2 한계를 넘으면 grasp_selector 3단계에서
    # 전부 None 이 되어 결과가 빈 리스트로 나온다 — GraspGenX 를 돌리기 전에 안다.
    ext = pc[:, :2].max(0) - pc[:, :2].min(0)
    print(f"  XY 폭 {ext.round(3)} m, 높이 {pc[:, 2].ptp():.3f} m")
    if ext.min() > 0.102:
        print("  ⚠️ 최소 폭도 RG2 개구 한계(0.102 m) 초과 — 배경이 섞였거나 못 잡는 물체다")

    # M0609 도달 범위(grasp_selector.py 의 2단계와 같은 근사). 여기서 걸리면
    # GraspGenX 를 돌려봐야 전부 탈락한다.
    r = np.linalg.norm(c - [0, 0, 0.1345])
    ok = 0.25 <= r <= 0.855
    print(f"  shoulder 거리 {r:.3f} m → 도달 {'OK' if ok else '❌ 범위 밖'}")
    if c[2] < -0.05:
        print("  ⚠️ 물체가 베이스보다 5cm 넘게 아래다 — camera_pose 프레임 의심")
    print(f"\n→ {a.out}/ 4파일 생성. 이 폴더를 Lightning 으로 올린다.")


def _check() -> None:
    """역투영+변환이 맞는지 합성 데이터로 확인. 실제 bag 없이 돈다."""
    from scipy.spatial.transform import Rotation

    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
    depth = np.zeros((480, 640), np.float32)
    depth[240, 320] = 1.5  # 광축 위 1.5 m
    mask = np.zeros((480, 640), bool)
    mask[240, 320] = True

    T = np.eye(4)  # optical == base 인 가짜 변환
    pc = backproject(depth, K, mask, T)
    assert np.allclose(pc[0], [0, 0, 1.5]), pc  # 주점은 (0,0,z) 로 떨어져야 한다

    # 회전이 실제로 적용되는지 — 비대칭 회전이라야 전치 버그가 잡힌다
    R = Rotation.from_euler("xyz", [10, 20, 30], degrees=True).as_matrix()
    assert not np.allclose(R, R.T)
    T2 = np.eye(4)
    T2[:3, :3], T2[:3, 3] = R, [0.4, 0.1, 0.2]
    pc2 = backproject(depth, K, mask, T2)
    assert np.allclose(pc2[0], R @ [0, 0, 1.5] + [0.4, 0.1, 0.2]), pc2

    # 광축 밖 픽셀의 부호: u > cx 면 optical +x
    depth[240, 420] = 1.0
    mask[240, 420] = True
    got = backproject(depth, K, mask, np.eye(4))
    assert (got[:, 0] > 0).any(), "u>cx 인데 x 가 음수 — cx 부호 확인"

    # depth 범위 밖은 버려져야 한다
    depth[100, 100] = 99.0
    mask[100, 100] = True
    assert len(backproject(depth, K, mask, np.eye(4))) == 2, "범위 밖 depth 가 통과했다"
    print("self-check PASS")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _check()
    else:
        main()
