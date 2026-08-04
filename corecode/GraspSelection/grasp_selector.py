#!/usr/bin/env python3
"""GraspGenX 출력 → M0609+RG2가 실제로 실행할 grasp 선별·순위.

GraspGenX가 주는 것:  (K,4,4) pose + (K,) 신뢰도.  그게 전부다.
GraspGenX가 안 주는 것: 개구 폭, 도달 가능성, 로봇 자세 선호, TCP 오프셋,
                        그리고 RG2 드라이버가 먹는 커맨드 단위.

설계 원칙 — 싼 필터부터 (처리속도의 핵심):
    0. 신뢰도 컷        공짜
    1. 접근 방향        O(K)
    2. 도달 거리        O(K)
    3. 개구 폭          O(K·N_obj)
    4. 씬 충돌          GPU cdist        ← 유일하게 비싼 단계
    5. 점수·정렬        O(K log K)

4번은 (batch·M·N) 거리행렬을 만든다. 앞에서 K를 미리 줄여두면 그만큼 싸진다.
순서를 바꾸면 안 되는 이유가 이것이다.
(각 단계의 실제 잔존 개수는 씬에 따라 다르다 — 미측정.)

재구현하지 않는 것 (상류에 이미 있다):
    graspgenx.utils.collision_filter.filter_colliding_grasps  — GPU 배치 충돌
    graspgenx.utils.scene_loaders.build_scene_pc_excluding_object  — 씬 구성
    onrobot_rg_control._OnRobotRGIsaacSimController.widthToJointValue — 폭→각도
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

# ---------------------------------------------------------------------------
# 하드웨어 보정값 — 실물은 도면과 다르다. 코드에서 지우지 말고 여기서 튜닝한다.
# ---------------------------------------------------------------------------

# RG2 개구 폭 한계가 **출처마다 다르다**. 둘 다 사실이고 용도가 다르다:
#   0.102 = onrobot_RG2/config.json 의 sweep_volume.extents[0]
#           → 모델이 conditioning에 쓴 볼륨. grasp 타당성은 이 기준이어야 한다.
#   0.110 = 드라이버 클램프 max_width=1100 (1/10mm)
#           → 하드웨어가 물리적으로 벌릴 수 있는 최대.
# 모델이 0.102를 가정하고 뽑은 grasp이므로 **선별에는 좁은 쪽(0.102)** 을 쓴다.
# 주의: XGripperInfo.width(=0.152)는 그리퍼 "몸통 폭"이지 개구 폭이 아니다.
RG2_MAX_OPENING_M = 0.102  # 선별 기준 (sweep_volume)
RG2_HW_MAX_OPENING_M = 0.110  # 드라이버 클램프
RG2_MIN_OPENING_M = 0.002

# 물체 폭에 더할 여유. 너무 작으면 접근 중 물체를 치고, 크면 헛집는다.
# UNVERIFIED: 실기에서 3~4점 재서 보정할 것. 도면값 아님.
CLEARANCE_M = 0.008

# grasp 원점(그리퍼 base_link)에서 손가락 끝까지. config.json fingertip[2].
TCP_OFFSET_M = 0.22

# sweep volume을 +Z로 확장하는 양. config.json standoff[1].
# x_grippers.py:171-178 의 grasp_volume 정의와 맞추기 위한 것.
SWEEP_STANDOFF_M = 0.04

# --- M0609 도달 범위 ---
# 근거: dsr_description2/xacro/macro.m0609.white.xacro
#   :34   base→joint_1  z = 0.1345   (shoulder 높이)
#   :118  0.411  +  :148  0.368  +  :220  0.121   =  0.900
# 즉 0.900은 **shoulder 기준 flange까지**의 최대 신장 거리다.
# base 원점 기준이 아니다 — 원점으로 재면 테이블 높이만큼 낙관적이 된다.
M0609_SHOULDER_Z_M = 0.1345
M0609_REACH_M = 0.900
# 완전 신장은 특이점이라 실제로 못 쓴다. 여유율.
REACH_SAFETY = 0.95
# 베이스 근처도 IK 특이점·자기충돌이 잦다.
SHOULDER_KEEPOUT_M = 0.25

# 씬 충돌 임계값. 상류 utils/point_cloud.py:215 의 기본값과 맞춘다.
# 하향 파지에서는 scene_pc에 **테이블 상판이 포함**되고 손가락이 물체 밑동까지
# 내려가므로, 이 값을 키우면 정답이 통째로 탈락한다. 키우기 전에 왜 키우는지 적을 것.
COLLISION_THRESHOLD_M = 0.002

# 점수 가중치. 신뢰도 대비 "수직 접근 선호"를 얼마나 볼지.
# UNVERIFIED: 실기 성공률로 튜닝할 값. 근거 있는 수치 아님.
W_VERTICALITY = 0.25

# 위쪽에서 찌르는(테이블을 향하는) 접근만 허용. cos 임계값.
# 0.30 → acos(0.3)=72.5°, 즉 수평에서 17.5° 아래까지 허용.
# UNVERIFIED: 테이블 상면 파지 전제. 선반·측면 파지 시나리오에서는 정답을 지운다.
MIN_APPROACH_DOWNWARD = 0.30

# 스윕볼륨 안에 최소 이 개수는 있어야 "물체를 문다"고 본다.
# UNVERIFIED: 포인트클라우드 밀도 의존. 얇은 물체에서 재조정 필요.
MIN_POINTS_IN_SWEEP = 3


@dataclass
class GraspCandidate:
    """실행 가능한 grasp 하나."""

    pose: np.ndarray  # (4,4) 로봇 베이스 프레임, 그리퍼 base_link(=flange) 기준
    tcp_pose: np.ndarray  # (4,4) 손가락 끝 기준 — IK 목표
    width_m: float  # 개구 폭 (미터)
    confidence: float  # GraspGenX discriminator 점수
    score: float  # 최종 순위 점수

    @property
    def rgwd(self) -> int:
        """RG2 드라이버 커맨드 값. **단위가 1/10 mm다.**

        근거: onrobot_rg_msgs/msg/OnRobotRGOutput.msg
              "rgwd ... must be provided in 1/10th millimeters.
               The valid range is 0 to 1100 for the RG2"

        m를 mm로만 바꿔서 넣으면 **10배 좁게** 명령해 물체를 으깬다.
        """
        return int(round(self.width_m * 10000.0))


# ---------------------------------------------------------------------------
# 단위 — 여기서 1000배 사고가 난다
# ---------------------------------------------------------------------------


def load_hand_eye(path: str) -> np.ndarray:
    """T_cam2base.npy 를 읽어 **미터 단위 T_base_cam** 으로 돌려준다.

    파일 이름은 cam2base지만 내용은 T_base<-cam 이다
    (eye2hand_calibration.py:696 주석에 명시).
    병진이 mm로 저장돼 있어 그대로 쓰면 1000배 어긋난다.

    이 결과를 GraspGenX meta_data.json 의 camera_pose 로 넣으면
    GraspGenX 출력이 곧 로봇 베이스 프레임이 된다 — select_grasps()의 전제.
    """
    T = np.load(path).astype(np.float64).copy()
    if np.linalg.norm(T[:3, 3]) > 10.0:  # 10m 밖 = mm로 저장된 것
        T[:3, 3] /= 1000.0
    return T


# ---------------------------------------------------------------------------
# 개구 폭 — 모델이 안 주므로 직접 잰다
# ---------------------------------------------------------------------------


def compute_grasp_width(
    pose: np.ndarray,
    obj_pc: np.ndarray,
    sweep_extents: np.ndarray,
    sweep_offset: np.ndarray,
) -> Optional[float]:
    """grasp 하나가 물체를 잡을 때 필요한 개구 폭(m). 불가하면 None.

    grasp 프레임 규약 (robot.py:59):
        +Z = 접근 축,  +X = 닫히는 방향,  원점 = 그리퍼 base_link
    """
    R, t = pose[:3, :3], pose[:3, 3]
    local = (obj_pc - t) @ R  # world → gripper local. (p @ R)_j = (Rᵀp)_j

    half = sweep_extents / 2.0
    # z는 standoff만큼 위로 확장 (x_grippers.py:171-178 의 grasp_volume 규약)
    lo = sweep_offset - half
    hi = sweep_offset + half
    hi = hi + np.array([0.0, 0.0, SWEEP_STANDOFF_M])

    in_yz = np.all((local[:, 1:] >= lo[1:]) & (local[:, 1:] <= hi[1:]), axis=1)
    if in_yz.sum() < MIN_POINTS_IN_SWEEP:
        return None

    x_in_slab = local[in_yz, 0]

    # 【중요】 x를 박스로 자르면 아무리 넓은 물체도 폭이 extents[0]로 클리핑돼
    # "딱 맞는다"고 통과한다. 자르기 전에 넘치는 점이 있는지 먼저 본다.
    # 이 가드가 없으면 CLEARANCE_M을 0으로 튜닝하는 순간 30cm 물체가 통과한다.
    if np.any(x_in_slab < lo[0]) or np.any(x_in_slab > hi[0]):
        return None  # 손가락 폭을 넘는 물체

    inside = in_yz & (local[:, 0] >= lo[0]) & (local[:, 0] <= hi[0])
    if inside.sum() < MIN_POINTS_IN_SWEEP:
        return None  # 손가락 사이에 물체가 없다 = 헛집음

    w = float(np.ptp(local[inside, 0])) + CLEARANCE_M
    if not (RG2_MIN_OPENING_M <= w <= RG2_MAX_OPENING_M):
        return None
    return w


def grasp_to_tcp(pose: np.ndarray) -> np.ndarray:
    """그리퍼 base_link 기준 pose → 손가락 끝(TCP) 기준 pose.

    +Z가 접근 축이므로 로컬 Z로 TCP_OFFSET_M 이동.
    x_grippers.py:164 의 tool_tcp_transform = translation([0,0,0.18]) 과 동일.
    부호를 틀리면 로봇이 물체를 18cm 뚫고 들어간다. RViz에서 먼저 확인할 것.
    """
    out = pose.copy()
    out[:3, 3] = pose[:3, 3] + pose[:3, 2] * TCP_OFFSET_M
    return out


# ---------------------------------------------------------------------------
# 선별 캐스케이드
# ---------------------------------------------------------------------------


def select_grasps(
    grasps: np.ndarray,
    confidences: np.ndarray,
    obj_pc: np.ndarray,
    sweep_extents,
    sweep_offset,
    *,
    scene_pc: Optional[np.ndarray] = None,
    gripper_collision_mesh=None,
    gripper_surface_points: Optional[np.ndarray] = None,
    min_confidence: float = 0.5,
    collision_threshold: float = COLLISION_THRESHOLD_M,
    top_k: int = 10,
) -> list[GraspCandidate]:
    """GraspGenX 출력에서 실행 가능한 grasp를 골라 점수순으로 돌려준다.

    Args:
        grasps:      (K,4,4) **로봇 베이스 프레임**. camera_pose에 load_hand_eye()
                     결과를 넣었다면 GraspGenX 출력이 이미 이 프레임이다.
                     이 모듈은 프레임을 검증할 방법이 없다 — 카메라 프레임을
                     넣으면 조용히 이상한 답이 나온다. 호출자 책임.
        confidences: (K,)
        obj_pc:      (N,3) 대상 물체 포인트클라우드 (같은 프레임)
        sweep_extents/offset: config.json 의 sweep_volume 값
        scene_pc:    (M,3) 대상 물체를 **제외한** 주변 점들.
                     graspgenx.utils.scene_loaders.build_scene_pc_excluding_object()
                     로 만든다. None이면 충돌 검사를 건너뛴다 — 실기 투입 금지.
        gripper_collision_mesh: trimesh.Trimesh. scene_pc를 줬는데 이것도
                     gripper_surface_points도 없으면 ValueError.
        gripper_surface_points: 미리 뽑아둔 (M,3) 표면 샘플. 여러 객체를 도는
                     루프에서는 이걸 한 번만 뽑아 재사용한다 (demo_scene_pc.py:275).

    Returns:
        점수 내림차순 GraspCandidate 리스트 (최대 top_k개).

    Raises:
        ValueError: scene_pc는 줬는데 그리퍼 형상을 안 준 경우.
    """
    if scene_pc is not None and len(scene_pc) > 0:
        if gripper_collision_mesh is None and gripper_surface_points is None:
            raise ValueError(
                "scene_pc를 주면 gripper_collision_mesh 또는 "
                "gripper_surface_points 중 하나는 필수다. "
                "grasp_sampler.get_gripper_info().collision_mesh 를 넘겨라."
            )

    grasps = np.asarray(grasps, dtype=np.float64)
    confidences = np.asarray(confidences, dtype=np.float64)
    sweep_extents = np.asarray(sweep_extents, dtype=np.float64)
    sweep_offset = np.asarray(sweep_offset, dtype=np.float64)

    keep = np.arange(len(grasps))

    # --- 0. 신뢰도 (공짜) ---
    keep = keep[confidences[keep] >= min_confidence]
    if len(keep) == 0:
        return []

    # --- 1. 접근 방향 (O(K)) ---
    # grasps[i,2,2] = 접근축(3열)의 월드 Z 성분. 아래를 향해야 하므로 음수.
    downward = -grasps[keep, 2, 2]  # 1.0 = 완전 수직 하향
    keep = keep[downward >= MIN_APPROACH_DOWNWARD]
    if len(keep) == 0:
        return []

    # --- 2. 도달 거리 (O(K)) ---
    # grasp 원점 = 그리퍼 base_link = flange 장착면. M0609_REACH_M이 shoulder→
    # flange 기준이므로 **flange 위치를 shoulder에서** 재야 한다.
    # TCP로 재면 그리퍼 길이만큼 비관적, base 원점에서 재면 낙관적이 된다.
    flange = grasps[keep, :3, 3].copy()
    flange[:, 2] -= M0609_SHOULDER_Z_M
    r = np.linalg.norm(flange, axis=1)
    keep = keep[(r <= M0609_REACH_M * REACH_SAFETY) & (r >= SHOULDER_KEEPOUT_M)]
    if len(keep) == 0:
        return []

    # --- 3. 개구 폭 (O(K·N)) ---
    widths = {}
    survivors = []
    for i in keep:
        w = compute_grasp_width(grasps[i], obj_pc, sweep_extents, sweep_offset)
        if w is not None:
            widths[i] = w
            survivors.append(i)
    keep = np.asarray(survivors, dtype=int)
    if len(keep) == 0:
        return []

    # --- 4. 씬 충돌 (GPU, 유일하게 비쌈) ---
    # 여기 오기 전에 K가 이미 줄어 있다. 그게 이 순서의 이유다.
    if scene_pc is not None and len(scene_pc) > 0:
        from graspgenx.utils.collision_filter import filter_colliding_grasps

        free = filter_colliding_grasps(  # (len(keep),) bool, True = 충돌 없음
            scene_pc,
            grasps[keep],
            gripper_collision_mesh=gripper_collision_mesh,
            collision_threshold=collision_threshold,
            gripper_surface_points=gripper_surface_points,
        )
        keep = keep[free]
        if len(keep) == 0:
            return []

    # --- 5. 점수 ---
    # 신뢰도가 주(主)이고 수직성은 동점을 가르는 보정이다. 뒤집지 말 것 —
    # 수직성을 주로 두면 모델이 나쁘다고 한 grasp를 자세 좋다고 뽑는다.
    out = []
    for i in keep:
        vert = float(-grasps[i, 2, 2])
        score = float(confidences[i]) * (1.0 + W_VERTICALITY * vert)
        out.append(
            GraspCandidate(
                pose=grasps[i],
                tcp_pose=grasp_to_tcp(grasps[i]),
                width_m=widths[i],
                confidence=float(confidences[i]),
                score=score,
            )
        )
    out.sort(key=lambda c: c.score, reverse=True)
    return out[:top_k]


# ---------------------------------------------------------------------------
# 자체 점검 — 로직이 깨지면 여기서 터진다
#
# 주의: 회전행렬을 대칭행렬(diag(1,-1,-1), I 등)로만 쓰면 R == R.T 라서
#       전치 버그를 절대 못 잡는다. 아래는 일부러 **비대칭** 회전을 쓴다.
# ---------------------------------------------------------------------------


def _pose(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _rot(axis: str, a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _demo() -> None:
    import tempfile

    # --- 단위 변환 ---
    with tempfile.NamedTemporaryFile(suffix=".npy", delete=True) as f:
        np.save(f.name, _pose(np.eye(3), [1063.2, 1166.0, 587.0]))
        T = load_hand_eye(f.name)
    assert np.linalg.norm(T[:3, 3]) < 2.0, "mm→m 변환이 안 됐다"

    # --- RG2 커맨드 단위: 1/10 mm ---
    c = GraspCandidate(np.eye(4), np.eye(4), width_m=0.050, confidence=1.0, score=1.0)
    assert c.rgwd == 500, f"50mm는 rgwd 500이어야 한다 (1/10mm). 실제 {c.rgwd}"
    assert c.rgwd <= 1100, "드라이버 유효범위 초과"

    sweep_ext = np.array([0.102, 0.02, 0.028])
    sweep_off = np.array([0.0, 0.0, 0.175])

    # --- 폭 계산: 비대칭 회전으로 전치 버그를 잡는다 ---
    # yaw 40° + x축 170° 기울임. R_y(π)·R_z(θ)는 **여전히 대칭**이라 못 쓴다.
    R_down = _rot("z", np.deg2rad(40)) @ _rot("x", np.deg2rad(170))
    assert not np.allclose(R_down, R_down.T), "테스트가 무의미하다 — 대칭 회전"
    assert abs(np.linalg.det(R_down) - 1.0) < 1e-9

    # 물체: 그리퍼 로컬 X방향으로 폭 0.04인 막대. 월드로 옮겨 놓는다.
    origin = np.array([0.35, 0.0, 0.40 + 0.175])
    local_bar = np.zeros((200, 3))
    local_bar[:, 0] = np.linspace(-0.02, 0.02, 200)
    local_bar[:, 2] = 0.175
    bar = local_bar @ R_down.T + origin  # local → world

    g = _pose(R_down, origin)
    w = compute_grasp_width(g, bar, sweep_ext, sweep_off)
    assert w is not None, "스윕볼륨 안에 있는데 None — 전치 방향 의심"
    assert abs(w - (0.04 + CLEARANCE_M)) < 1e-6, w

    # 전치를 뒤집으면 실패해야 한다 (테스트가 실제로 판별력이 있는지 확인)
    bad = local_bar @ R_down + origin  # 일부러 틀린 변환
    assert compute_grasp_width(g, bad, sweep_ext, sweep_off) != w

    # --- 개구 한계 초과 거부 ---
    wide_local = local_bar.copy()
    wide_local[:, 0] = np.linspace(-0.08, 0.08, 200)  # 폭 0.16 > 0.102
    wide = wide_local @ R_down.T + origin
    assert compute_grasp_width(g, wide, sweep_ext, sweep_off) is None, (
        "손가락 폭 초과 물체가 통과했다 — x 클리핑 가드 확인"
    )

    # --- 스윕볼륨 밖(헛집음) 거부 ---
    far_local = local_bar.copy()
    far_local[:, 2] = -0.10
    far = far_local @ R_down.T + origin
    assert compute_grasp_width(g, far, sweep_ext, sweep_off) is None

    # --- TCP 오프셋 부호 ---
    tcp = grasp_to_tcp(g)
    assert tcp[2, 3] < g[2, 3], "TCP 부호가 뒤집혔다 — 로봇이 물체를 뚫는다"
    assert abs(np.linalg.norm(tcp[:3, 3] - g[:3, 3]) - TCP_OFFSET_M) < 1e-9

    # --- 캐스케이드 ---
    R_up = _rot("z", np.deg2rad(40))  # 위를 향함 → 1단계 탈락
    grasps = np.stack([g, _pose(R_up, origin), g])
    conf = np.array([0.9, 0.95, 0.1])  # 3번째는 신뢰도 탈락
    picked = select_grasps(grasps, conf, bar, sweep_ext, sweep_off, scene_pc=None)
    assert len(picked) == 1, f"기대 1개, 실제 {len(picked)}"
    assert abs(picked[0].confidence - 0.9) < 1e-9
    # 폭 0.048 m = 48 mm = 480 (1/10 mm 단위)
    assert picked[0].rgwd == 480, picked[0].rgwd

    # --- 도달 범위 밖 거부 ---
    far_origin = origin + np.array([5.0, 0, 0])
    assert (
        select_grasps(
            _pose(R_down, far_origin)[None],
            np.array([0.9]),
            local_bar @ R_down.T + far_origin,
            sweep_ext,
            sweep_off,
            scene_pc=None,
        )
        == []
    ), "도달 불가 grasp가 통과했다"

    # --- 충돌 검사 인자 누락 시 조용히 넘어가지 않아야 한다 ---
    try:
        select_grasps(
            grasps, conf, bar, sweep_ext, sweep_off, scene_pc=np.zeros((10, 3))
        )
        raise AssertionError("그리퍼 형상 없이 충돌검사가 통과했다")
    except ValueError:
        pass

    print("self-check PASS")


if __name__ == "__main__":
    _demo()
