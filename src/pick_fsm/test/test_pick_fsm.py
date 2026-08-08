"""로봇·move_group 없이 검증 가능한 부분만 테스트한다.

여기서 검증하는 것은 넷뿐이고, 전부 **틀리면 조용히 물체를 망가뜨리거나 문서가 거짓말을 하는** 것이다:
  1. 폭 → rgwd 단위 변환 (mm 로 잘못 바꾸면 10배 좁게 명령한다)
  2. grasp 포즈의 접근축 오프셋 (월드 -Z 로 물러나면 비스듬한 grasp 에서 물체를 친다)
  3. 상태 전이표
  4. README 의 mermaid stateDiagram ↔ 전이표 (다이어그램과 코드가 갈라지는 걸 막는다)

    colcon test --packages-select pick_fsm
    python3 -m pytest src/pick_fsm/test/test_pick_fsm.py -q      # ROS 없이도 됨
"""

import math
import re
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

from pick_fsm import geometry as geo
from pick_fsm.moveit_bridge import OCTOMAP_ACM_NAME, merge_acm
from pick_fsm.rg2 import (
    RG2_BRACKET_LENGTH_M,
    RG2_MAX_RGWD,
    fingertip_from_rg2_base_m,
    fingertip_length_m,
    width_to_rgwd,
)
from pick_fsm.states import TRANSITIONS, State, is_allowed


# ── 1. 단위 변환 ─────────────────────────────────────────
def test_width_to_rgwd_는_1_10mm_단위다():
    # 드라이버 주석: "must be provided in 1/10th millimeters"
    assert width_to_rgwd(0.048) == 480          # mm(48)로 바꾸면 10배 좁다 = 으깬다
    assert width_to_rgwd(0.110) == RG2_MAX_RGWD


def test_width_to_rgwd_는_유효범위로_클램프한다():
    assert width_to_rgwd(-1.0) == 0
    assert width_to_rgwd(5.0) == RG2_MAX_RGWD   # 범위를 넘겨 보내면 드라이버가 무시/오동작


def test_fingertip_length_m_은_실측_보정점을_그대로_재현한다():
    # 2026-08-07 실측 3점 (닫힘/70mm/100mm) — 보간 함수는 이 점들에서 정확히 맞아야 한다
    assert fingertip_length_m(0.000) == pytest.approx(0.240)
    assert fingertip_length_m(0.070) == pytest.approx(0.240 - 0.017)
    assert fingertip_length_m(0.100) == pytest.approx(0.240 - 0.041)


def test_fingertip_length_m_은_구간_내에서_선형보간한다():
    assert fingertip_length_m(0.035) == pytest.approx(0.240 - 0.017 / 2)


def test_fingertip_length_m_은_음수와_표_밖_폭도_처리한다():
    assert fingertip_length_m(-1.0) == fingertip_length_m(0.0)     # 음수는 0으로 클램프
    beyond = fingertip_length_m(0.110)                             # RG2 최대 폭, 외삽 구간
    assert beyond < fingertip_length_m(0.100)                      # 계속 짧아져야 한다(단조)
    assert 0.0 < beyond < 0.240


def test_rg2_base_기준_손끝은_플랜지_기준보다_브라켓만큼_짧다():
    # grasp 포즈 원점은 rg2_base_link 다. 플랜지 기준 240 mm 를 그대로 쓰면 22 mm 더 나간다.
    assert fingertip_from_rg2_base_m(0.0) == pytest.approx(0.218)
    assert (fingertip_length_m(0.07) - fingertip_from_rg2_base_m(0.07)
            == pytest.approx(RG2_BRACKET_LENGTH_M))


# ── 2. 포즈 연산 ─────────────────────────────────────────
def _pose(x=0.0, y=0.0, z=0.0, quat=(0.0, 0.0, 0.0, 1.0)) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
    (ps.pose.orientation.x, ps.pose.orientation.y,
     ps.pose.orientation.z, ps.pose.orientation.w) = quat
    return ps


def test_단위_쿼터니언의_축():
    p = _pose()
    assert geo.quat_axis_z(p.pose.orientation) == pytest.approx((0.0, 0.0, 1.0))
    assert geo.quat_axis_x(p.pose.orientation) == pytest.approx((1.0, 0.0, 0.0))


def test_pre_grasp_는_월드가_아니라_grasp_자신의_Z로_물러난다():
    # Y축 기준 90° 회전 → 로컬 +Z 가 월드 +X 를 향한다
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))
    assert geo.quat_axis_z(tilted.pose.orientation) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)

    pre = geo.pre_grasp(tilted, 0.10)
    # 월드 -Z(수직)로 물러났다면 z 가 0.2 가 됐을 것이다. 접근축 기준이면 x 가 0.4 다.
    assert pre.pose.position.x == pytest.approx(0.40)
    assert pre.pose.position.z == pytest.approx(0.30)
    # 자세는 건드리지 않는다 — 물러나면서 손목이 돌면 그건 다른 grasp 다
    assert pre.pose.orientation.y == pytest.approx(tilted.pose.orientation.y)


def test_lift_는_접근축이_아니라_월드_위쪽이다():
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))
    up = geo.lifted(tilted, 0.15)
    assert up.pose.position.z == pytest.approx(0.45)
    assert up.pose.position.x == pytest.approx(0.50)


def test_standoff_는_하강_종점과_lift_를_같이_민다():
    """LIFT 를 grasp 원점 기준으로 두면 물체를 문 채 접근축으로 도로 밀고 들어간다."""
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))     # 로컬 +Z 가 월드 +X
    poses = geo.plan_poses(tilted, approach_offset_m=0.10, standoff_m=0.025,
                           lift_offset_m=0.15)
    assert poses['pre_grasp'].pose.position.x == pytest.approx(0.40)
    # 종점은 grasp 원점(0.50)이 아니라 접근축으로 25 mm 앞
    assert poses['grasp'].pose.position.x == pytest.approx(0.475)
    # 🔴 lift 의 x 가 0.50 이면 물체를 문 채 25 mm 밀고 들어간 것이다
    assert poses['lift'].pose.position.x == pytest.approx(0.475)
    assert poses['lift'].pose.position.z == pytest.approx(0.45)


def test_standoff_는_approach_를_넘지_못하고_부호도_무시한다():
    """하강이 후진이 되는 설정을 만들 수 없어야 한다. 실기에서 튜닝하는 값이다."""
    assert geo.clamped_standoff(0.025, 0.10) == pytest.approx(0.025)
    assert geo.clamped_standoff(-0.025, 0.10) == pytest.approx(0.025)
    assert geo.clamped_standoff(0.30, 0.10) == pytest.approx(0.10)

    p = _pose(0.5, 0.0, 0.3)                                 # 접근축 = 월드 +Z
    poses = geo.plan_poses(p, approach_offset_m=0.10, standoff_m=0.30, lift_offset_m=0.15)
    # 클램프가 없으면 종점 z 가 pre_grasp(0.20)보다 위로 올라가 하강이 뒤집힌다
    assert poses['grasp'].pose.position.z == pytest.approx(poses['pre_grasp'].pose.position.z)


def test_standoff_0_이면_예전_동작_그대로다():
    p = _pose(0.5, 0.0, 0.3)
    poses = geo.plan_poses(p, approach_offset_m=0.10, standoff_m=0.0, lift_offset_m=0.15)
    assert poses['grasp'].pose.position.z == pytest.approx(0.30)
    assert poses['lift'].pose.position.z == pytest.approx(0.45)


def test_tcp_는_grasp_원점이_아니다():
    """접근이 기울면 그리퍼 base 는 물체에서 크게 벗어난다 (2026-08-05 오진 사례)."""
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))
    tcp = geo.tcp_of(tilted, 0.18)
    assert tcp == pytest.approx((0.68, 0.0, 0.3))       # 원점(0.5,0,0.3)에서 18 cm 떨어져 있다


def test_to_gripper_base_는_접근축을_보존하고_손가락만_90도_돌린다():
    """이게 깨지면 그리퍼가 물체를 폭이 아니라 긴 축으로 물거나, 90° 누워서 진입한다."""
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))    # 로컬 +Z 가 월드 +X
    out = geo.to_gripper_base(tilted)

    # ① 접근축(+Z)은 그대로여야 한다 — Z 축 둘레의 회전이므로
    assert geo.quat_axis_z(out.pose.orientation) == pytest.approx(
        geo.quat_axis_z(tilted.pose.orientation), abs=1e-9)
    # ② 손가락 방향(+X)은 원래의 +Y 로 간다 (로컬 요 +90°).
    #    Ry(90°) 에서 원래 +X 는 (0,0,-1), 원래 +Y 는 (0,1,0) 이다.
    assert geo.quat_axis_x(tilted.pose.orientation) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)
    assert geo.quat_axis_x(out.pose.orientation) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    # ③ 원점은 안 움직인다 — 두 프레임은 요만 다르다
    assert (out.pose.position.x, out.pose.position.y, out.pose.position.z) == pytest.approx(
        (0.5, 0.0, 0.3))


def test_to_gripper_base_는_단위쿼터니언에서_정확히_요_90도다():
    out = geo.to_gripper_base(_pose())
    assert geo.quat_axis_x(out.pose.orientation) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    assert geo.quat_axis_z(out.pose.orientation) == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)


def test_deg2rad():
    assert geo.deg2rad([0, 90, -30]) == pytest.approx([0.0, math.pi / 2, -math.pi / 6])


# ── 3. 상태 전이표 ───────────────────────────────────────
def test_모든_상태가_전이표에_있다():
    missing = [s.name for s in State if s not in TRANSITIONS]
    assert not missing, f'전이표에 없는 상태: {missing}'


def test_전이표의_목적지도_전부_정의된_상태다():
    for src, dsts in TRANSITIONS.items():
        for dst in dsts:
            assert isinstance(dst, State), f'{src} -> {dst}'


def test_문서의_주요_경로가_허용된다():
    path = [State.IDLE, State.LISTENING, State.PERCEIVE, State.SCENE_PREP, State.PLAN,
            State.WAIT_APPROVAL, State.STOW, State.APPROACH, State.OPEN_GRIPPER, State.DESCEND,
            State.CLOSE, State.VERIFY,
            State.LIFT, State.PLACE, State.RELEASE, State.HOME, State.IDLE]
    for a, b in zip(path, path[1:]):
        assert is_allowed(a, b), f'{a.name} -> {b.name} 가 막혀 있다'


def test_실패_경로도_허용된다():
    assert is_allowed(State.PLAN, State.NEXT_CANDIDATE)
    assert is_allowed(State.NEXT_CANDIDATE, State.PLAN)
    assert is_allowed(State.NEXT_CANDIDATE, State.SPEAK_FAIL)
    assert is_allowed(State.VERIFY, State.RELEASE_RETRY)
    # RELEASE_RETRY/SAFE_STOP 은 곧장 PERCEIVE/IDLE 로 가지 않고 HOME 을 거친다 —
    # 팔이 물체 높이에 남은 채 재촬영하면 그리퍼가 물체로 오인식된다(2026-08-07).
    assert is_allowed(State.RELEASE_RETRY, State.HOME)
    assert is_allowed(State.HOME, State.PERCEIVE)
    assert is_allowed(State.ABORT, State.SAFE_STOP)
    assert is_allowed(State.SAFE_STOP, State.HOME)
    assert is_allowed(State.HOME, State.IDLE)


def test_지름길은_막혀_있다():
    # 승인을 건너뛰고 바로 실행하는 전이가 있으면 안 된다
    assert not is_allowed(State.PLAN, State.APPROACH)
    # SAFE_STOP 에서 곧바로 동작 상태로 나가면 안 된다 — 사람 리셋을 거쳐야 한다
    assert not is_allowed(State.SAFE_STOP, State.APPROACH)
    # 인식 없이 계획으로 갈 수 없다
    assert not is_allowed(State.IDLE, State.PLAN)


def test_자기_자신으로의_전이는_상태_유지다():
    assert is_allowed(State.PERCEIVE, State.PERCEIVE)


# ── 3-1. README 다이어그램 ↔ 전이표 ──────────────────────
# states.py 가 경고하는 그 실패("다이어그램과 코드가 조용히 갈라진다")를 여기서 막는다.
_MERMAID = re.compile(r'```mermaid\n(.*?)```', re.S)
_EDGE = re.compile(r'^\s*(\w+)\s*-->\s*(\w+)\s*(?::.*)?$')

#: 전이표에 있지만 **일부러** 안 그린 전이. 이유 없이 여기 추가하지 말 것.
#: 모든 `* -> ABORT` 는 16개라 그리면 못 읽는다 → 다이어그램의 note 로 묶었다 (아래 테스트가 예외처리).
_UNDRAWN = {
    # 표에는 있지만 _st_listening 이 실제로 쓰지 않는 경로 (README 에 각주로 적어뒀다)
    (State.LISTENING, State.IDLE),
}


def _readme_state_edges() -> set:
    readme = Path(__file__).resolve().parent.parent / 'README.md'
    if not readme.exists():                     # 설치 트리에서 돌 때는 README 가 없다
        pytest.skip(f'README 없음: {readme}')
    for block in _MERMAID.findall(readme.read_text(encoding='utf-8')):
        if 'stateDiagram' not in block:
            continue
        edges, unknown = set(), []
        for line in block.splitlines():
            m = _EDGE.match(line)
            if m is None:                       # `[*] --> IDLE`, note 본문, 노드 선언
                continue
            src, dst = m.groups()
            for name in (src, dst):
                if name not in State.__members__:
                    unknown.append(name)
            if not unknown:
                edges.add((State[src], State[dst]))
        assert not unknown, f'README 다이어그램에 없는 상태 이름: {unknown}'
        return edges
    pytest.fail('README 에 stateDiagram mermaid 블록이 없다')


def test_README_다이어그램의_전이가_전부_전이표에_있다():
    bogus = [(s.name, d.name) for s, d in _readme_state_edges() if not is_allowed(s, d)]
    assert not bogus, f'그림에만 있고 TRANSITIONS 에 없는 전이: {bogus}'


def test_전이표의_전이가_전부_README_에_그려져_있다():
    drawn = _readme_state_edges()
    missing = [(s.name, d.name)
               for s, dsts in TRANSITIONS.items() for d in dsts
               if d is not State.ABORT and (s, d) not in drawn and (s, d) not in _UNDRAWN]
    assert not missing, f'전이표에는 있는데 그림에 없는 전이: {missing}'


# ── 4. ACM 병합 ──────────────────────────────────────────
# 🔴 2026-08-06 실측: PlanningScene 의 allowed_collision_matrix 는 is_diff 여도
#    **전체 교체**다. 내 행렬만 보냈더니 SRDF 의 disable_collisions 34개가 날아가
#    인접 링크가 자기충돌로 잡히고 IK 가 모든 포즈에서 NO_IK_SOLUTION 을 냈다.
#    아래 테스트가 그 회귀를 막는다.
def _acm(names, pairs=()):
    m = AllowedCollisionMatrix()
    m.entry_names = list(names)
    idx = {n: i for i, n in enumerate(names)}
    m.entry_values = [AllowedCollisionEntry(enabled=[False] * len(names)) for _ in names]
    for a, b in pairs:
        m.entry_values[idx[a]].enabled[idx[b]] = True
        m.entry_values[idx[b]].enabled[idx[a]] = True
    return m


def _enabled(m, a, b):
    idx = {n: i for i, n in enumerate(m.entry_names)}
    return m.entry_values[idx[a]].enabled[idx[b]]


def test_merge_acm_은_기존_항목을_보존한다():
    """SRDF 에서 온 disable_collisions 가 살아남아야 한다 — 이게 핵심이다."""
    cur = _acm(['link_6', 'tool0', 'rg2_base_link'],
               pairs=[('link_6', 'rg2_base_link'), ('tool0', 'rg2_base_link')])
    out = merge_acm(cur, 'pick_target', ['rg2_base_link'])
    assert _enabled(out, 'link_6', 'rg2_base_link')      # 날아가면 자기충돌로 IK 가 죽는다
    assert _enabled(out, 'tool0', 'rg2_base_link')


def test_merge_acm_이_대상물체를_추가하고_대칭이다():
    cur = _acm(['link_6', 'rg2_base_link'])
    out = merge_acm(cur, 'pick_target', ['rg2_base_link'])
    assert 'pick_target' in out.entry_names
    assert _enabled(out, 'rg2_base_link', 'pick_target')
    assert _enabled(out, 'pick_target', 'rg2_base_link')
    # 그리퍼가 아닌 링크는 물체와 부딪히면 안 된다 (허용하면 팔뚝으로 물체를 밀고 간다)
    assert not _enabled(out, 'link_6', 'pick_target')


def test_merge_acm_은_행렬을_정사각으로_유지한다():
    cur = _acm(['a', 'b'])
    out = merge_acm(cur, 'obj', ['a', 'c'])          # 'c' 는 기존에 없던 이름
    n = len(out.entry_names)
    assert all(len(r.enabled) == n for r in out.entry_values)


def test_merge_acm_의_octomap_은_옵션이다():
    cur = _acm(['rg2_base_link'])
    off = merge_acm(cur, 'obj', ['rg2_base_link'], allow_octomap=False)
    assert OCTOMAP_ACM_NAME not in off.entry_names   # 기본값에서 켜지면 사람 팔이 안 보인다
    on = merge_acm(cur, 'obj', ['rg2_base_link'], allow_octomap=True)
    assert _enabled(on, 'rg2_base_link', OCTOMAP_ACM_NAME)


def test_merge_acm_은_입력을_변형하지_않는다():
    cur = _acm(['a'])
    merge_acm(cur, 'obj', ['a'])
    assert cur.entry_names == ['a']                  # 원본을 건드리면 재시도 때 누적된다
    assert len(cur.entry_values[0].enabled) == 1
