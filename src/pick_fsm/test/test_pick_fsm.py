"""로봇·move_group 없이 검증 가능한 부분만 테스트한다.

여기서 검증하는 것은 셋뿐이고, 전부 **틀리면 조용히 물체를 망가뜨리는** 계산이다:
  1. 폭 → rgwd 단위 변환 (mm 로 잘못 바꾸면 10배 좁게 명령한다)
  2. grasp 포즈의 접근축 오프셋 (월드 -Z 로 물러나면 비스듬한 grasp 에서 물체를 친다)
  3. 상태 전이표 (다이어그램과 코드가 갈라지는 걸 막는다)

    colcon test --packages-select pick_fsm
    python3 -m pytest src/pick_fsm/test/test_pick_fsm.py -q      # ROS 없이도 됨
"""

import math

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

from pick_fsm import geometry as geo
from pick_fsm.moveit_bridge import OCTOMAP_ACM_NAME, merge_acm
from pick_fsm.rg2 import RG2_MAX_RGWD, width_to_rgwd
from pick_fsm.states import TRANSITIONS, State, is_allowed


# ── 1. 단위 변환 ─────────────────────────────────────────
def test_width_to_rgwd_는_1_10mm_단위다():
    # 드라이버 주석: "must be provided in 1/10th millimeters"
    assert width_to_rgwd(0.048) == 480          # mm(48)로 바꾸면 10배 좁다 = 으깬다
    assert width_to_rgwd(0.110) == RG2_MAX_RGWD


def test_width_to_rgwd_는_유효범위로_클램프한다():
    assert width_to_rgwd(-1.0) == 0
    assert width_to_rgwd(5.0) == RG2_MAX_RGWD   # 범위를 넘겨 보내면 드라이버가 무시/오동작


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


def test_tcp_는_grasp_원점이_아니다():
    """접근이 기울면 그리퍼 base 는 물체에서 크게 벗어난다 (2026-08-05 오진 사례)."""
    s = math.sin(math.pi / 4)
    tilted = _pose(0.5, 0.0, 0.3, quat=(0.0, s, 0.0, s))
    tcp = geo.tcp_of(tilted, 0.18)
    assert tcp == pytest.approx((0.68, 0.0, 0.3))       # 원점(0.5,0,0.3)에서 18 cm 떨어져 있다


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
            State.WAIT_APPROVAL, State.APPROACH, State.DESCEND, State.CLOSE, State.VERIFY,
            State.LIFT, State.PLACE, State.RELEASE, State.HOME, State.IDLE]
    for a, b in zip(path, path[1:]):
        assert is_allowed(a, b), f'{a.name} -> {b.name} 가 막혀 있다'


def test_실패_경로도_허용된다():
    assert is_allowed(State.PLAN, State.NEXT_CANDIDATE)
    assert is_allowed(State.NEXT_CANDIDATE, State.PLAN)
    assert is_allowed(State.NEXT_CANDIDATE, State.SPEAK_FAIL)
    assert is_allowed(State.VERIFY, State.RELEASE_RETRY)
    assert is_allowed(State.RELEASE_RETRY, State.PERCEIVE)
    assert is_allowed(State.ABORT, State.SAFE_STOP)
    assert is_allowed(State.SAFE_STOP, State.IDLE)


def test_지름길은_막혀_있다():
    # 승인을 건너뛰고 바로 실행하는 전이가 있으면 안 된다
    assert not is_allowed(State.PLAN, State.APPROACH)
    # SAFE_STOP 에서 곧바로 동작 상태로 나가면 안 된다 — 사람 리셋을 거쳐야 한다
    assert not is_allowed(State.SAFE_STOP, State.APPROACH)
    # 인식 없이 계획으로 갈 수 없다
    assert not is_allowed(State.IDLE, State.PLAN)


def test_자기_자신으로의_전이는_상태_유지다():
    assert is_allowed(State.PERCEIVE, State.PERCEIVE)


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
