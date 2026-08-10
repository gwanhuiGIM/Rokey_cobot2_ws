"""상태 정의와 허용 전이표.

src/PACKAGES.md#pick_fsm §1(상태) 의 stateDiagram 을 코드로 옮긴 것이다.
(옛 설계 문서는 2026-08-08 ws-cleanup 으로 삭제됐다 — PACKAGES.md#pick_fsm 이 단일 출처.)
**전이표를 여기 한 곳에만 둔다** — task_manager 안에 흩뿌리면 다이어그램과 코드가 조용히 갈라진다.

문서와 다른 점 2가지 (의도적):
  1. `WAIT_APPROVAL` 을 별도 상태로 뺐다. 문서에서는 `PLAN --> APPROACH : 사용자 승인 ✋` 라는
     전이 라벨이었는데, 승인 대기는 **시간이 흐르는 구간**이라 상태가 아니면 관측이 안 된다
     (`/pick/state` 를 보고 "지금 사람을 기다리는 중"인지 알 수 없다).
  2. 문서의 `APPROACH` 안쪽 서브상태(EXECUTING/MONITOR/STOP_REPLAN/WAITING)는 상태로 만들지 않았다.
     move_group 이 `planning_options.replan` 으로 그 루프를 이미 돌린다 (README "STOP_REPLAN" 절).
     FSM 은 그 바깥의 재시도만 센다 — 같은 일을 두 겹으로 구현하면 서로 싸운다.
"""

from enum import Enum, auto


class State(Enum):
    IDLE = auto()            # 아무것도 안 함. /pick/start 를 기다린다
    LISTENING = auto()       # get_keyword 호출 중 (음성 경로)
    PERCEIVE = auto()        # ComputeGrasp 호출 중 (촬영→세그→GPU→선택은 전부 저쪽)
    SCENE_PREP = auto()      # 대상을 CollisionObject 로 등록 + ACM 조정
    PLAN = auto()            # pre-grasp / grasp / lift 3점 IK
    NEXT_CANDIDATE = auto()  # alternatives 에서 다음 후보 꺼내기 (GPU 재호출 없음)
    WAIT_APPROVAL = auto()   # ✋ 사람이 /pick/approve 를 부를 때까지 정지
    STOW = auto()            # APPROACH 이동 전 그리퍼를 닫는다 — 이동 중 폭을 줄여 주변과의 충돌을 피한다
    APPROACH = auto()        # pre-grasp 로 이동 (실행 구간, 그리퍼는 닫힌 채)
    OPEN_GRIPPER = auto()    # pre-grasp 도착 후, 하강 전에 그리퍼를 연다
    DESCEND = auto()         # grasp pose 로 하강
    CLOSE = auto()           # 그리퍼 닫기
    VERIFY = auto()          # 잡혔는지 확인
    RELEASE_RETRY = auto()   # 놓치면 열고 재시도
    LIFT = auto()            # 들어올리기 (물체 attach 된 상태)
    PLACE = auto()           # 놓을 자세로 이동
    RELEASE = auto()         # 그리퍼 열기 + detach
    HOME = auto()            # 홈 복귀
    SPEAK_FAIL = auto()      # 실패 통보
    ABORT = auto()           # 중단 결정
    SAFE_STOP = auto()       # 정지 유지. /pick/reset 전까지 아무 명령도 안 낸다


#: 허용 전이. {현재 상태: {갈 수 있는 상태들}}
TRANSITIONS: dict[State, set[State]] = {
    State.IDLE:           {State.LISTENING, State.PERCEIVE},
    State.LISTENING:      {State.IDLE, State.PERCEIVE, State.SPEAK_FAIL, State.ABORT},
    State.PERCEIVE:       {State.SCENE_PREP, State.SPEAK_FAIL, State.ABORT},
    State.SCENE_PREP:     {State.PLAN, State.ABORT},
    State.PLAN:           {State.WAIT_APPROVAL, State.NEXT_CANDIDATE, State.ABORT},
    State.NEXT_CANDIDATE: {State.PLAN, State.SPEAK_FAIL, State.ABORT},
    State.WAIT_APPROVAL:  {State.STOW, State.ABORT},
    State.STOW:           {State.APPROACH, State.ABORT},
    State.APPROACH:       {State.OPEN_GRIPPER, State.NEXT_CANDIDATE, State.ABORT},
    State.OPEN_GRIPPER:   {State.DESCEND, State.ABORT},
    State.DESCEND:        {State.CLOSE, State.NEXT_CANDIDATE, State.ABORT},
    State.CLOSE:          {State.VERIFY, State.ABORT},
    State.VERIFY:         {State.LIFT, State.RELEASE_RETRY, State.ABORT},
    # RELEASE_RETRY, SAFE_STOP 모두 곧장 PERCEIVE/IDLE 로 가지 않고 HOME 을 거친다 —
    # 팔이 작업공간 박스 안(물체 높이)에 남은 채 재촬영하면 그리퍼 자신이 물체로
    # 오인식된다 (2026-08-07 발견: capture_graspgenx_scene.py 의 세그멘테이션은 팔을
    # XY 로 못 빼고 높이로만 구분한다). HOME 도착 후 어디로 갈지는
    # task_manager._home_next 가 정한다.
    State.RELEASE_RETRY:  {State.HOME, State.ABORT},
    State.LIFT:           {State.PLACE, State.ABORT},
    State.PLACE:          {State.RELEASE, State.ABORT},
    State.RELEASE:        {State.HOME, State.ABORT},
    State.HOME:           {State.IDLE, State.PERCEIVE, State.ABORT},
    State.SPEAK_FAIL:     {State.IDLE, State.LISTENING},
    State.ABORT:          {State.SAFE_STOP},
    State.SAFE_STOP:      {State.HOME},
}

#: 로봇이 실제로 움직일 수 있는 상태. ABORT 시 진행 중인 goal 을 취소해야 하는 구간이다.
MOTION_STATES = frozenset({State.APPROACH, State.DESCEND, State.LIFT, State.PLACE, State.HOME})

#: 물체를 물고 있을 수 있는 상태. 여기서 ABORT 가 나도 **그리퍼를 열지 않는다** —
#: 들고 있던 걸 떨어뜨리는 게 멈춰 있는 것보다 위험하다.
HOLDING_STATES = frozenset({State.VERIFY, State.LIFT, State.PLACE})


def is_allowed(src: State, dst: State) -> bool:
    """전이표에 있는 전이인가. 자기 자신으로의 전이(= 상태 유지)는 항상 허용."""
    return dst is src or dst in TRANSITIONS.get(src, set())
