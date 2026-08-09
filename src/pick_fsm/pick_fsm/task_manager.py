#!/usr/bin/env python3
"""음성 지시 pick 상태머신.

    ros2 launch pick_fsm pick_fsm.launch.py
    ros2 service call /pick/start   std_srvs/srv/Trigger {}
    ros2 service call /pick/approve std_srvs/srv/Trigger {}     # ✋ 실행 승인

설계 출처: md/voice-pick-statemachine.md (§1 노드 그래프 · §2 계약 · §3 상태머신)

이 노드가 존재하는 이유는 하나다: **로봇 명령 경로가 두 개 살아 있기 때문이다.**
`dsr_controller2`(서비스 movej/movel)와 `dsr_moveit_controller`(JTC)에 동시에 명령하면
안 된다. 그 배타권을 한 노드가 소유해야 하고, 이 노드가 그 자리다.
→ 그래서 이 노드는 **MoveIt 경로만** 쓴다. DSR_ROBOT2 의 movej/movel 을 부르지 않는다.

⚠️ **이 노드는 항상 실행한다.** `dry_run`(plan_only) 파라미터는 2026-08-09 제거했다 —
   실기 모션 데이터 수집 단계로 넘어갔고, 팔이 안 움직이는데 그리퍼만 실제로 개폐되는
   반쪽 안전(`_move()`만 게이트되고 `rg2.*`는 안 됨)이 오히려 오해를 낳았다.
   남은 안전장치는 `require_approval:=true`(기본값)와 **물리 비상정지 버튼**이다.
"""

import threading

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from pick_fsm import geometry as geo
from pick_fsm.moveit_bridge import SUCCESS, MoveItBridge, err_name, merge_acm
from pick_fsm.rg2 import (
    RG2_MODEL_WIDTH_M, Rg2Client, fingertip_from_rg2_base_m, grip_target_width_m,
)
from pick_fsm.robot_safety_node import UNSAFE_STATES
from pick_fsm.states import HOLDING_STATES, State, is_allowed

try:                                    # pick_fsm_msgs 가 없어도 legacy/manual 경로는 돌게 한다
    from pick_fsm_msgs.srv import ComputeGrasp
except ImportError:                     # pragma: no cover
    ComputeGrasp = None

#: 상태별 제한시간 [s]. 넘으면 ABORT. 사람 입력을 기다리는 상태는 여기 없다.
DEFAULT_TIMEOUTS = {
    State.LISTENING: 60.0,
    State.PERCEIVE: 120.0,      # GPU 추론 + 모델 로딩(첫 호출 수십 초)
    State.SCENE_PREP: 10.0,
    State.PLAN: 30.0,
    State.STOW: 20.0,
    State.APPROACH: 180.0,      # replan 루프가 도는 구간이라 넉넉히
    State.OPEN_GRIPPER: 20.0,
    State.DESCEND: 120.0,
    State.CLOSE: 20.0,
    State.VERIFY: 10.0,
    State.RELEASE_RETRY: 20.0,
    State.LIFT: 120.0,
    State.PLACE: 180.0,
    State.RELEASE: 20.0,
    State.HOME: 180.0,
}

#: SPEAK_FAIL -> LISTENING 을 몇 번 연속으로 돌면 IDLE 로 내려앉는지.
#: 이 왕복은 매번 `_to()` 가 `_entered` 를 리셋해서 LISTENING 제한시간이 영원히 안 걸린다 —
#: 멈추는 조건을 따로 두지 않으면 tick 주기로 무한히 돈다(2026-08-07 실기 로그 폭주).
MAX_FAIL_STREAK = 3

#: `/pick/target`(지시) · `/pick/target_active`(현재값) 용.
#: TRANSIENT_LOCAL 이라야 **늦게 뜨는 쪽**이 마지막 값을 받는다 — rqt 패널은 FSM 과 따로
#: 껐다 켜므로, VOLATILE 이면 패널을 다시 띄울 때마다 타겟 표시가 비어 보이고 사람이
#: "타겟이 풀렸나?" 하고 다시 누르게 된다. 양쪽 다 이 프로파일을 써야 한다(durability 가
#: 어긋나면 아예 연결이 안 된다) — 그래서 여기 한 곳에 두고 rqt_panel 이 import 한다.
TARGET_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

#: `/pick/place_location` 로 고를 수 있는 값 -> 실제로 쓸 joint 파라미터 이름.
#: '내려놓는 위치' 세 경우(장바구니/작업테이블 지정 자리/작업테이블 바깥 폐기)를
#: 관절각 프리셋 세 개로 나눈다 — grasp pose 처럼 좌표를 계산하지 않고 home/place 와
#: 같은 고정 관절이동이라 IK 가 필요 없다(_joint_move 재사용).
PLACE_LOCATIONS = {
    'basket': 'place_joints_deg',
    'table': 'place_table_joints_deg',
    'discard': 'place_discard_joints_deg',
}


def str_param(name: str, value: str) -> Parameter:
    """rcl_interfaces 문자열 파라미터 하나. `SetParameters` 요청에 넣는다."""
    return Parameter(name=name,
                     value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                          string_value=str(value)))


#: 파라미터 기본값 = 타입의 정본. `config/pick_fsm.yaml` 의 값은 여기 적힌 타입과
#: 같아야 한다 (float 는 `0` 이 아니라 `0.0`). test_pick_fsm.py 가 이 대조를 자동화한다.
PARAM_DEFAULTS = {
    # 안전
    # `dry_run`(plan_only) 은 없다 — 2026-08-09 제거. 모듈 docstring 참고.
    'require_approval': True,        # false 로 두면 사람 승인 없이 실행한다
    'approval_timeout_sec': 300.0,

    # MoveIt
    'planning_group': 'manipulator',
    # 🔴 `tool0` 이 아니다 (2026-08-07 정정). tool0 의 접근축은 +Z 가 아니라 +X 라
    #    (`onrobot_rg2.xacro:40` rpy="1.5708 0 1.5708"), grasp 포즈를 tool0 에
    #    그대로 걸면 그리퍼가 90° 누운 채 진입한다. `rg2_base_link` 는 GraspGenX 의
    #    그리퍼 base 와 같은 프레임이라 브라켓 22 mm 오프셋도 같이 해소된다.
    #    MoveIt 은 solver tip(tool0)에 고정조인트로 붙은 링크를 ik_link 로 받는다.
    'ee_link': 'rg2_base_link',
    'base_frame': 'base_link',       # ⚠️ world 아님. planning scene 이 world 를 모른다
    'joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
    'vel_scale': 0.1,                # 실기 첫 시도는 느리게
    'acc_scale': 0.1,
    'planning_time': 5.0,
    'planning_attempts': 10,
    'joint_tolerance': 0.001,
    'ik_timeout_sec': 0.2,
    'ik_avoid_collisions': True,
    # 'ompl' | 'isaac_ros_cumotion' (scripts/bench_planning_time.py 와 같은 이름).
    # IK 는 이 값과 무관 — move_group 의 GetPositionIK 는 파이프라인을 안 탄다.
    'planning_pipeline': 'ompl',
    'planner_id': '',
    'replan': True,                  # 실행 중 씬이 바뀌면 move_group 이 다시 계획한다
    'replan_attempts': 3,
    'replan_delay': 0.5,
    'motion_retries': 2,             # move_group 실패 시 FSM 바깥 재시도 횟수
    # PERCEIVE(재)촬영 실패 시 재시도 횟수. 2026-08-09 실기 로그: 정지된 같은 물체·같은
    # 자리에서 collision-free 비율이 0%~53%로 요동쳤다(depth 노이즈로 OBB 후보 자체가
    # 흔들린다) — 재촬영 한 번으로 5번 중 4번은 살아났다. `motion_retries` 와 같은 뼈대.
    'perceive_retries': 2,

    # 자세
    'approach_offset_m': 0.10,       # pre-grasp: grasp 의 -Z 로 물러나는 거리
    'grasp_standoff_m': 0.0,         # DESCEND 종점을 grasp 의 -Z 로 덜 내리는 양
    'lift_offset_m': 0.15,           # LIFT: 월드 +Z
    # tcp_offset_m 은 더 이상 파라미터가 아니다 — rg2.fingertip_length_m(width_m)이
    # 2026-08-07 실측(폭에 따라 손끝이 짧아지는 비선형 보정표)으로 대체했다.
    'max_reach_m': 0.900,            # M0609 URDF 실측 (shoulder 기준)
    'home_joints_deg': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],     # robot_control JReady
    'place_joints_deg': [4.0, 38.0, 64.0, -0.1, 78.0, 4.0],  # robot_control BUCKET_POS ('basket')
    # UNVERIFIED: 아래 둘은 teach 된 적 없다 — home_joints_deg 를 임시로 복사해 둔 것뿐이다.
    # 실기에서 안전한 자세로 다시 잡기 전에는 'table'/'discard' 를 쓰지 말 것.
    'place_table_joints_deg': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
    'place_discard_joints_deg': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
    # PLACE_LOCATIONS 의 키 중 하나('basket'|'table'|'discard'). 런타임엔 /pick/place_location
    # (rqt 패널)이 이 값을 이긴다 — /pick/target 과 같은 패턴.
    'place_location': 'basket',

    # 씬
    'object_id': 'pick_target',
    'object_radius_m': 0.04,
    'clear_octomap_before_descend': False,
    'allow_gripper_octomap_collision': False,
    'gripper_links': ['rg2_base_link',
                      'rg2_left_outer_knuckle', 'rg2_left_inner_knuckle',
                      'rg2_left_inner_finger', 'rg2_right_outer_knuckle',
                      'rg2_right_inner_knuckle', 'rg2_right_inner_finger'],

    # 그리퍼
    'gripper_backend': 'real',       # real | virtual (숫자 명령의 의미가 다르다)
    'gripper_service': '/onrobot/sendCommand',
    'grip_detected_topic': '/onrobot/grip_detected',
    # 물체 폭에서 **빼는** 조임 여유 [m]. 목표 개구 = 물체 폭 - 이 값 (`_grip_width`).
    'grip_clearance_m': 0.008,       # UNVERIFIED: 실측 튜닝값. 도면값 아님
    'max_grip_width_m': RG2_MODEL_WIDTH_M,
    'force_down_steps': 0,           # 'd' 반복 횟수. 0 = 드라이버 기본(=40 N, RG2 최대)
    'gripper_settle_sec': 1.5,
    'verify_required': False,        # true 면 grip_detected 를 못 받았을 때도 실패 처리
    'grip_retries': 1,

    # 인식
    # `/grasp/compute_grasp`(pick_fsm_msgs/ComputeGrasp) 서버는 2026-08-09 에
    # grasp_bridge_node 에 생겼다 — 그전까지 이 ws 어디에도 없어서 기본값이 legacy_trigger 다.
    # 🔴 **폭(width_m)은 compute_grasp 경로로만 온다.** legacy_trigger 는 std_srvs/Trigger 라
    #    응답에 폭을 담을 필드가 없어 `default_width_m`(UNVERIFIED 상수)로 전부 때운다.
    #    물체마다 폭을 맞추려면 `grasp_source:=compute_grasp` 로 바꿔야 한다 — 단
    #    **실기 미검증이다**(폭 측정·조임 여유 부호 둘 다 2026-08-09 신규).
    'grasp_source': 'legacy_trigger',  # legacy_trigger | compute_grasp | manual
    'grasp_service': '/grasp/compute_grasp',
    'grasp_trigger_service': '/grasp/compute',
    'grasp_best_topic': '/grasp/best',
    'grasp_candidates_topic': '/grasp/candidates',
    'min_confidence': 0.5,
    'default_width_m': 0.06,         # legacy/manual 경로에는 폭 정보가 없다
    'max_alternatives': 5,

    # 인식 브리지 파라미터 푸시 (PERCEIVE 진입 때 1회)
    # 타겟의 정본은 **이 FSM** 이고, 브리지는 그 값을 받아 쓰는 쪽이다. 이 푸시가 없으면
    # FSM 의 target 과 브리지의 target_classes 가 각자 살아 어긋나도 아무도 모른다.
    'bridge_node': '/grasp_bridge_node',   # 비우면 푸시하지 않는다(브리지를 직접 설정할 때)
    # 이 ws 의 기본 파이프라인은 YOLO 세그다 — 클래스 이름으로 타겟을 고르려면 필수다.
    # `geometric` 은 클래스를 모르므로 타겟 지정이 불가능하다. 비우면 브리지 설정을 안 건드린다.
    'bridge_seg_source': 'yolo',           # yolo | geometric | '' (안 건드림)

    # 음성
    'voice_enabled': True,
    'keyword_service': '/get_keyword',
    # voice_enabled=false 일 때의 **초기** 타겟. 콤마로 여러 클래스도 된다('apple,orange').
    # 빈 문자열 = 자동(브리지가 본 것 전부에서 점수 최고). 런타임에는 `/pick/target`(String)
    # 이 이 값을 덮어쓴다 — rqt 패널의 타겟 상자가 그 토픽을 쏜다.
    'target': '',

    'tick_hz': 10.0,
}


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        p = self._declare_params()
        # ⚠️ 여기서 죽는 게 맞다 (vla_command_node.pixel_policy 검증과 같은 이유). 검증을
        # 안 하면 yaml 오타(`place_location: bakset`)가 조용히 통과해 rqt 패널엔 오타 그대로
        # 표시되면서 실제 이동은 _st_place 의 fallback 으로 basket 에 접힌다 — 표시값과
        # 실제 목적지가 갈라진다. `/pick/place_location`(토픽) 쪽은 _on_place_location 이
        # 따로 막는다(2026-08-10 cross-review 지적).
        if p['place_location'] not in PLACE_LOCATIONS:
            raise ValueError(
                f"place_location 파라미터 기본값이 잘못됐다: {p['place_location']!r} "
                f'(허용: {sorted(PLACE_LOCATIONS)})')
        cb = ReentrantCallbackGroup()

        self.moveit = MoveItBridge(self, cb, base_frame=p['base_frame'])
        self.rg2 = Rg2Client(self, cb, backend=p['gripper_backend'],
                             service=p['gripper_service'],
                             grip_topic=p['grip_detected_topic'])
        self.kw_cli = self.create_client(Trigger, p['keyword_service'], callback_group=cb)

        # ── grasp 공급원 ──────────────────────────────────
        # 문서의 정본 계약은 ComputeGrasp 다. 하지만 지금 실제로 도는 건
        # graspgenx_perception 의 grasp_bridge_node 가 내는 Trigger 경로다.
        # 둘 다 지원하지 않으면 이 FSM 은 "언젠가 돌 코드"가 된다.
        self.grasp_cli = None
        if p['grasp_source'] == 'compute_grasp':
            if ComputeGrasp is None:
                raise RuntimeError('grasp_source=compute_grasp 인데 pick_fsm_msgs import 실패')
            self.grasp_cli = self.create_client(ComputeGrasp, p['grasp_service'],
                                                callback_group=cb)
        elif p['grasp_source'] == 'legacy_trigger':
            self.grasp_cli = self.create_client(Trigger, p['grasp_trigger_service'],
                                                callback_group=cb)
        elif p['grasp_source'] != 'manual':
            raise ValueError(f"grasp_source 값이 이상하다: {p['grasp_source']!r}")

        # ── 브리지 파라미터 푸시 ──────────────────────────
        # 🔴 2026-08-09 실기: `pick_fsm.launch.py ... target_classes:=apple,orange,banana` 로
        #    띄웠는데 이 런치엔 그런 인자가 없어 **경고도 없이 무시**됐고, 정작 브리지에는
        #    이전 실행의 target_classes + seg_source=geometric 이 남아 있어서 매번
        #    "target_classes 는 seg_source='yolo' 에서만 쓴다" 로 실패했다.
        #    두 값이 각자 사는 한 같은 사고가 반복된다 → PERCEIVE 마다 여기서 밀어 넣는다.
        # manual 경로는 사람이 /grasp/best 를 직접 쏘므로 브리지를 안 건드린다.
        # 노드 이름은 **여기서 한 번만** 읽고 붙잡는다. 클라이언트는 기동 때의 값으로
        # 만들어지므로, 로그에서만 파라미터를 다시 읽으면 런타임에 그 값이
        # 바뀌었을 때 "실제로 설정한 노드"와 "메시지가 말하는 노드"가 갈라진다 —
        # 이번에 고친 사고(정본이 두 군데)와 같은 부류라 여기서 막는다.
        self._bridge_name = p['bridge_node']
        self.bridge_param_cli = None
        if self._bridge_name and p['grasp_source'] != 'manual':
            self.bridge_param_cli = self.create_client(
                SetParameters, self._bridge_name.rstrip('/') + '/set_parameters',
                callback_group=cb)

        self._best = None
        self._best_seq = 0
        self._seq_at_call = 0
        self._candidates = []
        self.create_subscription(PoseStamped, p['grasp_best_topic'], self._on_best, 10,
                                 callback_group=cb)
        self.create_subscription(PoseArray, p['grasp_candidates_topic'], self._on_candidates,
                                 10, callback_group=cb)
        # robot_safety_node 가 별도 프로세스로 Doosan 로봇상태를 폴링해 발행한다
        # (충돌 등으로 SAFE_STOP/EMERGENCY_STOP 에 들어가면 여기가 값을 받는다).
        # 이 노드는 안 떠 있어도 된다 — 그러면 그냥 이 감시 기능만 빠진다.
        self.create_subscription(Int8, '/pick/robot_state_code', self._on_robot_state, 10,
                                 callback_group=cb)

        # ── 관측·조작 인터페이스 ───────────────────────────
        self.state_pub = self.create_publisher(String, '/pick/state', 10)
        # 타겟은 세 군데서 들어온다: 파라미터(초기값) · 음성(LISTENING) · `/pick/target`(사람).
        # `_active` 는 그 결과를 되돌려주는 표시용이다 — 지시와 현재값을 같은 토픽에 섞으면
        # 패널이 자기가 쏜 값을 다시 받아 되먹임이 된다.
        self.target_pub = self.create_publisher(String, '/pick/target_active', TARGET_QOS)
        self.create_subscription(String, '/pick/target', self._on_target, TARGET_QOS,
                                 callback_group=cb)
        # 내려놓을 위치 — basket(장바구니)/table(작업테이블 지정 자리)/discard(테이블 밖 폐기).
        # /pick/target 과 똑같은 패턴: 사람이 못 바꾸면 파라미터 기본값을 쓴다.
        self.place_pub = self.create_publisher(String, '/pick/place_location_active', TARGET_QOS)
        self.create_subscription(String, '/pick/place_location', self._on_place_location,
                                 TARGET_QOS, callback_group=cb)
        self.create_service(Trigger, '/pick/start', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/pick/approve', self._srv_approve, callback_group=cb)
        self.create_service(Trigger, '/pick/abort', self._srv_abort, callback_group=cb)
        self.create_service(Trigger, '/pick/reset', self._srv_reset, callback_group=cb)

        # ── FSM 내부 상태 ─────────────────────────────────
        self.state = State.IDLE
        self._entered = self.get_clock().now()
        self._fut = None            # 진행 중 서비스 future
        self._call = None           # 진행 중 액션 (ActionCall)
        self._extra = []            # 결과를 기다릴 필요 없는 부수 future (그리퍼 명령 등)
        self._start_req = False
        self._approved = False
        self._abort_req = None      # 사유 문자열
        # _abort_req 는 /pick/abort(서비스 콜백)와 _on_robot_state(구독 콜백) 양쪽에서
        # 쓰고 _tick(타이머 콜백)이 읽어서 비운다 — 셋 다 ReentrantCallbackGroup 이라 다른
        # 스레드에서 동시에 돌 수 있다. 락 없이 두면 robot_state 트리거가 조용히
        # 유실될 수 있다(2026-08-07 cross-review 지적).
        self._abort_lock = threading.Lock()
        self._octomap_cleared = False
        self._acm = None
        self.target = ''
        self._target_override = None   # `/pick/target` 로 들어온 값. None = 파라미터를 쓴다
        self.place_location = ''
        self._place_override = None    # `/pick/place_location` 로 들어온 값. None = 파라미터를 쓴다
        self._push_fut = None          # 브리지 SetParameters future (_fut 과 겹치면 안 된다)
        self._pushed = False           # 이번 PERCEIVE 에서 푸시를 끝냈는지
        self.grasp = None           # PoseStamped, ee_link 목표 자세
        self.width_m = 0.0
        self.alternatives = []
        self.alternative_widths = []   # alternatives 와 1:1. _st_next_candidate 가 같이 pop 한다
        self.poses = {}             # 'pre_grasp'|'grasp'|'lift' -> PoseStamped
        self.solutions = {}         # 같은 키 -> JointState
        self._plan_i = 0
        self._retry_motion = 0
        self._retry_grip = 0
        self._retry_perceive = 0
        self._fail_streak = 0       # SPEAK_FAIL 연속 횟수. _st_idle 이 start 마다 0 으로 되돌린다
        self._object_added = False
        self._nag = 0
        self._home_next = State.IDLE   # HOME 도착 후 갈 곳. _srv_reset/_st_release_retry 가 덮어쓴다

        self.timer = self.create_timer(1.0 / p['tick_hz'], self._tick, callback_group=cb)
        self._publish_target(p['target'])
        self._publish_place(p['place_location'])
        self.get_logger().info(
            f"준비됨 — require_approval={p['require_approval']}, "
            f"grasp_source={p['grasp_source']}, gripper_backend={p['gripper_backend']}")
        self.get_logger().info(
            f"타겟='{p['target'] or '(자동)'}' — 바꾸려면 /pick/target (rqt 패널의 '타겟' 상자). "
            + (f"PERCEIVE 마다 {p['bridge_node']} 에 target_classes"
               + (f"+seg_source={p['bridge_seg_source']}" if p['bridge_seg_source'] else '')
               + ' 를 밀어 넣는다'
               if self.bridge_param_cli is not None else '브리지 푸시 없음(bridge_node 비어 있음)'))
        self.get_logger().warn(
            '⚠️ 계획만 하는 모드는 없다 — 로봇이 실제로 움직인다. 비상정지 버튼을 손에 둘 것')

    # ────────────────────────────────────────────────────────
    # 파라미터
    # ────────────────────────────────────────────────────────
    def _declare_params(self):
        for k, v in PARAM_DEFAULTS.items():
            self.declare_parameter(k, v)
        return {k: self.get_parameter(k).value for k in PARAM_DEFAULTS}

    def p(self, key):
        return self.get_parameter(key).value

    # ────────────────────────────────────────────────────────
    # 구독·서비스 콜백
    # ────────────────────────────────────────────────────────
    def _on_best(self, msg):
        self._best = msg
        self._best_seq += 1

    def _on_candidates(self, msg):
        self._candidates = [(msg.header, pose) for pose in msg.poses]

    def _on_robot_state(self, msg):
        """충돌 등으로 로봇이 자체적으로 안전정지에 들어가면 하던 작업을 즉시 ABORT.

        IDLE/ABORT/SAFE_STOP/SPEAK_FAIL 에서는 중단할 작업이 없으니 다시 안 건드린다 —
        안 그러면 이미 SAFE_STOP 인데 폴링될 때마다 로그만 쌓인다.
        """
        if int(msg.data) not in UNSAFE_STATES:
            return
        if self.state in (State.IDLE, State.ABORT, State.SAFE_STOP, State.SPEAK_FAIL):
            return
        with self._abort_lock:
            self._abort_req = f'로봇 안전정지 감지 (robot_state={int(msg.data)})'

    def _on_target(self, msg):
        """사람이 잡을 대상을 지정한다. 빈 문자열 = 자동(브리지가 본 것 중 점수 최고).

        콤마로 여러 개도 된다('apple,orange') — 그러면 그 셋 중 점수 최고를 잡는다.
        ⚠️ **진행 중인 작업에는 적용하지 않는다.** PERCEIVE 는 진입할 때 이 값을 브리지에
        밀어넣고 시작하므로, 도중에 바꾸면 로그가 가리키는 대상과 실제로 계산된 대상이
        갈라진다. 다음 `/pick/start` 부터 쓴다.
        """
        self._target_override = msg.data.strip()
        shown = self._target_override or '(자동)'
        if self.state is State.IDLE:
            self.get_logger().info(f'타겟 지정: {shown}')
        else:
            self.get_logger().warn(
                f'타겟 지정 {shown} — 진행 중인 {self.state.name} 에는 적용하지 않는다. '
                '다음 /pick/start 부터다')
        self._publish_target(self._target_override)

    def _publish_target(self, value: str):
        self.target_pub.publish(String(data=str(value)))

    def _on_place_location(self, msg):
        """내려놓을 위치 지정. `PLACE_LOCATIONS` 키가 아니면 무시하고 이전 값을 유지한다.

        ⚠️ target 과 같은 이유로 **진행 중인 작업에는 적용하지 않는다** — PICK 도중에
        바뀌면 로그가 가리키는 목적지와 실제 PLACE 관절이 갈라진다. 다음 `/pick/start`부터.
        """
        value = msg.data.strip()
        if value not in PLACE_LOCATIONS:
            self.get_logger().warn(
                f"잘못된 place_location '{value}' — {list(PLACE_LOCATIONS)} 중 하나만 된다. 무시함")
            return
        self._place_override = value
        if self.state is State.IDLE:
            self.get_logger().info(f'내려놓을 위치 지정: {value}')
        else:
            self.get_logger().warn(
                f'내려놓을 위치 지정 {value} — 진행 중인 {self.state.name} 에는 적용하지 않는다. '
                '다음 /pick/start 부터다')
        self._publish_place(self._place_override)

    def _publish_place(self, value: str):
        self.place_pub.publish(String(data=str(value)))

    def _srv_start(self, _req, res):
        if self.state is not State.IDLE:
            res.success, res.message = False, f'IDLE 이 아니다 (현재 {self.state.name})'
            return res
        self._start_req = True
        res.success, res.message = True, '시작'
        return res

    def _srv_approve(self, _req, res):
        if self.state is not State.WAIT_APPROVAL:
            res.success, res.message = False, f'승인 대기 중이 아니다 (현재 {self.state.name})'
            return res
        self._approved = True
        res.success, res.message = True, '승인됨 — 실행한다'
        return res

    def _srv_abort(self, _req, res):
        if self.state in (State.IDLE, State.ABORT, State.SAFE_STOP):
            res.success, res.message = False, f'중단할 게 없다 (현재 {self.state.name})'
            return res
        with self._abort_lock:
            self._abort_req = '사용자 abort'
        res.success, res.message = True, '중단 요청'
        return res

    def _srv_reset(self, _req, res):
        if self.state is not State.SAFE_STOP:
            res.success, res.message = False, f'SAFE_STOP 이 아니다 (현재 {self.state.name})'
            return res
        # 곧장 IDLE 로 가지 않는다 — 안전정지가 걸린 자리(테이블/물체 근처일 수 있다)에
        # 팔을 그대로 두면 다음 PERCEIVE 가 그 자리에서 재촬영해 그리퍼 자신을 물체로
        # 오인식한다. HOME 을 거쳐야 한다.
        self._home_next = State.IDLE
        self._to(State.HOME, 'SAFE_STOP 복구 — 홈으로 복귀 후 재개')
        res.success, res.message = True, 'HOME 복귀 후 IDLE'
        return res

    # ────────────────────────────────────────────────────────
    # 전이
    # ────────────────────────────────────────────────────────
    def _to(self, nxt: State, why: str = ''):
        if not is_allowed(self.state, nxt):
            # 전이표에 없는 전이는 버그다. 조용히 넘어가면 상태머신이 아니라 그냥 함수 호출이다.
            self.get_logger().error(f'허용되지 않은 전이 {self.state.name} -> {nxt.name} — ABORT')
            why = f'잘못된 전이 {self.state.name}->{nxt.name}'
            nxt = State.ABORT
        if nxt is not self.state:
            self.get_logger().info(f'[{self.state.name}] -> [{nxt.name}] {why}')
        self.state = nxt
        self._entered = self.get_clock().now()
        self._fut = None
        self._push_fut = None
        self._pushed = False
        self._extra = []
        self._plan_i = 0
        self._nag = 0
        self._octomap_cleared = False
        if nxt is State.PERCEIVE:
            # `_perceive_failed()` 내부 재시도는 `_to()` 를 안 거치므로(같은 상태에 머문다)
            # 여기서 지워도 그 카운트는 안 지워진다 — 여기는 오직 **새 PERCEIVE 진입**(LISTENING
            # 이후, voice_enabled=false 직행, RELEASE_RETRY 뒤 HOME 경유 재인식)만 잡는다.
            # 안 지우면 frame 불일치 같은 `_perceive_failed()` 밖의 SPEAK_FAIL 경로(_accept_grasp)
            # 나, voice_enabled=true 라 `_st_idle` 리셋을 안 거치는 SPEAK_FAIL->LISTENING 루프에서
            # 이전 시도의 잔여 카운트를 물려받아 재시도 예산이 조용히 줄어든다(cross-review
            # 2026-08-09 지적).
            self._retry_perceive = 0
        self.state_pub.publish(String(data=nxt.name))

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._entered).nanoseconds * 1e-9

    def _abort(self, why: str):
        # ⚠️ 물체를 들고 있을 수 있는 구간에서는 **그리퍼를 열지 않는다.**
        #    떨어뜨리는 게 멈춰 있는 것보다 위험하다. 판정은 전이 **전에** 해야 한다 —
        #    _to() 뒤에는 self.state 가 이미 ABORT 라 어디서 왔는지 알 수 없다.
        if self.state in HOLDING_STATES:
            self.get_logger().warn('물체 보유 가능 상태 — 그리퍼를 열지 않고 정지한다')
        if self._call is not None:
            self._call.cancel()
            self._call = None
        self.get_logger().error(f'ABORT: {why}')
        self._to(State.ABORT, why)

    # ────────────────────────────────────────────────────────
    # 메인 루프
    # ────────────────────────────────────────────────────────
    def _tick(self):
        with self._abort_lock:
            why = self._abort_req
            self._abort_req = None
        if why and self.state not in (State.ABORT, State.SAFE_STOP):
            self._abort(why)
            return
        timeout = DEFAULT_TIMEOUTS.get(self.state)
        if timeout is not None and self._elapsed() > timeout:
            self._abort(f'{self.state.name} 제한시간 {timeout:.0f}s 초과')
            return
        try:
            getattr(self, f'_st_{self.state.name.lower()}')()
        except Exception as exc:                                    # noqa: BLE001
            self.get_logger().error(f'{self.state.name}: {type(exc).__name__}: {exc}')
            self._abort(f'{type(exc).__name__}: {exc}')

    def _service(self, client, request, name):
        """(상태, 결과). 상태는 'pending' | 'done'.

        타이머 콜백 안에서 `spin_until_future_complete` 를 부르면 재진입으로 엉킨다
        (executor 가 이미 이 노드를 돌리고 있다) → 기다리지 않고 매 tick 확인한다.

        ⚠️ 서비스가 아직 안 떠 있으면 **기다린다**(실패로 처리하지 않는다). 즉시 실패로
        보내면 노드 기동 순서에 의존하게 되고, LISTENING 이 SPEAK_FAIL 과 tick 마다
        왕복하며 제한시간을 리셋해 영원히 안 멈춘다. 끝내 안 뜨면 DEFAULT_TIMEOUTS 가
        그 상태를 ABORT 시킨다 (LISTENING 60s / PERCEIVE 120s).
        """
        if self._fut is None:
            if not client.service_is_ready():
                self.get_logger().warn(f'{name} 서비스를 기다리는 중',
                                       throttle_duration_sec=5.0)
                return 'pending', None
            self._fut = client.call_async(request)
            return 'pending', None
        if not self._fut.done():
            return 'pending', None
        return 'done', self._fut.result()

    # ────────────────────────────────────────────────────────
    # 상태 핸들러
    # ────────────────────────────────────────────────────────
    def _st_idle(self):
        if not self._start_req:
            return
        self._start_req = False
        self._retry_grip = 0
        self._fail_streak = 0
        self.alternatives = []
        self.solutions.clear()
        self.poses.clear()
        # `/pick/place_location` 로 들어온 값이 파라미터를 이긴다 (target 과 같은 패턴).
        self.place_location = (self._place_override if self._place_override is not None
                               else self.p('place_location'))
        self._publish_place(self.place_location)
        if self.p('voice_enabled'):
            self._to(State.LISTENING)
        else:
            # `/pick/target` 로 들어온 값이 파라미터를 이긴다 (한 번이라도 들어왔다면).
            self.target = (self._target_override if self._target_override is not None
                           else self.p('target'))
            self._publish_target(self.target)
            self._to(State.PERCEIVE, f"타겟 '{self.target or '(자동 — 점수 최고)'}'")

    def _st_listening(self):
        status, res = self._service(self.kw_cli, Trigger.Request(), self.p('keyword_service'))
        if status != 'done':
            return
        if res is None or not res.success:
            self._to(State.SPEAK_FAIL, f'키워드 실패: {getattr(res, "message", "응답 없음")}')
            return
        # get_keyword 는 공백으로 이어붙인 타겟 목록을 message 에 담는다 (robot_control.py:101).
        # 이 FSM 은 한 번에 하나만 집는다 — 여러 개를 큐로 돌리는 건 성공률을 본 뒤에 할 일이다.
        words = res.message.split()
        if not words:
            self._to(State.SPEAK_FAIL, '키워드가 비어 있다')
            return
        self.target = words[0]
        self._publish_target(self.target)
        self._to(State.PERCEIVE, f"타겟 '{self.target}'")

    def _st_perceive(self):
        src = self.p('grasp_source')
        if src == 'manual':
            if self._best is None:
                return                      # 사람이 /grasp/best 를 쏠 때까지 기다린다
            self._accept_grasp(self._best, float(self.p('default_width_m')), 1.0, [])
            return

        # 브리지에 "이번엔 뭘 잡을지"를 먼저 심는다. 끝나기 전에는 계산을 시키지 않는다 —
        # 순서가 뒤집히면 브리지가 **직전 실행의 타겟**으로 수십 초를 계산한다.
        if not self._pushed and not self._push_bridge():
            return

        if src == 'compute_grasp':
            req = ComputeGrasp.Request()
            req.target = self.target
            req.min_confidence = float(self.p('min_confidence'))
            status, res = self._service(self.grasp_cli, req, self.p('grasp_service'))
            if status != 'done':
                return
            if res is None or not res.success:
                self._perceive_failed(f'grasp 없음: {getattr(res, "message", "응답 없음")}')
                return
            self.get_logger().info(f'ComputeGrasp: {res.message}')
            self._accept_grasp(res.grasp_pose, res.width_m, res.confidence,
                               list(res.alternatives),
                               list(getattr(res, 'alternative_widths', [])))
            return

        # legacy_trigger: 지금 graspgenx_perception 의 grasp_bridge_node 가 제공하는 계약.
        # 응답에 포즈가 없고 /grasp/best 로 따로 나온다 → **이번 호출 이후에 들어온** 것만 쓴다.
        # 직전 요청의 포즈를 재활용하면 아무 로그도 없이 엉뚱한 물체를 집는다.
        if self._fut is None:
            self._seq_at_call = self._best_seq
        status, res = self._service(self.grasp_cli, Trigger.Request(),
                                    self.p('grasp_trigger_service'))
        if status != 'done':
            return
        if res is None or not res.success:
            self._perceive_failed(f'grasp 없음: {getattr(res, "message", "응답 없음")}')
            return
        if self._best is None or self._best_seq == self._seq_at_call:
            return                          # 서비스는 끝났지만 포즈가 아직 안 왔다 — 기다린다
        self.get_logger().info(f'/grasp/compute: {res.message}')
        alts = []
        for header, pose in self._candidates[: int(self.p('max_alternatives'))]:
            ps = PoseStamped()
            ps.header, ps.pose = header, pose
            alts.append(ps)
        self._accept_grasp(self._best, float(self.p('default_width_m')), 1.0, alts)

    def _push_bridge(self) -> bool:
        """FSM 타겟 -> 브리지 `target_classes`(+`seg_source`). 끝났으면 True.

        `_service()` 와 달리 브리지가 안 떠 있으면 **기다린다** — PERCEIVE 제한시간(120s)이
        결국 끊는다. 설정 실패는 조용히 넘기지 않는다: 실패하면 브리지가 이전 타겟으로
        계산해 "엉뚱한 물체를 잡았는데 로그는 맞다고 말하는" 상태가 된다.
        """
        if self.bridge_param_cli is None:
            self._pushed = True
            return True
        if self._push_fut is None:
            if not self.bridge_param_cli.service_is_ready():
                self.get_logger().warn(
                    f"{self._bridge_name} 의 set_parameters 를 기다리는 중 "
                    '(grasp_bridge_node 가 떠 있는지 확인할 것)', throttle_duration_sec=5.0)
                return False
            req = SetParameters.Request()
            req.parameters = [str_param('target_classes', self.target)]
            seg = self.p('bridge_seg_source')
            if seg:
                req.parameters.append(str_param('seg_source', seg))
            self._push_fut = self.bridge_param_cli.call_async(req)
            return False
        if not self._push_fut.done():
            return False
        res = self._push_fut.result()
        self._push_fut = None
        names = ['target_classes'] + (['seg_source'] if self.p('bridge_seg_source') else [])
        if res is None or len(res.results) != len(names):
            self._to(State.SPEAK_FAIL, f"{self._bridge_name} 파라미터 설정 응답이 이상하다")
            return False
        bad = [f'{n}: {r.reason}' for n, r in zip(names, res.results) if not r.successful]
        if bad:
            self._to(State.SPEAK_FAIL,
                     f"{self._bridge_name} 파라미터 설정 실패 — " + ' / '.join(bad))
            return False
        self._pushed = True
        seg = self.p('bridge_seg_source')
        self.get_logger().info(
            f"브리지 설정: target_classes='{self.target or '(전부)'}'"
            + (f", seg_source={seg}" if seg else ''))
        return True

    def _perceive_failed(self, why: str):
        """PERCEIVE 요청 실패(grasp 없음 포함) 시 재촬영 재시도. `_motion_failed` 와 같은 뼈대다.

        2026-08-09 실기 로그: 정지된 같은 물체·같은 자리에서 collision-free 비율이
        0%~53%로 요동쳤다(depth 노이즈로 GraspGenX OBB 후보 자체가 흔들린다) — 재촬영
        한 번으로 5번 중 4번은 살아났다. 실패 사유를 가리지 않고 재시도한다: 브리지가
        안 뜬 것 같은 하드 오류라도 몇 초 안에 재시도가 소진되어 결국 SPEAK_FAIL 로
        가는 결과는 같다 — 가려서 얻는 이득이 없다.
        """
        self._retry_perceive += 1
        limit = int(self.p('perceive_retries'))
        if self._retry_perceive <= limit:
            self.get_logger().warn(f'{why} — 재촬영 재시도 {self._retry_perceive}/{limit}')
            self._entered = self.get_clock().now()      # PERCEIVE 제한시간(120s) 갱신
            self._fut = None                             # 다음 tick 에 새 요청을 쏜다
            return
        self._retry_perceive = 0
        self._to(State.SPEAK_FAIL, why)

    def _accept_grasp(self, pose: PoseStamped, width_m, confidence, alternatives,
                      alternative_widths=None):
        base = self.p('base_frame')
        if pose.header.frame_id and pose.header.frame_id != base:
            # tf2 로 옮겨줄 수도 있지만, 규약이 어긋난 채 조용히 도는 것보다 멈추는 게 낫다.
            self._to(State.SPEAK_FAIL,
                     f'grasp 프레임이 {pose.header.frame_id} 다 (기대: {base})')
            return
        # 🔴 여기가 grasp 프레임 -> `rg2_base_link`(= ee_link) 로 넘어오는 **유일한 지점**이다.
        #    best 와 alternatives 를 같이 돌린다 — 대안만 빠뜨리면 첫 후보가 실패한 뒤부터
        #    조용히 90° 틀어진다. 이 아래로는 전부 ee_link 목표 자세다.
        self.grasp = geo.to_gripper_base(pose)
        self.width_m = self._grip_width(width_m)
        n = int(self.p('max_alternatives'))
        self.alternatives = [geo.to_gripper_base(a) for a in list(alternatives)[:n]]
        # 폭은 **후보마다 다르다**(닫힘축이 다르면 같은 물체라도 다르게 잰다). 길이가 안 맞거나
        # 아예 없으면(legacy/manual 경로) 1등 폭을 복제한다 — 짝을 못 맞춘 채 인덱스로 꺼내면
        # 조용히 남의 폭으로 닫는다. 대신 그때는 "모른다"가 아니라 "1등과 같다"로 명시한다.
        w = [float(v) for v in list(alternative_widths or [])[:n]]
        self.alternative_widths = (w if len(w) == len(self.alternatives)
                                   else [float(width_m)] * len(self.alternatives))
        # 변환 **후** 포즈로 잰다. 지금은 요 회전이라 +Z 가 같아서 결과가 같지만, 이 변환에
        # 언젠가 평행이동이 붙으면 여기만 조용히 :502/:568 과 갈라진다.
        tcp = geo.tcp_of(self.grasp, fingertip_from_rg2_base_m(self.width_m))
        self.get_logger().info(
            f'grasp conf={float(confidence):.2f} '
            f'손끝=({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}) '
            f'폭={self.width_m * 1000:.1f} mm 대안={len(self.alternatives)}개')
        self._to(State.SCENE_PREP)

    def _grip_width(self, width_m) -> float:
        """물체 폭 -> 그리퍼 목표 개구 폭 [m]. 부호 규약은 `rg2.grip_target_width_m` 참고.

        여기서 하는 건 파라미터를 읽어 넘기는 것과, 0(=폭 모름)을 기본값으로 대체하는 것뿐이다.
        0 을 그대로 흘리면 완전히 닫혀 물체를 으깬다.
        """
        w = float(width_m)
        if w <= 0.0:
            w = float(self.p('default_width_m'))
            self.get_logger().warn(
                f'폭을 못 받았다(0) — default_width_m={w * 1000:.0f} mm 로 대체한다. '
                'UNVERIFIED 상수라 물체에 맞는다는 보장이 없다')
        return grip_target_width_m(w, float(self.p('grip_clearance_m')),
                                   float(self.p('max_grip_width_m')))

    def _st_scene_prep(self):
        """2단계다: 현재 ACM 을 **읽어서** 거기에 얹은 뒤 적용한다.

        🔴 한 번에 보내면 안 된다. `is_diff=true` 라도 `allowed_collision_matrix` 는
           병합이 아니라 **전체 교체**다 (2026-08-06 실측). 그냥 덮어쓰면 SRDF 의
           `disable_collisions` 34개가 사라져 인접 링크가 자기충돌로 잡히고,
           `avoid_collisions=true` IK 가 **모든 포즈에서** NO_IK_SOLUTION 을 낸다.
        """
        ok, why = self.moveit.ready()
        if not ok:
            if self._nag == 0:
                self._nag = 1
                self.get_logger().warn(f'move_group {why}')
            return
        if self._plan_i == 0:                       # ① ACM 읽기
            if self._fut is None:
                self._fut = self.moveit.get_acm_async()
                return
            if not self._fut.done():
                return
            res = self._fut.result()
            if res is None:
                self._abort('/get_planning_scene 응답 없음')
                return
            self._acm = res.scene.allowed_collision_matrix
            self._fut = None
            self._plan_i = 1
            return
        if self._fut is None:                       # ② 물체 + 병합 ACM 적용
            tcp = geo.tcp_of(self.grasp, fingertip_from_rg2_base_m(self.width_m))
            obj = self.moveit.make_object(self.p('object_id'), tcp,
                                          float(self.p('object_radius_m')))
            acm = merge_acm(self._acm, self.p('object_id'), list(self.p('gripper_links')),
                            allow_octomap=bool(self.p('allow_gripper_octomap_collision')))
            self._fut = self.moveit.add_object_async(obj, acm)
            self._object_added = True
            return
        if not self._fut.done():
            return
        res = self._fut.result()
        if res is None or not res.success:
            self._abort('planning scene 갱신 실패 (/apply_planning_scene)')
            return
        self._to(State.PLAN, f"대상 등록 + ACM {len(self._acm.entry_names)}개 보존")

    def _st_plan(self):
        """pre-grasp → grasp → lift 3점 IK. 하나라도 실패하면 다음 후보로 간다."""
        if not self.poses:
            # `grasp_standoff_m` 은 **이동 목표만** 뒤로 뺀다. `self.grasp` 는 인식이 준 값
            # 그대로 둔다 — SCENE_PREP 의 CollisionObject 와 로그는 "물체가 있다고 본 자리"를
            # 가리켜야 하고, standoff 는 "손끝 모델이 그만큼 틀렸다"는 보정이라 의미가 다르다.
            # 클램프가 걸릴 수 있으니 **적용된 값**을 찍는다(설정값이 아니라).
            approach = float(self.p('approach_offset_m'))
            self.poses = geo.plan_poses(self.grasp, approach,
                                        float(self.p('grasp_standoff_m')),
                                        float(self.p('lift_offset_m')))
            applied = geo.clamped_standoff(float(self.p('grasp_standoff_m')), approach)
            if applied > 0.0:
                self.get_logger().info(
                    f'그립 시작점을 접근축 -Z 로 {applied * 1000:.1f} mm 뺐다 '
                    f'(하강 {(approach - applied) * 1000:.0f} mm, LIFT 도 이 지점 기준)')
        order = ['pre_grasp', 'grasp', 'lift']
        if self._plan_i >= len(order):
            self._to(State.WAIT_APPROVAL, 'IK 3점 성공')
            return
        key = order[self._plan_i]
        pose = self.poses[key]

        if self._fut is None:
            reach = geo.reach_of(pose)
            if reach > float(self.p('max_reach_m')):
                self._to(State.NEXT_CANDIDATE, f'{key} 도달범위 밖 ({reach:.3f} m)')
                return
            if not self.moveit.ik.service_is_ready():
                return
            # 직전 해를 시드로 넘긴다. 안 주면 pre-grasp 와 grasp 의 해가 다른 IK 분기에
            # 앉을 수 있고, 10 cm 하강이 팔 전체를 뒤집는 궤적이 된다.
            seed = self.solutions.get(order[self._plan_i - 1]) if self._plan_i else None
            self._fut = self.moveit.ik_async(
                pose, self.p('planning_group'), self.p('ee_link'), seed=seed,
                avoid_collisions=bool(self.p('ik_avoid_collisions')),
                timeout_sec=float(self.p('ik_timeout_sec')))
            return
        if not self._fut.done():
            return
        res = self._fut.result()
        self._fut = None
        if res is None or res.error_code.val != SUCCESS:
            code = err_name(res.error_code.val) if res is not None else '응답 없음'
            self._to(State.NEXT_CANDIDATE, f'{key} IK 실패 {code}')
            return
        self.solutions[key] = res.solution.joint_state
        self._plan_i += 1

    def _st_next_candidate(self):
        """GPU 를 다시 부르지 않는다. alternatives 를 미리 받아둔 이유가 이것이다."""
        for k in ('pre_grasp', 'grasp', 'lift'):
            self.solutions.pop(k, None)
        self.poses.clear()
        if not self.alternatives:
            self._to(State.SPEAK_FAIL, '후보 소진')
            return
        self.grasp = self.alternatives.pop(0)
        # 포즈만 갈아타고 폭을 그대로 두면 새 후보의 닫힘축에 맞지 않는 폭으로 닫는다.
        if self.alternative_widths:
            self.width_m = self._grip_width(self.alternative_widths.pop(0))
        tcp = geo.tcp_of(self.grasp, fingertip_from_rg2_base_m(self.width_m))
        self.get_logger().info(
            f'다음 후보 (남은 {len(self.alternatives)}개) '
            f'손끝=({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}) '
            f'폭={self.width_m * 1000:.1f} mm')
        self._to(State.PLAN)

    def _st_wait_approval(self):
        if not self.p('require_approval'):
            self._to(State.STOW, '승인 불필요 설정')
            return
        if self._approved:
            self._approved = False
            self._to(State.STOW, '사용자 승인')
            return
        if self._elapsed() > float(self.p('approval_timeout_sec')):
            self._abort('승인 대기 시간 초과')
            return
        if self._elapsed() > self._nag * 10.0:
            self._nag += 1
            self.get_logger().info(
                '✋ 승인 대기 — ros2 service call /pick/approve std_srvs/srv/Trigger {}')

    # ── 이동 4종(pre_grasp/grasp/lift/고정자세)은 같은 뼈대다 ──
    def _move(self, key: str, nxt: State, on_fail: State):
        js = self.solutions.get(key)
        if js is None:
            self._abort(f'{key} 관절해가 없다')
            return
        if self._call is None:
            if not self.moveit.move.server_is_ready():
                return
            self.get_logger().info(f'{key}: 계획+실행 (pipeline={self.p("planning_pipeline")})')
            self._call = self.moveit.move_to_joints_async(
                js, self.p('planning_group'), list(self.p('joint_names')),
                plan_only=False,
                vel_scale=self.p('vel_scale'), acc_scale=self.p('acc_scale'),
                planning_time=self.p('planning_time'), attempts=self.p('planning_attempts'),
                tolerance=self.p('joint_tolerance'),
                replan=bool(self.p('replan')), replan_attempts=self.p('replan_attempts'),
                replan_delay=self.p('replan_delay'),
                pipeline=self.p('planning_pipeline'), planner_id=self.p('planner_id'))
            return
        done, result = self._call.poll()
        if not done:
            return
        rejected = self._call.rejected
        self._call = None
        if rejected or result is None:
            self._motion_failed(key, 'goal 거부됨', on_fail)
            return
        if result.error_code.val == SUCCESS:
            self._retry_motion = 0
            self._to(nxt, f'{key} 완료 (계획 {result.planning_time:.2f}s)')
            return
        self._motion_failed(key, err_name(result.error_code.val), on_fail)

    def _motion_failed(self, key, why, on_fail: State):
        """move_group 이 replan 까지 하고도 실패한 경우의 바깥 재시도."""
        self._retry_motion += 1
        limit = int(self.p('motion_retries'))
        if self._retry_motion <= limit:
            self.get_logger().warn(f'{key} 실패({why}) — 재시도 {self._retry_motion}/{limit}')
            self._entered = self.get_clock().now()      # 제한시간 갱신
            return
        self._retry_motion = 0
        if on_fail is State.ABORT:
            self._abort(f'{key} 실패: {why}')
        else:
            self._to(on_fail, f'{key} 실패: {why}')

    def _st_stow(self):
        """이동 전 그리퍼를 완전히 닫는다 — 벌어진 폭만큼 주변과 부딪힐 여지를 줄인다."""
        if not self._extra:
            if not self.rg2.service_ready():
                return
            self._extra = [self.rg2.close_async(0.0)]
            self.get_logger().info('그리퍼 닫기(이동 대비)')
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        self._to(State.APPROACH)

    def _st_approach(self):
        self._move('pre_grasp', State.OPEN_GRIPPER, State.NEXT_CANDIDATE)

    def _st_open_gripper(self):
        """pre-grasp 도착 후, 하강 전에 그리퍼를 연다."""
        if not self._extra:
            if not self.rg2.service_ready():
                return
            self._extra = [self.rg2.open_async()]
            self.get_logger().info('그리퍼 열기(그립 준비)')
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        self._to(State.DESCEND)

    def _st_descend(self):
        if bool(self.p('clear_octomap_before_descend')) and not self._octomap_cleared:
            self._octomap_cleared = True
            self.moveit.clear_octomap_async()
            self.get_logger().warn('octomap 을 비웠다 — 이 구간은 미모델링 장애물이 안 보인다')
        self._move('grasp', State.CLOSE, State.NEXT_CANDIDATE)

    def _st_close(self):
        if not self._extra:
            if not self.rg2.service_ready():
                return
            self._extra = self.rg2.lower_force_async(int(self.p('force_down_steps')))
            self._extra.append(self.rg2.close_async(self.width_m))
            self.get_logger().info(f'그리퍼 닫기 → {self.width_m * 1000:.1f} mm')
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        self._to(State.VERIFY)

    def _st_verify(self):
        """힘 센서가 없다. 판정 근거는 드라이버가 주는 grip 비트뿐이다."""
        if self._elapsed() < 0.5:
            return
        ok, why = self.rg2.grip_detected()
        if ok is None:
            if bool(self.p('verify_required')):
                self._to(State.RELEASE_RETRY, why)
                return
            if self._nag == 0:          # attach 가 2 tick 걸려서 안 막으면 두 번 찍힌다
                self._nag = 1
                self.get_logger().warn(f'파지 확인 불가 — 통과시킨다 ({why})')
            self._attach_then_lift()
            return
        if ok:
            self._attach_then_lift()
        else:
            self._to(State.RELEASE_RETRY, f'파지 실패 ({why})')

    def _attach_then_lift(self):
        if self._fut is None:
            self._fut = self.moveit.attach_async(
                self.p('object_id'), self.p('ee_link'), list(self.p('gripper_links')))
            return
        if not self._fut.done():
            return
        self._to(State.LIFT, '물체 attach — 이제 부피가 팔을 따라다닌다')

    def _st_release_retry(self):
        if not self._extra:
            self._extra = [self.rg2.open_async()]
            self._retry_grip += 1
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        if self._retry_grip > int(self.p('grip_retries')):
            self._abort(f'파지 재시도 {self._retry_grip}회 실패')
            return
        for k in ('pre_grasp', 'grasp', 'lift'):
            self.solutions.pop(k, None)
        self.poses.clear()
        # 곧장 PERCEIVE 로 가지 않는다 — 팔이 방금 그랩을 시도한 자리(물체 높이, 작업공간
        # 박스 안)에 그대로 있어, 거기서 재촬영하면 그리퍼 자신이 물체로 오인식된다.
        self._home_next = State.PERCEIVE
        self._to(State.HOME, f'재인식 전 홈 복귀 ({self._retry_grip}회차)')

    def _st_lift(self):
        self._move('lift', State.PLACE, State.ABORT)

    def _st_place(self):
        # _st_idle 이 잠근 self.place_location 을 쓴다. 정상 경로로는 여기 도달할 때 항상
        # PLACE_LOCATIONS 의 키다 — __init__ 이 파라미터 기본값을, _on_place_location 이
        # 토픽 오버라이드를 각각 진입 시점에 검증해서 막는다. .get() 의 기본값은 그 두 검증을
        # 모두 우회하는 경로가 생기더라도 조용히 잘못된 곳으로 움직이지 않기 위한 방어선이다.
        param_name = PLACE_LOCATIONS.get(self.place_location, PLACE_LOCATIONS['basket'])
        self._joint_move(param_name, State.RELEASE)

    def _st_release(self):
        if not self._extra:
            self._extra = [self.rg2.open_async()]
            return
        if self._elapsed() < float(self.p('gripper_settle_sec')):
            return
        if self._fut is None:
            self._fut = self.moveit.detach_and_remove_async(self.p('object_id'),
                                                            self.p('ee_link'))
            return
        if not self._fut.done():
            return
        self._object_added = False
        self._home_next = State.IDLE
        self._to(State.HOME)

    def _st_home(self):
        self._joint_move('home_joints_deg', self._home_next)

    def _joint_move(self, param_name: str, nxt: State):
        """고정 관절자세로 이동. IK 가 필요 없어 해를 직접 만든다."""
        if param_name not in self.solutions:
            names = list(self.p('joint_names'))
            positions = geo.deg2rad(self.p(param_name))
            if len(names) != len(positions):
                self._abort(f'{param_name} 길이({len(positions)})가 '
                            f'joint_names({len(names)})와 다르다')
                return
            js = JointState()
            js.name = names
            js.position = positions
            self.solutions[param_name] = js
        self._move(param_name, nxt, State.ABORT)

    def _st_speak_fail(self):
        # TTS 는 이 ws 에 아직 없다. 로그 + /pick/state 로 통보한다.
        self.get_logger().warn(f"실패 통보: 타겟 '{self.target}'")
        self._cleanup_scene()
        self._fail_streak += 1
        if not self.p('voice_enabled'):
            self._to(State.IDLE)
        elif self._fail_streak >= MAX_FAIL_STREAK:
            # 여기서 멈추지 않으면 LISTENING 과 tick 주기로 왕복한다. IDLE 은 조용하고,
            # 사람이 /pick/start 를 다시 불러야 재개된다.
            self._to(State.IDLE, f'연속 실패 {self._fail_streak}회 — /pick/start 로 다시 시작')
        else:
            self._to(State.LISTENING)

    def _st_abort(self):
        self._to(State.SAFE_STOP)

    def _st_safe_stop(self):
        if self._nag == 0:
            self._nag = 1
            self.get_logger().error(
                'SAFE_STOP — 상황 확인 후 ros2 service call /pick/reset std_srvs/srv/Trigger {}')

    def _cleanup_scene(self):
        if self._object_added and self.moveit.scene.service_is_ready():
            self.moveit.detach_and_remove_async(self.p('object_id'), self.p('ee_link'))
            self._object_added = False


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
