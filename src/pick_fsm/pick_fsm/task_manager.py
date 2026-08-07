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

⚠️ 기본값은 `dry_run:=true` (계획만, 실행 안 함) + `require_approval:=true` 다.
   실기에서 움직이려면 두 개를 명시적으로 꺼야 한다. 사고는 기본값에서 나온다.
"""

import threading

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from pick_fsm import geometry as geo
from pick_fsm.moveit_bridge import SUCCESS, MoveItBridge, err_name, merge_acm
from pick_fsm.rg2 import RG2_MODEL_WIDTH_M, Rg2Client, fingertip_from_rg2_base_m
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


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        p = self._declare_params()
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
        self.grasp = None           # PoseStamped, ee_link 목표 자세
        self.width_m = 0.0
        self.alternatives = []
        self.poses = {}             # 'pre_grasp'|'grasp'|'lift' -> PoseStamped
        self.solutions = {}         # 같은 키 -> JointState
        self._plan_i = 0
        self._retry_motion = 0
        self._retry_grip = 0
        self._object_added = False
        self._nag = 0
        self._home_next = State.IDLE   # HOME 도착 후 갈 곳. _srv_reset/_st_release_retry 가 덮어쓴다

        self.timer = self.create_timer(1.0 / p['tick_hz'], self._tick, callback_group=cb)
        self.get_logger().info(
            f"준비됨 — dry_run={p['dry_run']}, require_approval={p['require_approval']}, "
            f"grasp_source={p['grasp_source']}, gripper_backend={p['gripper_backend']}")
        if not p['dry_run']:
            self.get_logger().warn('⚠️ dry_run=false — 승인하면 로봇이 실제로 움직인다')

    # ────────────────────────────────────────────────────────
    # 파라미터
    # ────────────────────────────────────────────────────────
    def _declare_params(self):
        d = {
            # 안전
            'dry_run': True,                 # true = plan_only. 궤적만 만들고 실행하지 않는다
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

            # 자세
            'approach_offset_m': 0.10,       # pre-grasp: grasp 의 -Z 로 물러나는 거리
            'lift_offset_m': 0.15,           # LIFT: 월드 +Z
            # tcp_offset_m 은 더 이상 파라미터가 아니다 — rg2.fingertip_length_m(width_m)이
            # 2026-08-07 실측(폭에 따라 손끝이 짧아지는 비선형 보정표)으로 대체했다.
            'max_reach_m': 0.900,            # M0609 URDF 실측 (shoulder 기준)
            'home_joints_deg': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],     # robot_control JReady
            'place_joints_deg': [4.0, 38.0, 64.0, -0.1, 78.0, 4.0],  # robot_control BUCKET_POS

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
            'grip_clearance_m': 0.008,       # UNVERIFIED: 실측 튜닝값. 도면값 아님
            'max_grip_width_m': RG2_MODEL_WIDTH_M,
            'force_down_steps': 0,           # 'd' 반복 횟수. 0 = 드라이버 기본(=40 N, RG2 최대)
            'gripper_settle_sec': 1.5,
            'verify_required': False,        # true 면 grip_detected 를 못 받았을 때도 실패 처리
            'grip_retries': 1,

            # 인식
            'grasp_source': 'compute_grasp',  # compute_grasp | legacy_trigger | manual
            'grasp_service': '/grasp/compute_grasp',
            'grasp_trigger_service': '/grasp/compute',
            'grasp_best_topic': '/grasp/best',
            'grasp_candidates_topic': '/grasp/candidates',
            'min_confidence': 0.5,
            'default_width_m': 0.06,         # legacy/manual 경로에는 폭 정보가 없다
            'max_alternatives': 5,

            # 음성
            'voice_enabled': True,
            'keyword_service': '/get_keyword',
            'target': '',                    # voice_enabled=false 일 때 쓸 고정 타겟

            'tick_hz': 10.0,
        }
        for k, v in d.items():
            self.declare_parameter(k, v)
        return {k: self.get_parameter(k).value for k in d}

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
        self._extra = []
        self._plan_i = 0
        self._nag = 0
        self._octomap_cleared = False
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
        """(상태, 결과). 상태는 'pending' | 'done' | 'unavailable'.

        타이머 콜백 안에서 `spin_until_future_complete` 를 부르면 재진입으로 엉킨다
        (executor 가 이미 이 노드를 돌리고 있다) → 기다리지 않고 매 tick 확인한다.
        """
        if self._fut is None:
            if not client.service_is_ready():
                return 'unavailable', f'{name} 서비스 없음'
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
        self.alternatives = []
        self.solutions.clear()
        self.poses.clear()
        if self.p('voice_enabled'):
            self._to(State.LISTENING)
        else:
            self.target = self.p('target')
            self._to(State.PERCEIVE, f"고정 타겟 '{self.target}'")

    def _st_listening(self):
        status, res = self._service(self.kw_cli, Trigger.Request(), self.p('keyword_service'))
        if status == 'unavailable':
            self._to(State.SPEAK_FAIL, res)
            return
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
        self._to(State.PERCEIVE, f"타겟 '{self.target}'")

    def _st_perceive(self):
        src = self.p('grasp_source')
        if src == 'manual':
            if self._best is None:
                return                      # 사람이 /grasp/best 를 쏠 때까지 기다린다
            self._accept_grasp(self._best, float(self.p('default_width_m')), 1.0, [])
            return

        if src == 'compute_grasp':
            req = ComputeGrasp.Request()
            req.target = self.target
            req.min_confidence = float(self.p('min_confidence'))
            status, res = self._service(self.grasp_cli, req, self.p('grasp_service'))
            if status == 'unavailable':
                self._to(State.SPEAK_FAIL, res)
                return
            if status != 'done':
                return
            if res is None or not res.success:
                self._to(State.SPEAK_FAIL, f'grasp 없음: {getattr(res, "message", "응답 없음")}')
                return
            self.get_logger().info(f'ComputeGrasp: {res.message}')
            self._accept_grasp(res.grasp_pose, res.width_m, res.confidence,
                               list(res.alternatives))
            return

        # legacy_trigger: 지금 graspgenx_perception 의 grasp_bridge_node 가 제공하는 계약.
        # 응답에 포즈가 없고 /grasp/best 로 따로 나온다 → **이번 호출 이후에 들어온** 것만 쓴다.
        # 직전 요청의 포즈를 재활용하면 아무 로그도 없이 엉뚱한 물체를 집는다.
        if self._fut is None:
            self._seq_at_call = self._best_seq
        status, res = self._service(self.grasp_cli, Trigger.Request(),
                                    self.p('grasp_trigger_service'))
        if status == 'unavailable':
            self._to(State.SPEAK_FAIL, res)
            return
        if status != 'done':
            return
        if res is None or not res.success:
            self._to(State.SPEAK_FAIL, f'grasp 없음: {getattr(res, "message", "응답 없음")}')
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

    def _accept_grasp(self, pose: PoseStamped, width_m, confidence, alternatives):
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
        self.alternatives = [geo.to_gripper_base(a)
                             for a in list(alternatives)[: int(self.p('max_alternatives'))]]
        # 변환 **후** 포즈로 잰다. 지금은 요 회전이라 +Z 가 같아서 결과가 같지만, 이 변환에
        # 언젠가 평행이동이 붙으면 여기만 조용히 :502/:568 과 갈라진다.
        tcp = geo.tcp_of(self.grasp, fingertip_from_rg2_base_m(self.width_m))
        self.get_logger().info(
            f'grasp conf={float(confidence):.2f} '
            f'손끝=({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}) '
            f'폭={self.width_m * 1000:.1f} mm 대안={len(self.alternatives)}개')
        self._to(State.SCENE_PREP)

    def _grip_width(self, width_m) -> float:
        """물체 폭 + 클리어런스, 모델/하드웨어 한계로 클램프."""
        w = float(width_m) + float(self.p('grip_clearance_m'))
        return max(0.0, min(float(self.p('max_grip_width_m')), w))

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
            self.poses = {
                'pre_grasp': geo.pre_grasp(self.grasp, float(self.p('approach_offset_m'))),
                'grasp': self.grasp,
                'lift': geo.lifted(self.grasp, float(self.p('lift_offset_m'))),
            }
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
        tcp = geo.tcp_of(self.grasp, fingertip_from_rg2_base_m(self.width_m))
        self.get_logger().info(
            f'다음 후보 (남은 {len(self.alternatives)}개) '
            f'손끝=({tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f})')
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
            dry = bool(self.p('dry_run'))
            self.get_logger().info(f'{key}: {"계획만(dry_run)" if dry else "계획+실행"}')
            self._call = self.moveit.move_to_joints_async(
                js, self.p('planning_group'), list(self.p('joint_names')),
                plan_only=dry,
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
        self._joint_move('place_joints_deg', State.RELEASE)

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
        self._to(State.LISTENING if self.p('voice_enabled') else State.IDLE)

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
