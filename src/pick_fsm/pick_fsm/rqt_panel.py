"""pick_fsm 상태 감시 + 승인/안전 조작용 rqt 패널.

    rqt --standalone pick_fsm

`rqt_gui` 가 플러그인마다 공유 rclpy 노드를 하나씩 준다(`context.node`) — 여기서 새로
`rclpy.init()`/스레드를 만들지 않는다. 버튼 클릭은 전부 `call_async()`로 쏘고 바로 리턴한다.
`spin_until_future_complete`류 블로킹 호출은 rqt 의 spin 스레드를 막아 패널 전체가 멈춘다
(`task_manager.py`/`robot_safety_node.py`가 같은 이유로 폴링 방식을 쓴다) — 여기서도 QTimer 로
매 200ms 마다 대기 중인 future 의 `done()` 만 확인한다. ROS 콜백 스레드에서 Qt 위젯을 직접
건드리지 않는다(Qt 는 GUI 스레드 밖에서 위젯을 만지면 크래시할 수 있다) — 최신값을 변수에만
넣어두고 같은 QTimer 가 라벨에 반영한다.
"""

import time

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from pick_fsm.robot_safety_node import ROBOT_STATE_NAMES, UNSAFE_STATES

#: 눌렀을 때 로봇에 물리적 영향이 있어 확인창을 띄우는 버튼들
CONFIRM = {
    '즉시정지': '지금 진행 중인 모든 계획/실행을 즉시 멈춥니다. 계속할까요?',
    '안전모드 진입(backdrive)': ('사람이 손으로 팔을 밀 수 있는 모드로 들어갑니다.\n'
                                '전환 직후 팔이 갑자기 무거움/가벼움이 바뀔 수 있습니다 — '
                                '준비된 상태에서 누르세요.'),
    '안전모드 해제': '정상 모드로 복귀합니다. 팔 주변에 손이 없는지 확인하세요.',
}

#: 서비스가 이 시간 안에 응답 안 하면 결과 표시에서 "시간초과"로 걷어낸다.
#: (버튼 자체가 fire-and-forget 라 로봇 명령이 취소되는 건 아니다 — UI 표시만 정리한다)
PENDING_TIMEOUT_SEC = 5.0


class PickFsmPanel(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('PickFsmPanel')
        node = context.node

        self._fsm_state = '(수신 대기)'
        self._robot_state = '(수신 대기)'
        self._robot_state_code = None
        self._pending = []          # [(label, future, deadline)] — 완료되면 QTimer 가 걷어간다
        self._clients = []          # 버튼마다 하나씩, __init__ 에서만 만든다 (재사용, 안 새로 안 만듦)

        self._widget = QWidget()
        self._widget.setWindowTitle('pick_fsm 상태/제어')
        outer = QVBoxLayout(self._widget)

        self._lbl_fsm = QLabel('FSM 상태: (수신 대기)')
        self._lbl_robot = QLabel('로봇 상태: (수신 대기)')
        self._lbl_result = QLabel('')
        self._lbl_result.setWordWrap(True)
        for lbl in (self._lbl_fsm, self._lbl_robot):
            f = lbl.font()
            f.setPointSize(f.pointSize() + 2)
            f.setBold(True)
            lbl.setFont(f)
        outer.addWidget(self._lbl_fsm)
        outer.addWidget(self._lbl_robot)

        task_box = QGroupBox('작업')
        task_row = QHBoxLayout(task_box)
        for text, srv in (('시작', '/pick/start'), ('승인', '/pick/approve'),
                          ('리셋', '/pick/reset')):
            task_row.addWidget(self._make_button(node, text, Trigger, srv))
        outer.addWidget(task_box)

        stop_box = QGroupBox('정지')
        stop_row = QHBoxLayout(stop_box)
        for text, srv in (('중단(ABORT)', '/pick/abort'), ('즉시정지', '/safety/stop')):
            btn = self._make_button(node, text, Trigger, srv)
            btn.setStyleSheet('background-color:#c0392b; color:white; font-weight:bold;')
            stop_row.addWidget(btn)
        outer.addWidget(stop_box)

        safe_box = QGroupBox('안전모드 (사람이 팔을 손으로 옮길 때)')
        safe_row = QHBoxLayout(safe_box)
        safe_row.addWidget(self._make_button(
            node, '안전모드 진입(backdrive)', Trigger, '/safety/enter_backdrive'))
        safe_row.addWidget(self._make_button(
            node, '안전모드 해제', Trigger, '/safety/exit_backdrive'))
        outer.addWidget(safe_box)
        warn = QLabel('⚠️ backdrive는 이 도구에서 실기 검증된 적이 없다 — 처음 쓸 때는 '
                      '비상정지 버튼을 손 닿는 곳에 두고 저속·저위험 자세에서 먼저 확인할 것.')
        warn.setWordWrap(True)
        warn.setStyleSheet('color:#c0392b;')
        outer.addWidget(warn)

        outer.addWidget(self._lbl_result)
        outer.addStretch(1)
        context.add_widget(self._widget)

        self._sub_fsm = node.create_subscription(String, '/pick/state', self._on_fsm, 10)
        self._sub_robot = node.create_subscription(
            Int8, '/pick/robot_state_code', self._on_robot, 10)

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(200)

    # ── ROS 콜백 (ROS 스레드에서 실행 — 변수에만 쓴다) ──────────
    def _on_fsm(self, msg):
        self._fsm_state = msg.data

    def _on_robot(self, msg):
        self._robot_state_code = int(msg.data)
        self._robot_state = ROBOT_STATE_NAMES.get(self._robot_state_code,
                                                   f'UNKNOWN({self._robot_state_code})')

    # ── Qt 타이머 (GUI 스레드에서 실행 — 위젯은 여기서만 건드린다) ──
    def _refresh(self):
        self._lbl_fsm.setText(f'FSM 상태: {self._fsm_state}')
        unsafe = self._robot_state_code in UNSAFE_STATES
        self._lbl_robot.setText(f'로봇 상태: {self._robot_state}' + ('  ⚠️ 안전정지류' if unsafe else ''))
        self._lbl_robot.setStyleSheet('color:#c0392b; font-weight:bold;' if unsafe else '')

        now = time.monotonic()
        still_pending = []
        for label, fut, deadline in self._pending:
            if fut.done():
                res = fut.result()
                if res is None:
                    self._lbl_result.setText(f'{label}: 응답 없음')
                else:
                    self._lbl_result.setText(
                        f'{label}: {"OK" if res.success else "실패"} — {res.message}')
                continue
            if now > deadline:
                self._lbl_result.setText(f'{label}: {PENDING_TIMEOUT_SEC:.0f}s 시간초과 — 응답 없음')
                continue
            still_pending.append((label, fut, deadline))
        self._pending = still_pending

    # ── 버튼 ────────────────────────────────────────────────
    def _make_button(self, node, label, srv_type, srv_name):
        # 클라이언트는 버튼당 한 번만 만든다. 클릭마다 새로 만들면 DDS 리소스가
        # 계속 쌓이고 아무도 destroy 하지 않는다 — 이 패널은 반복 클릭이 정상 사용이다.
        client = node.create_client(srv_type, srv_name)
        self._clients.append(client)
        btn = QPushButton(label)

        def on_click():
            if label in CONFIRM:
                if QMessageBox.question(
                        self._widget, label, CONFIRM[label]) != QMessageBox.Yes:
                    return
            if not client.service_is_ready():
                self._lbl_result.setText(f'{label}: {srv_name} 서비스 없음')
                return
            fut = client.call_async(srv_type.Request())
            self._pending.append((label, fut, time.monotonic() + PENDING_TIMEOUT_SEC))
            self._lbl_result.setText(f'{label}: 요청 전송함…')
        btn.clicked.connect(on_click)
        return btn

    # ── rqt 생명주기 ─────────────────────────────────────────
    def shutdown_plugin(self):
        self._timer.stop()
        try:
            self._sub_fsm.destroy()
            self._sub_robot.destroy()
            for client in self._clients:
                client.destroy()
        except Exception:                                  # noqa: BLE001
            pass
