#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goal_setter_replan.py — NVIDIA 공식 예제(isaac_ros_moveit_goal_setter) 방식의 재계획 루프

reactive_replan.py 와 **같은 목표를 다른 방식으로** 수행한다. 둘을 같은 목표로 돌려
비교하는 것이 이 파일의 존재 이유다.

    reactive_replan.py : plan_only=True  + JTC 액션 직접 (MoveIt 실행관리자 우회)
    이 파일            : plan_only=False + move_group 이 실행까지 (예제와 동일)

베낀 원본 (도커 마운트 실제 소스)
---------------------------------
isaac_ros-dev/src/isaac_ros_cumotion/isaac_ros_moveit_goal_setter/
    ├── move_group_client.py    ← MoveGroupClient 구조·필드 세팅을 그대로 따랐다
    └── goal_initializer.py     ← 타이머 기반 재계획 루프 구조를 따랐다

원본과 **의도적으로 다르게** 한 곳 (전부 이유가 있다)
------------------------------------------------------
① 🔴 `_get_joint_constraints()` 의 `return` 누락 — **원본의 버그다.**
   move_group_client.py:60-66 이 Constraints 를 만들어 append 만 하고 return 이 없어
   None 을 돌려준다. 그대로 쓰면 `goal_constraints.append(None)` 이 되어 관절 목표가 깨진다.
   여기서는 고쳐서 썼다.

② `goal_change_position_threshold` 기본값 0.0 (원본은 0.1 m)
   원본 goal_initializer.py:119 는 **목표가 10 cm 넘게 움직였을 때만** 재계획한다.
   원본은 "움직이는 목표 추종(object following)"이 목적이라 그게 맞다.
   우리는 **목표 고정 + 장애물이 움직임**이라 그 조건을 켜면 재계획이 한 번도 안 일어난다.
   그래서 기본값을 0.0(항상 재계획)으로 둔다. 원본 거동을 보려면 0.1 로 올린다.

③ 관절 목표를 지원한다 (원본은 TF 의 grasp_frame → pose 목표 전용)
   ⚠️ cuMotion 은 관절 목표도 내부에서 FK 로 EE pose 를 만든 뒤 **IK 를 푼다**
      (cumotion_planner.py:714-730). 그래서 관절 목표인데도 IK_FAIL 이 날 수 있다.

④ `mode` 파라미터로 순차/선점 두 거동을 고를 수 있게 했다 (원본은 순차뿐)
   원본 move_group_client.py:83-84 의 `while self._result is None: sleep(0.01)` 이
   **plan+execute 완료까지 블록**한다. 즉 원본은 실행이 끝나야 다음 계획을 던진다
   → **실행 중에는 장애물이 들어와도 궤적이 안 바뀐다.**
   그게 우리가 풀려는 문제 그 자체이므로, 비교를 위해 두 모드를 다 넣었다.

     mode:=sequential (기본, 원본과 동일)
         계획→실행 완료→다음 계획. **동적 회피가 안 되는 것을 보여주는 대조군.**
     mode:=preemptive
         완료를 안 기다리고 plan_timer_period 마다 새 goal 을 던진다.
         move_group 이 기존 goal 을 선점 교체해 주기를 기대한다.
         🔴 **이 거동은 검증 안 됐다.** move_group 이 선점을 거부하거나(goal reject),
            현재 궤적을 멈췄다 재시작할 수 있다. 그걸 보려고 만든 모드다.

관련 설정 (실측 확인)
---------------------
· m0609_rg2_moveit/config/moveit_controllers.yaml : allowed_start_tolerance = **0.08** rad
  MoveIt 실행관리자가 "궤적 첫 점 vs 현재 로봇 상태" 오차를 이 값으로 검사한다.
  선점 교체가 이 검사에 걸리면 실행이 거부된다. 0.08(≈4.6°)은 두산 원본(0.01)보다 관대하다.
  ⚠️ reactive_replan.py 는 JTC 를 직접 불러 이 검사를 아예 안 탄다. 이게 두 파일의 핵심 차이다.
· cumotion_planner_manager.hpp:53 : canServiceRequest 가 planner_id == "cuMotion" 을 요구
  → planner_id 기본값을 'cuMotion' 으로 둔다(원본과 동일).

⚠️ 로봇이 실제로 움직인다. 첫 실행은 vel 을 낮추고 비상정지 버튼에 손을 올린 채로.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

D2R = math.pi / 180.0
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


def err_name(code: int) -> str:
    for name in dir(MoveItErrorCodes):
        if name.startswith('_') or name == 'val':
            continue
        try:
            if getattr(MoveItErrorCodes, name) == code:
                return f'{name}({code})'
        except TypeError:
            continue
    return f'UNKNOWN({code})'


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class MoveGroupExecClient:
    """예제의 MoveGroupClient 와 같은 역할.

    🔴 원본과의 결정적 공통점: `plan_only = False`.
       계획만 받아오는 게 아니라 **move_group 이 컨트롤러로 실행까지** 한다
       (MoveItSimpleControllerManager → /dsr01/dsr_moveit_controller/follow_joint_trajectory).
       그래서 이 파일은 컨트롤러 액션 이름을 몰라도 된다 — MoveIt 이 알아서 찾는다.
    """

    def __init__(self, node: Node, group_name: str, pipeline_id: str,
                 planner_id: str, ee_link: str, base_frame: str):
        self._node = node
        self._group_name = group_name
        self._pipeline_id = pipeline_id
        self._planner_id = planner_id
        self._ee_link = ee_link
        self._base_frame = base_frame

        self._cb = ReentrantCallbackGroup()
        self._client = ActionClient(node, MoveGroup, '/move_action', callback_group=self._cb)

        self._result = None
        self._goal_handle = None
        self._lock = threading.Lock()

    def wait_for_server(self, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while not self._client.wait_for_server(timeout_sec=1.0):
            if time.time() - t0 > timeout:
                self._node.get_logger().error(
                    '/move_action 액션 서버 없음 → move_group(T7) 미기동')
                return False
            self._node.get_logger().info('move_action 대기 중...')
        return True

    # ── 목표 구성 ───────────────────────────────────────────────────────────
    def joint_constraints(self, positions: Sequence[float],
                          joint_names: Sequence[str]) -> Constraints:
        """🔴 원본(move_group_client.py:60-66)은 return 이 빠져 None 을 돌려준다 — 버그.
        여기서는 constraints 를 제대로 돌려준다."""
        constraints = Constraints()
        for position, joint_name in zip(positions, joint_names):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            # 원본은 tolerance/weight 를 안 채운다. cuMotion 은 position 만 읽으므로
            # (cumotion_planner.py:714-722) 동작은 하지만, ompl 로 바꿔 쓸 때를 위해 채운다.
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def pose_constraints(self, pose: PoseStamped) -> Constraints:
        """원본 _get_pose_constraints 와 같은 구성.

        ⚠️ 원본은 constraint_region.primitives 를 안 채우고 primitive_poses 만 채운다.
           cuMotion 은 primitive_poses[0].position 만 읽으므로(cumotion_planner.py:737-743)
           그래도 동작하지만, pipeline_id:=ompl 로 바꾸면 영역이 비어 실패한다.
           그래서 여기서는 SPHERE 를 하나 넣어 둔다.
        """
        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = pose.header.frame_id
        pc.link_name = self._ee_link
        pc.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])]
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = pose.header.frame_id
        oc.link_name = self._ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        return constraints

    # ── 목표 전송 ───────────────────────────────────────────────────────────
    def send_goal(self, constraints: Constraints,
                  vel_scale: float, acc_scale: float,
                  allowed_planning_time: float = 10.0,
                  wait: bool = True, wait_timeout: float = 120.0):
        """MoveGroup goal 을 보낸다.

        wait=True  : 원본과 동일. plan+execute 가 **끝날 때까지 블록**하고 결과를 돌려준다.
        wait=False : 보내고 바로 리턴(선점 실험용). 결과는 last_result() 로 나중에 본다.
        """
        with self._lock:
            self._result = None

        goal_msg = MoveGroup.Goal()
        goal_msg.request.planner_id = self._planner_id      # 'cuMotion' (canServiceRequest)
        goal_msg.request.pipeline_id = self._pipeline_id    # 'isaac_ros_cumotion'
        goal_msg.request.group_name = self._group_name
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = allowed_planning_time
        # 🔴 원본은 scaling 을 안 건드린다(=0.0 → cuMotion 이 자기 time_dilation_factor 사용).
        #    우리는 실기 속도를 통제해야 하므로 명시한다. cumotion_planner.yaml 이
        #    override_moveit_scaling_factors:false 라 여기서 보낸 값이 이긴다.
        goal_msg.request.max_velocity_scaling_factor = vel_scale
        goal_msg.request.max_acceleration_scaling_factor = acc_scale

        # 🔴 이 한 줄이 reactive_replan.py 와의 근본 차이다.
        #    False = move_group 이 계획하고 **실행까지** 한다(실행관리자·start tolerance 경유).
        goal_msg.planning_options.plan_only = False

        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

        if not wait:
            return None

        deadline = time.time() + wait_timeout
        while rclpy.ok():
            with self._lock:
                if self._result is not None:
                    return self._result
            if time.time() > deadline:
                self._node.get_logger().error(f'결과 대기 {wait_timeout:.0f}s 초과')
                return None
            time.sleep(0.01)
        return None

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # 🔴 선점 모드에서 여기가 찍히면 "move_group 은 동시 goal 을 거부한다"는 뜻이다.
            #    그 경우 예제 방식으로는 실행 중 궤적 교체가 원리적으로 불가능하다.
            self._node.get_logger().error(
                'move_group 이 goal 을 거부했다 (선점 모드면 = 동시 goal 불가)')
            with self._lock:
                self._result = 'REJECTED'
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_cb)

    def _get_result_cb(self, future):
        result = future.result().result
        with self._lock:
            self._result = result

    def last_result(self):
        with self._lock:
            return self._result

    def cancel(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None


class GoalSetterReplanNode(Node):
    """예제 goal_initializer.py 의 타이머 루프 구조를 따른 노드."""

    def __init__(self):
        super().__init__('goal_setter_replan')

        p = self.declare_parameter
        # sequential = 원본 거동(실행 완료까지 대기) / preemptive = 완료 안 기다리고 새 goal
        self.mode = p('mode', 'sequential').value
        # 원본 goal_initializer.py:50 과 같은 이름·기본값
        self.plan_timer_period = float(p('plan_timer_period', 2.0).value)
        # 원본은 0.1 m. 목표 고정인 우리 용도에서는 0.0(항상 재계획)이 맞다 — 파일 상단 ② 참고
        self.goal_change_position_threshold = float(
            p('goal_change_position_threshold', 0.0).value)

        self.group_name = p('planner_group_name', 'manipulator').value
        self.pipeline_id = p('pipeline_id', 'isaac_ros_cumotion').value
        self.planner_id = p('planner_id', 'cuMotion').value        # canServiceRequest 요구값
        self.ee_link = p('end_effector_link', 'tool0').value
        self.base_frame = p('world_frame', 'base_link').value

        self.vel_scale = float(p('vel_scale', 0.15).value)
        self.acc_scale = float(p('acc_scale', 0.15).value)
        self.allowed_planning_time = float(p('allowed_planning_time', 10.0).value)

        self.goal_type = p('goal_type', 'joint').value              # joint | pose
        self.goal_joint_deg = list(p('goal_joint_deg', [45.0, 0.0, 90.0, 0.0, 90.0, 0.0]).value)
        self.goal_pose = list(p('goal_pose', [0.45, 0.0, 0.35, 180.0, 0.0, 0.0]).value)

        self.pingpong = bool(p('pingpong', False).value)
        self.pingpong_b_deg = list(p('pingpong_b_deg',
                                     [-45.0, 0.0, 90.0, 0.0, 90.0, 0.0]).value)
        self.cycles = int(p('cycles', 0).value)                     # 0 = 무한

        self.joint_goal_tol = float(p('joint_goal_tol', 0.01).value)
        # 🔴 목표 하나(pingpong=false)일 때 도착하면 종료한다.
        #    이게 없으면 도착한 뒤에도 타이머가 같은 goal 을 영원히 다시 던진다
        #    (2026-08-07 실측: 이미 목표에 있는데도 사이클마다 SUCCESS 가 5.8초씩 찍혔다).
        self.stop_when_arrived = bool(p('stop_when_arrived', True).value)

        self._cb = ReentrantCallbackGroup()
        self._js_lock = threading.Lock()
        self._js_pos: dict = {}
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
            callback_group=self._cb)

        self.client = MoveGroupExecClient(
            self, self.group_name, self.pipeline_id, self.planner_id,
            self.ee_link, self.base_frame)

        self._previous_goal_position = None
        self._busy = False
        self._target_idx = 0
        self._cycle_count = 0
        self.stat_goals = 0
        self.stat_fail = 0

        if self.mode not in ('sequential', 'preemptive'):
            self.get_logger().error(
                f"mode:={self.mode} 는 없다. sequential | preemptive 중 하나.")

        self.get_logger().info(
            f'모드={self.mode} / plan_timer_period={self.plan_timer_period}s / '
            f'vel={self.vel_scale} / goal_type={self.goal_type}')
        if self.mode == 'sequential':
            self.get_logger().warn(
                '🔴 sequential 은 예제 원본 거동이다 — 실행이 끝나야 다음 계획을 던진다. '
                '즉 **실행 중에 장애물이 들어와도 궤적이 안 바뀐다.** 대조군용이다.')
        else:
            self.get_logger().warn(
                '⚠️ preemptive 는 검증 안 된 실험 모드다. move_group 이 동시 goal 을 '
                '거부하거나 궤적을 멈췄다 재시작할 수 있다. 그걸 보려는 게 목적이다.')

        # 원본 goal_initializer.py:71 과 같은 구조
        self.timer = self.create_timer(self.plan_timer_period, self.on_timer,
                                       callback_group=self._cb)

    def _on_joint_states(self, msg: JointState) -> None:
        with self._js_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._js_pos[name] = msg.position[i]

    def current_joints(self) -> List[float]:
        with self._js_lock:
            return [self._js_pos.get(j, 0.0) for j in JOINT_NAMES]

    def _current_target_deg(self) -> List[float]:
        if self.pingpong and self._target_idx == 1:
            return self.pingpong_b_deg
        return self.goal_joint_deg

    def _build_constraints(self) -> Optional[Constraints]:
        if self.goal_type == 'joint':
            target = [v * D2R for v in self._current_target_deg()]
            return self.client.joint_constraints(target, JOINT_NAMES)

        if self.goal_type == 'pose':
            ps = PoseStamped()
            ps.header.frame_id = self.base_frame
            ps.header.stamp = self.get_clock().now().to_msg()
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = \
                [float(v) for v in self.goal_pose[:3]]
            pose.orientation = quat_from_rpy(*[v * D2R for v in self.goal_pose[3:]])
            ps.pose = pose

            # 원본 goal_initializer.py:116-124 의 "목표가 충분히 움직였을 때만" 게이트.
            # threshold 0.0 이면 항상 통과한다(우리 기본값).
            if self.goal_change_position_threshold > 0.0:
                import numpy as np
                new_goal = np.array(self.goal_pose[:3], dtype=float)
                if self._previous_goal_position is not None:
                    d = float(np.linalg.norm(self._previous_goal_position - new_goal))
                    if d <= self.goal_change_position_threshold:
                        self.get_logger().warning(
                            f'목표가 {d:.3f} m 밖에 안 움직였다 '
                            f'(<= {self.goal_change_position_threshold}) — 재계획 생략. '
                            '⚠️ 목표 고정 + 장애물 회피 용도라면 이 값을 0.0 으로 둬야 한다.')
                        return None
                self._previous_goal_position = new_goal
            return self.client.pose_constraints(ps)

        self.get_logger().error(f"goal_type:={self.goal_type} 는 없다. joint | pose.")
        return None

    def _arrived(self) -> bool:
        if self.goal_type != 'joint':
            return False
        target = [v * D2R for v in self._current_target_deg()]
        cur = self.current_joints()
        return max(abs(cur[i] - target[i]) for i in range(len(cur))) < self.joint_goal_tol

    def _finish(self, reason: str) -> None:
        """타이머를 멈추고 spin() 을 빠져나가게 한다.

        타이머 콜백은 executor 의 워커 스레드에서 도므로 여기서 예외를 던져도
        main 의 spin() 까지 안 올라간다. rclpy.shutdown() 이라야 spin() 이 리턴한다.
        """
        try:
            self.timer.cancel()
        except Exception:
            pass
        self.get_logger().info(f'{reason} | {self.summary()}')
        if rclpy.ok():
            rclpy.shutdown()

    def on_timer(self) -> None:
        # sequential 모드에서 send_goal 이 블록하는 동안 타이머가 재진입하지 않게 막는다.
        # (원본은 MutuallyExclusive 콜백그룹이라 자동으로 막혔다. 우리는 Reentrant 라 직접 막는다)
        if self.mode == 'sequential' and self._busy:
            return

        # 🔴 목표가 하나뿐인데 이미 도착했으면 끝낸다.
        #    ⚠️ pingpong 일 때는 도착이 "다음 목표로 전환" 신호이므로 여기서 끝내면 안 된다.
        if self.stop_when_arrived and not self.pingpong and self._arrived():
            self._finish('목표 도착 — 종료')
            return

        constraints = self._build_constraints()
        if constraints is None:
            return

        wait = (self.mode == 'sequential')
        self._busy = True
        self.stat_goals += 1
        name = 'B' if (self.pingpong and self._target_idx == 1) else 'A'
        self.get_logger().info(f'goal #{self.stat_goals} 전송 (목표 {name})')

        t0 = time.time()
        try:
            result = self.client.send_goal(
                constraints, self.vel_scale, self.acc_scale,
                allowed_planning_time=self.allowed_planning_time, wait=wait)
        finally:
            self._busy = False

        if not wait:
            return   # 선점 모드: 결과는 다음 틱에서 로그로만 확인
        elapsed = time.time() - t0

        if result is None:
            self.stat_fail += 1
            self.get_logger().error('결과 없음(타임아웃)')
            return
        if result == 'REJECTED':
            self.stat_fail += 1
            return

        code = result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            # 🔴 elapsed = 이 goal 이 나가고 실행이 끝날 때까지의 시간.
            #    이 구간 내내 **새 계획이 안 들어간다** = 장애물이 들어와도 못 본다.
            #    예제 방식(plan_only=False)이 동적 회피를 못 하는 이유가 이 숫자다.
            #    2026-08-07 실측: 이미 목표에 있는데도 매번 5.8~6.0초.
            self.get_logger().info(
                f'✅ 목표 {name} 도착 (plan+execute 완료) | '
                f'회피 불가 구간 {elapsed:.2f}초')
            if self.pingpong:
                self._target_idx ^= 1
                if self._target_idx == 0:
                    self._cycle_count += 1
                    self.get_logger().info(f'왕복 {self._cycle_count}회 | {self.summary()}')
                    if self.cycles and self._cycle_count >= self.cycles:
                        self._finish('지정 왕복 완료 — 종료')
                        return
        else:
            self.stat_fail += 1
            hint = ''
            if code == MoveItErrorCodes.PLANNING_FAILED:
                hint = (' → cuMotion 경로면 원인이 안 담긴다(플러그인이 덮어씀). '
                        'T6 로그의 "Motion planning failed wih status:" 를 본다.')
            elif code == MoveItErrorCodes.CONTROL_FAILED:
                hint = (' → 실행 단계 실패. allowed_start_tolerance(0.08) 위반이거나 '
                        'JTC 가 궤적을 거부했다.')
            elif code == MoveItErrorCodes.FAILURE:
                hint = ' → cumotion_planner_node 가 이미 다른 계획 처리 중(planner_busy)일 수 있다.'
            self.get_logger().error(f'❌ 실패: {err_name(code)}{hint}')

    def summary(self) -> str:
        return f'goal {self.stat_goals}회 (실패 {self.stat_fail})'


def main(args=None) -> int:
    rclpy.init(args=args)
    node = GoalSetterReplanNode()

    if not node.client.wait_for_server(timeout=30.0):
        node.destroy_node()
        rclpy.shutdown()
        return 1

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    rc = 0
    try:
        executor.spin()
    except ExternalShutdownException:
        # _finish() 가 rclpy.shutdown() 을 부른 정상 종료 경로다. 이미 요약을 찍었다.
        pass
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('중단 — goal 취소')
        node.client.cancel()
    finally:
        try:
            node.get_logger().info(node.summary())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
