"""포즈 연산. ROS 메시지에만 의존하고 로봇/네트워크는 안 건드린다 → 단위테스트가 가능하다.

grasp 포즈의 규약 (md/context/constraints.md "GraspGenX grasp 4x4 = tool0 목표 자세"):
    +Z = 접근축(approach) · +X = 손가락이 닫히는 방향 · 원점 = 그리퍼 base = 우리 `tool0`

여기서 scipy 를 쓰지 않는다. 필요한 건 쿼터니언에서 축 하나 뽑는 것뿐인데
그 한 줄 때문에 노드 실행 의존성을 늘릴 이유가 없다.
"""

import math

from geometry_msgs.msg import Pose, PoseStamped


def quat_axis_x(q) -> tuple[float, float, float]:
    """쿼터니언이 나타내는 회전행렬의 1열 = 로컬 +X축을 월드에서 본 방향."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w))


def quat_axis_z(q) -> tuple[float, float, float]:
    """회전행렬의 3열 = 로컬 +Z축. grasp 포즈에서는 **접근축**이다."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return (2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y))


def translated(pose: Pose, axis: tuple[float, float, float], dist: float) -> Pose:
    """자세는 그대로 두고 위치만 `axis` 방향으로 `dist` [m] 옮긴 새 Pose."""
    out = Pose()
    out.orientation = pose.orientation
    out.position.x = pose.position.x + axis[0] * dist
    out.position.y = pose.position.y + axis[1] * dist
    out.position.z = pose.position.z + axis[2] * dist
    return out


def pre_grasp(grasp: PoseStamped, back_off_m: float) -> PoseStamped:
    """접근축(-Z)으로 `back_off_m` 만큼 물러난 포즈.

    **월드 -Z(수직 위)가 아니라 grasp 자신의 -Z 다.** 비스듬히 접근하는 grasp 에서
    수직으로 물러나면 하강 구간이 직선이 아니게 되어 손가락이 물체 옆구리를 친다.
    """
    out = PoseStamped()
    out.header = grasp.header
    out.pose = translated(grasp.pose, quat_axis_z(grasp.pose.orientation), -abs(back_off_m))
    return out


def lifted(grasp: PoseStamped, up_m: float) -> PoseStamped:
    """월드(base_link) +Z 로 들어올린 포즈. 여기는 접근축이 아니라 **중력 반대**가 맞다."""
    out = PoseStamped()
    out.header = grasp.header
    out.pose = translated(grasp.pose, (0.0, 0.0, 1.0), abs(up_m))
    return out


def tcp_of(grasp: PoseStamped, tcp_offset_m: float) -> tuple[float, float, float]:
    """손끝 위치. grasp 원점 + offset x (+Z축).

    grasp 원점은 그리퍼 base 라 **물체 위치가 아니다.** 접근이 30° 기울면 원점은
    물체에서 9 cm 옆에 앉는다 (2026-08-05 에 이걸 "좌표 오차"로 오진한 이력).
    로그·CollisionObject 배치에는 이쪽을 쓴다.
    """
    a = quat_axis_z(grasp.pose.orientation)
    p = grasp.pose.position
    return (p.x + a[0] * tcp_offset_m, p.y + a[1] * tcp_offset_m, p.z + a[2] * tcp_offset_m)


def reach_of(pose: PoseStamped) -> float:
    """base_link 원점에서의 거리 [m]. 도달범위 사전 판정용."""
    p = pose.pose.position
    return math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)


def deg2rad(values) -> list[float]:
    """두산 펜던트/`robot_control` 은 도(deg), MoveIt JointConstraint 는 라디안이다."""
    return [math.radians(float(v)) for v in values]
