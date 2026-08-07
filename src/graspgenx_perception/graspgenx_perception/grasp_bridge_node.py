#!/usr/bin/env python3
"""그립 명령 하나로 촬영→세그멘테이션→GraspGenX→선택까지 도는 ROS 노드.

    ros2 service call /grasp/compute std_srvs/srv/Trigger

한 번 부를 때마다:
    1. depth N프레임 수집 → 픽셀별 중앙값 병합
    2. base 프레임 작업공간 박스로 세그멘테이션(신경망 0개)
    3. 씬 4파일을 임시 디렉토리에 쓰고 **GPU 워커**에 경로 한 줄 전달
    4. 워커가 준 grasp(=base_link 프레임)를 도달·접근축·점수로 거르고 1개 선택
    5. `/grasp/best`(PoseStamped) + `/grasp/candidates`(PoseArray) 발행,
       서비스 응답 message 에 요약

왜 워커가 별도 프로세스인가:
    GraspGenX 는 torch 가 있는 uv venv 에서만 import 되고, rclpy 는 시스템 파이썬이다.
    이 워크스페이스는 **ROS 노드에 venv 를 쓰지 않는다** — 무거운 ML 은 격리하고 통신한다.
    워커는 모델을 한 번만 올려두므로 요청당 비용은 추론 시간뿐이다.

⚠️ 이 노드는 로봇을 움직이지 않는다. grasp 포즈를 계산해 발행할 뿐이다.
   실행은 MoveIt 쪽에서 사람이 승인한 뒤에 한다.
"""

import json
import os
import subprocess
import tempfile
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from graspgenx_perception.capture_graspgenx_scene import SceneCapture, segment, write_scene

EXTRA_DEFAULTS = {
    # 워커
    'graspgenx_root': os.path.expanduser('~/cobot2_ws/isaac_ros-dev/src/GraspGenX'),
    'worker_script': os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'graspgen_worker.py'),
    'gripper': 'onrobot_RG2',
    'worker_timeout_sec': 60.0,
    # 선택 정책 (계획서 §1-3 5번). 전부 실측 튜닝값이다
    # GraspGenX grasp 의 원점은 **그리퍼 base** 이고 손끝(TCP)은 +Z 로 이만큼 떨어져 있다
    # (RG2 config.json "fingertip"[2] = 0.18). 접근이 30° 기울면 base 는 물체에서 9cm 옆에
    # 앉는다 — 2026-08-05 에 이걸 "좌표 오차"로 오진했다. 그래서 TCP 를 같이 발행·기록한다.
    'tcp_offset_m': 0.18,
    'min_score': 0.5,
    'max_reach_m': 0.900,       # M0609 도달. constraints.md 의 URDF 실측
    'max_approach_z': -0.3,     # grasp +Z(접근축)가 아래를 향하는 것만: R[2,2] < 이 값
    'target': '',               # 비우면 점수 최고 물체. 이름 지정은 YOLO 붙인 뒤
}


def pose_msg(T):
    """4x4 -> geometry_msgs/Pose. 회전은 행렬 -> 쿼터니언(Shepperd)."""
    m, p = T[:3, :3], Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
        float(x), float(y), float(z), float(w))
    return p


def tcp_of(g, offset):
    """grasp 4x4 -> 손끝 위치. TCP = 원점 + offset * (grasp 의 +Z축).

    GraspGenX 의 grasp 원점은 그리퍼 base 다(`robot.py:59`, constraints.md).
    **이걸 물체 위치로 읽으면 안 된다** — 접근이 기울수록 옆으로 벗어난다.
    """
    g = np.asarray(g, dtype=float).reshape(-1, 4, 4)
    return g[:, :3, 3] + offset * g[:, :3, 2]


def select(objects, p):
    """워커 결과 -> (label, T, score, 진단문자열). 없으면 (None, None, None, 이유).

    grasp 는 이미 base_link 프레임이다(씬의 camera_pose 를 로더가 적용한다).
    """
    lines, best = [], None
    for label, d in sorted(objects.items()):
        g = np.asarray(d['grasps'], dtype=float).reshape(-1, 4, 4)
        s = np.asarray(d['scores'], dtype=float)
        if len(g) == 0:
            lines.append(f'  {label}: 후보 0')
            continue
        ok = s >= p['min_score']
        reach = np.linalg.norm(g[:, :3, 3], axis=1)
        ok &= reach < p['max_reach_m']
        ok &= g[:, 2, 2] < p['max_approach_z']     # +Z 접근축이 아래를 향하는 것만
        # 위치를 같이 찍는다 — 이게 없으면 "엉뚱한 물체를 골랐다"를 눈치챌 방법이 없다.
        # 2026-08-05: 사과가 bounds 밖이라 다른 덩어리가 선택됐는데 개수만 보고는 못 알아챘다.
        # base 가 아니라 **손끝**을 찍는다. base 는 접근 각도에 따라 물체에서 크게 벗어난다.
        c = tcp_of(g, p['tcp_offset_m']).mean(axis=0)
        lines.append(f'  {label}: {len(g)}개 -> 점수 {int((s >= p["min_score"]).sum())} '
                     f'-> 도달 {int((reach < p["max_reach_m"]).sum())} '
                     f'-> 접근축 {int(ok.sum())} 통과  '
                     f'손끝≈({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})')
        if not ok.any():
            continue
        i = int(np.argmax(np.where(ok, s, -np.inf)))
        if p['target'] and label != p['target']:
            continue
        if best is None or s[i] > best[2]:
            best = (label, g[i], float(s[i]), g[ok], s[ok])
    if best is None:
        return None, None, None, None, None, '\n'.join(lines) or '물체 없음'
    return (*best, '\n'.join(lines))


class GraspBridge(SceneCapture):
    def __init__(self):
        super().__init__()
        for k, v in EXTRA_DEFAULTS.items():
            self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
        self.lock = threading.Lock()
        self.worker = None
        cb = ReentrantCallbackGroup()
        self.pub_best = self.create_publisher(PoseStamped, '/grasp/best', 10)
        # RViz 육안 확인용. /grasp/best 는 그리퍼 base 라 물체 옆에 뜬다 — 이쪽이 물체 위다
        self.pub_tcp = self.create_publisher(PoseStamped, '/grasp/best_tcp', 10)
        self.pub_all = self.create_publisher(PoseArray, '/grasp/candidates', 10)
        self.srv = self.create_service(Trigger, '/grasp/compute', self.on_compute,
                                       callback_group=cb)
        self.get_logger().info('준비됨 — ros2 service call /grasp/compute std_srvs/srv/Trigger')

    def extra(self):
        out = dict(self.params())
        for k, default in EXTRA_DEFAULTS.items():
            v = self.get_parameter(k).value
            out[k] = str(v) if isinstance(default, str) else type(default)(v)
        return out

    def start_worker(self, p):
        """모델 로딩은 여기서 한 번만. ready 한 줄을 기다린다."""
        if self.worker and self.worker.poll() is None:
            return True
        cmd = ['uv', 'run', '--no-sync', 'python', p['worker_script'],
               '--gripper', p['gripper']]
        self.get_logger().info(f"워커 기동(모델 로딩 수십 초): {' '.join(cmd)}")
        self.worker = subprocess.Popen(
            cmd, cwd=p['graspgenx_root'], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=None, text=True, bufsize=1)
        line = self.worker.stdout.readline()
        try:
            ok = json.loads(line).get('ready') is True
        except json.JSONDecodeError:
            ok = False
        if not ok:
            self.get_logger().error(f'워커 기동 실패 (첫 줄: {line!r}) — stderr 를 확인한다')
            return False
        self.get_logger().info('워커 준비 완료')
        return True

    def ask_worker(self, scene_dir, timeout):
        self.worker.stdin.write(scene_dir + '\n')
        self.worker.stdin.flush()
        box = {}
        t = threading.Thread(target=lambda: box.setdefault('l', self.worker.stdout.readline()))
        t.daemon = True
        t.start()
        t.join(timeout)
        if 'l' not in box:
            raise TimeoutError(f'워커 응답 {timeout}s 초과')
        return json.loads(box['l'])

    def on_compute(self, request, response):
        if not self.lock.acquire(blocking=False):
            response.success, response.message = False, '이미 계산 중이다'
            return response
        try:
            response.success, response.message = self.compute()
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'{type(e).__name__}: {e}')
            response.success, response.message = False, f'{type(e).__name__}: {e}'
        finally:
            self.lock.release()
        return response

    def compute(self):
        p = self.extra()
        if not self.start_worker(p):
            return False, '워커 기동 실패'

        # 1) 최신 프레임만 쓴다 — 이전 요청 때 쌓인 것을 섞지 않는다
        self.depths.clear()
        n = max(1, p['frames'])
        # ⚠️ 여기서 rclpy.spin_once 를 부르면 안 된다 — 이미 executor 가 이 노드를 돌리고 있고
        #    콜백 안에서 다시 spin 하면 재진입으로 엉킨다. 구독 콜백은 다른 스레드에서
        #    (서비스만 ReentrantCallbackGroup, 구독은 기본 그룹) 채워주므로 기다리기만 한다.
        end = time.monotonic() + p['timeout_sec']
        while rclpy.ok() and not self.ready(n, p) and time.monotonic() < end:
            time.sleep(0.02)
        if not self.ready(n, p):
            return False, (f'수신 실패 (depth {len(self.depths)}/{n}, '
                           f'color={self.color is not None}, info={self.K is not None}, '
                           f'tf={self.tf_ready(p)})')

        T_base_cam = self.camera_pose()
        if T_base_cam is None:
            return False, 'TF lookup 실패'
        depth, _ = self.merged_depth(n, p['min_valid_ratio'])
        if self.info_wh != (depth.shape[1], depth.shape[0]):
            return False, f'camera_info {self.info_wh} != depth {depth.shape[::-1]}'

        # 2) 세그멘테이션
        # seg_source='yolo' 면 SceneCapture 가 구독해 둔 /yolo_seg/labels 최신값을 쓴다
        seg, label_map, diag = segment(depth, self.K, T_base_cam, p,
                                       getattr(self, 'yolo_labels', None))
        if seg is None:
            return False, diag
        self.get_logger().info(diag)

        # 3) 워커 호출. 씬은 임시 디렉토리 — 남기고 싶으면 out_dir 파라미터를 준다
        # ⚠️ 워커에 넘기는 경로는 **반드시 절대경로**다. 이 노드의 cwd 는 사용자가 ros2 를
        #    실행한 곳이고 워커의 cwd 는 graspgenx_root 다(start_worker). 상대경로를 넘기면
        #    워커가 <graspgenx_root>/output/00 을 찾다가
        #    "FileNotFoundError: No meta_data.json in output/00" 으로 죽는다 (2026-08-07).
        tmp = None
        if p['out_dir']:
            scene_dir = os.path.abspath(
                os.path.expanduser(os.path.join(p['out_dir'], p['scene'])))
        else:
            tmp = tempfile.TemporaryDirectory(prefix='graspgen_scene_')
            scene_dir = os.path.join(tmp.name, '00')
        self.get_logger().info(f'씬 저장: {scene_dir}')
        try:
            write_scene(scene_dir, depth, self.color, seg, self.K, T_base_cam, label_map,
                        [p['x_min'], p['y_min'], p['z_min'],
                         p['x_max'], p['y_max'], p['z_max']])
            res = self.ask_worker(scene_dir, p['worker_timeout_sec'])
        finally:
            if tmp is not None:
                tmp.cleanup()
        if not res.get('ok'):
            return False, f"워커 오류: {res.get('error')}"

        # 4) 선택
        label, T, score, g_ok, _s_ok, sel_diag = select(res.get('objects', {}), p)
        self.get_logger().info(f'선택 단계:\n{sel_diag}')
        if label is None:
            return False, f'조건을 통과한 grasp 0개\n{sel_diag}'

        # 5) 발행
        stamp = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = stamp, p['base_frame']
        msg.pose = pose_msg(T)
        self.pub_best.publish(msg)
        T_tcp = T.copy()
        T_tcp[:3, 3] = tcp_of(T, p['tcp_offset_m'])[0]
        tcp_msg = PoseStamped()
        tcp_msg.header, tcp_msg.pose = msg.header, pose_msg(T_tcp)
        self.pub_tcp.publish(tcp_msg)
        arr = PoseArray()
        arr.header = msg.header
        arr.poses = [pose_msg(g) for g in g_ok]
        self.pub_all.publish(arr)

        t = T[:3, 3]
        tcp = tcp_of(T, p['tcp_offset_m'])[0]
        return True, (f'{label} 선택: score={score:.3f}, '
                      f'**손끝**=({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f}) {p["base_frame"]}, '
                      f'그리퍼base=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}), '
                      f'접근축 z={T[2, 2]:+.2f}, 후보 {len(g_ok)}개\n{sel_diag}')

    def destroy_node(self):
        if self.worker and self.worker.poll() is None:
            try:
                self.worker.stdin.close()
                self.worker.wait(timeout=5)
            except Exception:                                    # noqa: BLE001
                self.worker.kill()
        super().destroy_node()


def main():
    rclpy.init()
    node = GraspBridge()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
