#!/usr/bin/env python3
"""RealSense 한 프레임을 GraspGenX의 real_world 포맷(파일 4개)으로 떨어뜨린다.

왜 노드가 아니라 파일인가:
  `demo_scene_pc.py`가 M2T2 real_world 디렉토리를 그대로 먹는다
  (scene_loaders.py:50 load_realworld_scene). 즉 ROS 배선을 하기 전에
  **패키지가 제공하는 데모 스크립트 그대로** 실기 데이터를 검증할 수 있다.
  브리지 노드는 이 경로가 뚫린 다음에 짠다.

출력 (loader가 요구하는 이름 그대로):
  <out_dir>/<scene>/depth.npy       float32, **미터**, (H,W)
  <out_dir>/<scene>/rgb.png
  <out_dir>/<scene>/seg.png         uint8 라벨맵
  <out_dir>/<scene>/meta_data.json  intrinsics 3x3 / camera_pose 4x4 / label_map / scene_bounds

camera_pose 는 tf2 lookup(base_link <- camera_color_optical_frame)이다.
  - **npy 직접 로드 금지** — 2026-08-02에 OpenCV optical 규약 때문에 90° 틀어진 이력이 있다.
  - loader가 이 4x4로 점을 world로 보내므로(scene_loaders.py:86), 이걸 넣으면
    GraspGenX가 말하는 "world"가 곧 base_link 가 된다.
  - 마스크가 **컬러 정렬** depth 기준이라 부모는 camera_color_optical_frame 이다(depth optical 아님).

세그멘테이션(seg.png)은 신경망 0개다:
  base 프레임 작업공간 박스 안 + 테이블면보다 obj_min_h 이상 높은 픽셀 →
  cv2.connectedComponents 로 덩어리 분리. 붙어 있는 물체는 하나로 뭉친다.
  ponytail: YOLO-seg/DBSCAN 은 이 경로가 grasp를 내는 걸 본 뒤에 붙인다.
            물체를 서로 떨어뜨려 놓으면 이걸로 충분하다.

⚠️ self-filter 가 없다. 로봇 팔이 작업공간 박스 안에 들어와 있으면 물체로 잡힌다
   — 캡처할 때 팔을 박스 밖으로 치워두거나 bounds 로 잘라낸다.
"""

import json
import os

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

# 전부 실측 튜닝값이다. 도면값 아님 — 첫 캡처 로그를 보고 다시 잡는다.
DEFAULTS = {
    'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
    'info_topic': '/camera/camera/aligned_depth_to_color/camera_info',
    'color_topic': '/camera/camera/color/image_raw',
    'base_frame': 'base_link',
    'camera_frame': 'camera_color_optical_frame',
    'out_dir': '',            # 비우면 <repo>/data/graspgenx_scene
    'scene': '00',            # <out_dir>/<scene>/ 아래로 쓴다
    'use_tf': True,           # False면 camera_pose=단위행렬 (world=카메라 프레임)
    # base_link 기준 작업공간 박스 [m]. 로봇 베이스가 테이블에 앉아 있다고 보고 잡은 값 — UNVERIFIED
    'x_min': 0.10, 'x_max': 0.90,
    'y_min': -0.50, 'y_max': 0.50,
    'z_min': -0.05, 'z_max': 0.60,
    'table_z': float('nan'),  # nan이면 박스 안 z 중앙값으로 자동 추정
    'obj_min_h': 0.015,       # 테이블면 위 이 높이부터 물체로 본다
    'min_pixels': 300,        # 이보다 작은 덩어리는 버린다(노이즈)
    'timeout_sec': 10.0,
}

LABEL_TABLE = 2
LABEL_OBJ_BASE = 100          # obj_1 -> 101, obj_2 -> 102 ... (샘플 데이터와 같은 규약)


def quat_to_matrix(x, y, z, w):
    """쿼터니언 -> 3x3. tf_transformations 가 이 랩탑에 없어 직접 만든다."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


class SceneCapture(Node):
    def __init__(self):
        super().__init__('graspgenx_scene_capture')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.bridge = CvBridge()
        self.depth = None
        self.color = None
        self.color_is_raw = False   # raw가 오면 compressed 를 덮어쓰지 않는다
        self.K = None

        p = self.get_parameter
        # 정지한 장면을 한 컷 뜨는 것이라 exact sync 를 쓰지 않는다. 각 토픽의 최신값을 쓴다.
        self.create_subscription(
            Image, p('depth_topic').value, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Image, p('color_topic').value, self._on_color, qos_profile_sensor_data)
        # bag 은 컬러가 compressed 로만 녹화돼 있었다(constraints.md). 라이브 런치도 raw 를
        # 안 낼 수 있으므로 둘 다 구독하고 raw 를 우선한다 — 없는 토픽 구독은 무해하다.
        self.create_subscription(
            CompressedImage, p('color_topic').value + '/compressed',
            self._on_color_compressed, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, p('info_topic').value, self._on_info, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_depth(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # 16UC1 은 mm, 32FC1 은 이미 m. 이 나눗셈을 빠뜨리면 로봇이 1.6 km 밖을 향한다.
        self.depth = (img.astype(np.float32) / 1000.0
                      if img.dtype == np.uint16 else img.astype(np.float32))

    def _on_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.color_is_raw = True

    def _on_color_compressed(self, msg):
        if self.color_is_raw:
            return
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            self.color = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def _on_info(self, msg):
        self.K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)

    def ready(self):
        return self.depth is not None and self.color is not None and self.K is not None

    def camera_pose(self):
        """base_frame <- camera_frame 4x4. 실패하면 None."""
        p = self.get_parameter
        if not p('use_tf').value:
            self.get_logger().warn('use_tf=False — camera_pose 를 단위행렬로 쓴다 (world=카메라)')
            return np.eye(4)
        try:
            tf = self.tf_buffer.lookup_transform(
                p('base_frame').value, p('camera_frame').value,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3.0))
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'TF lookup 실패: {e}')
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T


def segment(depth, K, T_base_cam, p):
    """작업공간 박스 + 높이 임계 + connectedComponents -> (seg uint8, label_map, 진단문자열)."""
    H, W = depth.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    # scene_loaders.depth_to_camera_xyz:24 와 같은 규약(optical: x 오른쪽, y 아래, z 전방)
    xyz_cam = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], axis=-1)
    xyz = xyz_cam @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]
    X, Y, Z = xyz[..., 0], xyz[..., 1], xyz[..., 2]

    valid = depth > 0
    in_box = (valid
              & (X >= p['x_min']) & (X <= p['x_max'])
              & (Y >= p['y_min']) & (Y <= p['y_max'])
              & (Z >= p['z_min']) & (Z <= p['z_max']))
    if not in_box.any():
        return None, None, '작업공간 박스 안에 유효 depth 가 0개다 — bounds 나 TF 를 의심한다'

    table_z = p['table_z']
    auto = not np.isfinite(table_z)
    if auto:
        table_z = float(np.median(Z[in_box]))

    obj = in_box & (Z > table_z + p['obj_min_h'])
    n, comp = cv2.connectedComponents(obj.astype(np.uint8), connectivity=8)

    seg = np.zeros((H, W), np.uint8)
    seg[in_box & ~obj] = LABEL_TABLE
    label_map = {'ground': 0, 'table': LABEL_TABLE}
    kept = []
    for c in range(1, n):
        m = comp == c
        px = int(m.sum())
        if px < p['min_pixels']:
            continue
        idx = len(kept) + 1
        seg[m] = LABEL_OBJ_BASE + idx
        label_map[f'obj_{idx}'] = LABEL_OBJ_BASE + idx
        kept.append((idx, px, float(Z[m].max() - table_z)))

    diag = [f"table_z={table_z:.4f} m ({'자동추정' if auto else '지정'}), "
            f"박스 안 픽셀 {int(in_box.sum())}, 물체 후보 덩어리 {n - 1}개 중 {len(kept)}개 채택"]
    for idx, px, h in kept:
        diag.append(f'  obj_{idx}: {px} px, 테이블 위 최대높이 {h * 100:.1f} cm')
    if not kept:
        diag.append('  ⛔ 채택 0개 — obj_min_h 를 낮추거나 min_pixels 를 줄이거나 bounds 를 확인한다')
    return seg, label_map, '\n'.join(diag)


def write_scene(out, depth, color_rgb, seg, K, T, label_map, bounds):
    """loader가 요구하는 파일 4개를 쓴다. 카메라 없이도 테스트할 수 있게 분리했다."""
    os.makedirs(out, exist_ok=True)
    np.save(os.path.join(out, 'depth.npy'), depth.astype(np.float32))
    cv2.imwrite(os.path.join(out, 'rgb.png'), cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(os.path.join(out, 'seg.png'), seg)
    with open(os.path.join(out, 'meta_data.json'), 'w') as f:
        json.dump({
            'intrinsics': np.asarray(K).tolist(),
            'camera_pose': np.asarray(T).tolist(),
            'label_map': label_map,
            'scene_bounds': list(bounds),
        }, f, indent=2)
    return out


def main():
    rclpy.init()
    node = SceneCapture()
    p = {k: node.get_parameter(k).value for k in DEFAULTS}

    end = node.get_clock().now() + rclpy.duration.Duration(seconds=p['timeout_sec'])
    while rclpy.ok() and not node.ready() and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready():
        node.get_logger().error(
            f"토픽 수신 실패 (depth={node.depth is not None}, color={node.color is not None}, "
            f"info={node.K is not None}) — 카메라 런치와 ROS_DOMAIN_ID 를 확인한다")
        rclpy.shutdown()
        return 1

    T = node.camera_pose()
    if T is None:
        rclpy.shutdown()
        return 1

    depth, color, K = node.depth, node.color, node.K
    if color.shape[:2] != depth.shape[:2]:
        node.get_logger().error(
            f'컬러 {color.shape[:2]} != depth {depth.shape[:2]} — '
            'aligned_depth_to_color 토픽이 맞는지 확인한다')
        rclpy.shutdown()
        return 1

    seg, label_map, diag = segment(depth, K, T, p)
    if seg is None:
        node.get_logger().error(diag)
        rclpy.shutdown()
        return 1

    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(p['out_dir'] or os.path.join(repo, 'data', 'graspgenx_scene'),
                       p['scene'])
    write_scene(out, depth, color, seg, K, T, label_map,
                [p['x_min'], p['y_min'], p['z_min'],
                 p['x_max'], p['y_max'], p['z_max']])

    node.get_logger().info(
        f"{diag}\n저장: {out}\n"
        f"  depth {depth.shape} {depth.dtype} 범위 {depth[depth > 0].min():.3f}~{depth.max():.3f} m\n"
        f"  camera_pose t={np.round(T[:3, 3], 4).tolist()}\n"
        f"다음: cd isaac_ros-dev/src/GraspGenX && uv run python scripts/demo_scene_pc.py "
        f"--sample_data_dir {os.path.dirname(out)} --gripper_name onrobot_RG2 --num_grasps 64")
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
