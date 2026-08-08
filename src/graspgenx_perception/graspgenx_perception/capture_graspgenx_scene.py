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
  <out_dir>/<scene>/seg.png         uint8 라벨맵cd /workspaces/isaac_ros-dev
  colcon build --symlink-install \
    --packages-up-to isaac_ros_cumotion_moveit isaac_ros_cumotion_robot_description
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
import warnings

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

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
    # 로봇 팔·그리퍼는 작업공간 박스 **안**에 있어 xy 로 뺄 수 없다 — 높이로 자른다.
    # 0.12 는 2026-08-08 실측 튜닝값이다(씬 cmp_geo): 그리퍼가 상판 위 17.9~33.0 cm 에
    # 걸쳐 4182 px 짜리 obj_1 로 잡혔고, 같은 씬의 사과는 상판 위 최대 7.2 cm 였다.
    # 그 사이를 자르면 그리퍼만 사라진다. 기하 경로 전용이다 — yolo 경로(segment_from_labels)
    # 는 이 값을 안 쓴다.
    # ⚠️ 그래서 **yolo 경로에는 self-filter 가 없다.** COCO 0 = person 이고 README 실측에
    # person 검출이 기록돼 있다 — `classes` 기본값이 전체라 사람·팔이 obj_N 으로 들어온다.
    # yolo 경로에서 이걸 막는 수단은 `classes`(탐지 제한)와 `target_classes`(연산 제한)뿐이다.
    # ⚠️ 키가 12 cm 를 넘는 물체도 같이 잘린다. 그런 물체를 잡으려면 올릴 것. nan 이면 무제한.
    'obj_max_h': 0.12,
    'min_pixels': 300,        # 이보다 작은 덩어리는 버린다(노이즈)
    # 단일 시점 depth 는 물체 뒤 가림영역을 전경 깊이로 메운다(occlusion shadow).
    # 그 꼬리가 덩어리에 붙어 OBB 를 부풀리고, 심하면 grasp 가 0개가 된다
    # (2026-08-05: 사과 8cm 가 12.6x18.8cm 로 잡혀 후보 0). 덩어리 중앙에서 이 반경 밖을 자른다.
    # RG2 개구가 0.102 m 라 이보다 큰 물체는 어차피 못 잡는다. nan 이면 끄기.
    # 0.05 는 씬 10(사과) 실측 튜닝값이다: nan→8개, 0.07→4개, **0.05→32개**, 0.04→1개.
    # 너무 조이면 OBB 가 작아져 후보가 사라진다. 씬 하나로 잡은 값이니 다른 물체에선 다시 본다.
    'obj_radius_m': 0.05,
    'frames': 10,             # depth 를 이만큼 모아 픽셀별 중앙값. 정지 장면이라 가능하다
    'min_valid_ratio': 0.5,   # 프레임 중 이 비율 이상에서 유효해야 그 픽셀을 쓴다
    'timeout_sec': 10.0,
    # 세그멘테이션 백엔드. 'geometric' = 작업공간 박스 + connectedComponents (신경망 0개),
    # 'yolo' = yolo_seg 노드가 내는 라벨맵을 그대로 쓴다. 라벨 규약(101,102,...)이 같아
    # 변환이 필요 없다. yolo 는 학습한 클래스만 잡으므로 공구 seg 모델이 없으면 geometric 이 낫다.
    # 2026-08-08: 잡을 물체를 target_classes 로 지정해 하나씩 돌리는 운용으로 확정 —
    # geometric 은 클래스를 모른다(위 target_classes 주석 참고). 기본값을 yolo 로 바꿨다.
    'seg_source': 'yolo',
    'label_topic': '/yolo_seg/labels',
    # yolo_seg_node 가 내는 "라벨값 -> 클래스 이름" 매핑. target_classes 필터가 이걸 쓴다.
    'class_topic': '/yolo_seg/classes',
}

# LABEL_TABLE 은 **사람용**이다. GraspGenX 로더는 `obj_` 접두어가 아닌 라벨을 전부 무시하고
# (scene_loaders.py:95), 씬 점군에는 라벨과 무관하게 유효 depth 가 전부 들어간다.
# 즉 라벨 2와 라벨 0은 GraspGenX 에게 동일하다 — 오버레이로 "박스가 상판을 덮었나"를 눈으로
# 확인하려고 남긴다. 지우면 디버깅 수단이 사라진다.
LABEL_TABLE = 2
LABEL_OBJ_BASE = 100          # obj_1 -> 101, obj_2 -> 102 ... (샘플 데이터와 같은 규약)
MAX_OBJECTS = 155             # seg.png 가 uint8 이라 100+156 은 조용히 0으로 랩어라운드한다
MAX_DEPTH_BUFFER = 120        # 실패 경로에서 depth 프레임이 무한히 쌓이는 것을 막는다


def stamp_ns(stamp):
    """builtin_interfaces/Time -> int ns. 라벨맵과 클래스맵을 짝짓는 유일한 키다."""
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


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
        # dynamic_typing: `-p scene:=00` 은 YAML 이 정수 0 으로 파싱해 STRING 선언과 충돌한다.
        # `-p obj_min_h:=0` (DOUBLE 선언에 INTEGER) 도 같은 예외로 죽는다.
        # 타입은 선언이 아니라 params() 에서 DEFAULTS 기준으로 맞춘다.
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
        self.bridge = CvBridge()
        self.depths = []            # 프레임별 원본. 마지막에 픽셀별 중앙값으로 합친다
        self.color = None
        self.color_is_raw = False   # raw가 오면 compressed 를 덮어쓰지 않는다
        self.K = None
        self.info_frame = None
        self.info_wh = None

        p = self.params()
        # 정지한 장면을 한 컷 뜨는 것이라 exact sync 를 쓰지 않는다. 각 토픽의 최신값을 쓴다.
        self.create_subscription(
            Image, p['depth_topic'], self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Image, p['color_topic'], self._on_color, qos_profile_sensor_data)
        # bag 은 컬러가 compressed 로만 녹화돼 있었다(constraints.md). 라이브 런치도 raw 를
        # 안 낼 수 있으므로 둘 다 구독하고 raw 를 우선한다 — 없는 토픽 구독은 무해하다.
        self.create_subscription(
            CompressedImage, p['color_topic'] + '/compressed',
            self._on_color_compressed, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, p['info_topic'], self._on_info, qos_profile_sensor_data)
        # seg_source='yolo' 일 때만 쓴다. 없는 토픽 구독은 무해하므로 항상 걸어둔다.
        self.yolo_labels_history = []   # [(stamp_ns, 라벨맵)] — best_labels() 가 최고를 고른다
        # stamp_ns -> {라벨값: 클래스 이름}. **이 클래스 자체는 안 쓴다** — grasp_bridge_node
        # 의 target_classes 가 쓴다. 이 노드를 CLI 로 단독 실행하면 클래스 필터는 없다.
        self.yolo_classes = {}
        self.create_subscription(
            Image, p['label_topic'], self._on_labels, qos_profile_sensor_data)
        self.create_subscription(
            String, p['class_topic'], self._on_classes, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_labels(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.yolo_labels_history.append((stamp_ns(msg.header.stamp), img))
        # depth 버퍼와 같은 상한을 재사용한다 — 실패 경로에서 무한히 쌓이는 것을 막는 목적이 같다.
        if len(self.yolo_labels_history) > MAX_DEPTH_BUFFER:
            del self.yolo_labels_history[:-MAX_DEPTH_BUFFER]

    def _on_classes(self, msg):
        """라벨맵과 **다른 토픽**이라 최신값 하나만 들면 짝이 어긋난다 — stamp 로 보관한다.

        `std_msgs/String` 은 범용 타입이라 아무나 이 토픽에 아무 문자열이나 발행할 수 있다.
        형태가 어긋난 것 하나로 구독 콜백이 죽으면 노드가 통째로 내려가므로 넓게 잡는다.
        """
        try:
            d = json.loads(msg.data)
            parsed = {int(o['label']): str(o['class']) for o in d['objects']}
            stamp = int(d['stamp_ns'])
        except (ValueError, TypeError, KeyError, AttributeError):
            return                       # 프레임 하나 버린다. 라벨맵 경로는 영향받지 않는다
        self.yolo_classes[stamp] = parsed
        if len(self.yolo_classes) > MAX_DEPTH_BUFFER:
            for k in list(self.yolo_classes)[:-MAX_DEPTH_BUFFER]:
                del self.yolo_classes[k]

    def _on_depth(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # 16UC1 은 mm, 32FC1 은 이미 m. 이 나눗셈을 빠뜨리면 로봇이 1.6 km 밖을 향한다.
        self.depths.append(img.astype(np.float32) / 1000.0
                           if img.dtype == np.uint16 else img.astype(np.float32))
        # color/info 가 안 오면 ready() 가 영원히 False 라 타임아웃까지 쌓인다(1280x720 이면 GB 단위).
        if len(self.depths) > MAX_DEPTH_BUFFER:
            del self.depths[:-MAX_DEPTH_BUFFER]

    def merged_depth(self, n_frames, min_valid_ratio):
        """픽셀별 중앙값. D435i 는 밝고 무늬 없는 상판에서 프레임마다 다른 곳이 튄다 —
        단일 프레임을 쓰면 그 노이즈가 그대로 '물체'가 된다. 장면이 정지해 있으므로
        시간축 중앙값이 가장 싼 해법이다(구멍 메우기 + 잡음 제거를 동시에)."""
        st = np.stack(self.depths[-n_frames:])
        valid = st > 0
        cnt = valid.sum(0)
        with warnings.catch_warnings():
            # 전 프레임이 0인 픽셀은 All-NaN 이 정상이다(바로 아래에서 0으로 만든다)
            warnings.simplefilter('ignore', category=RuntimeWarning)
            med = np.nanmedian(np.where(valid, st, np.nan), axis=0)
        med[~np.isfinite(med)] = 0.0
        med[cnt < max(1, int(round(min_valid_ratio * len(st))))] = 0.0
        return med.astype(np.float32), len(st)

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
        # 잘못된 camera_info(예: depth/camera_info)를 물리면 fx 가 ~640 vs ~900 으로 다른데
        # 아무 에러 없이 통과한다. 프레임 이름과 해상도를 뒤에서 대조하려고 들고 있는다.
        self.info_frame = msg.header.frame_id
        self.info_wh = (int(msg.width), int(msg.height))

    def params(self):
        """파라미터를 DEFAULTS 의 타입으로 강제한다. dynamic_typing 의 짝이다."""
        out = {}
        for k, default in DEFAULTS.items():
            val = self.get_parameter(k).value
            if isinstance(default, str):
                # scene:=00 -> 정수 0 으로 들어온다. 샘플 데이터 규약(00, 01)대로 두 자리로 되돌린다.
                out[k] = f'{val:02d}' if isinstance(val, int) and k == 'scene' else str(val)
            elif isinstance(default, bool):
                out[k] = bool(val)
            else:
                out[k] = type(default)(val)
        return out

    def tf_ready(self, p):
        """TF 를 대기 조건에 넣는다.

        `Buffer.lookup_transform(timeout=…)` 은 이 구성에서 **무효다**: TransformListener 를
        spin_thread 없이 만들었으므로 /tf_static 구독이 이 노드에 붙어 있고, Humble 의 Buffer 는
        타임아웃 동안 sleep 폴링만 할 뿐 executor 를 돌리지 않는다 → 그 3초 동안 새 TF 가
        들어올 수 없다. depth 10프레임(≈330ms)이 /tf_static 디스커버리보다 먼저 차면
        헛기다린 뒤 실패하고 **이미 찍은 프레임을 버린다.** 그래서 spin 루프 안에서 확인한다.
        """
        if not p['use_tf']:
            return True
        return self.tf_buffer.can_transform(
            p['base_frame'], p['camera_frame'], rclpy.time.Time())

    def ready(self, n_frames, p):
        return (len(self.depths) >= n_frames and self.color is not None
                and self.K is not None and self.tf_ready(p))

    def camera_pose(self):
        """base_frame <- camera_frame 4x4. 실패하면 None."""
        p = self.params()
        if not p['use_tf']:
            self.get_logger().warn(
                'use_tf=False — camera_pose 가 단위행렬이 된다(world=카메라 광학 프레임). '
                '⚠️ x_min/x_max/y_*/z_* 는 base 기준 값인데 그대로 카메라 좌표에 적용되므로 '
                '대개 "박스 안 유효 depth 0개"가 된다. bounds 도 카메라 기준으로 다시 줄 것.')
            return np.eye(4)
        try:
            tf = self.tf_buffer.lookup_transform(
                p['base_frame'], p['camera_frame'],
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3.0))
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(f'TF lookup 실패: {e}')
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T


def to_base(depth, K, T_base_cam):
    """(H,W) depth[m] + K + 4x4 -> (H,W,3) base 프레임 XYZ.

    GraspGenX 쪽 구현과 **반드시 같아야 한다**:
      scene_loaders.depth_to_camera_xyz:24-33 (optical: x 오른쪽, y 아래, z 전방)
      scene_loaders.transform_xyz:36-38      (xyz @ R.T + t)
    한 줄로 분리해 둔 이유는 테스트가 이 식만 따로 겨냥할 수 있게 하려는 것이다 —
    `R.T` 를 `R` 로 바꾸는 실수는 사과 위치를 통째로 옮기면서도 조용하다.
    """
    H, W = depth.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    xyz_cam = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], axis=-1)
    return xyz_cam @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]


def workspace_mask(depth, K, T_base_cam, p):
    """base 프레임 작업공간 박스 안의 유효 depth 픽셀 마스크. 기하/yolo 경로가 공유한다."""
    xyz = to_base(depth, K, T_base_cam)
    X, Y, Z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
    in_box = ((depth > 0)
              & (X >= p['x_min']) & (X <= p['x_max'])
              & (Y >= p['y_min']) & (Y <= p['y_max'])
              & (Z >= p['z_min']) & (Z <= p['z_max']))
    return in_box, X, Y, Z


def segment_from_labels(labels, depth, K, T_base_cam, p):
    """yolo_seg 라벨맵(101,102,...)을 그대로 씬 seg 로 쓴다.

    라벨 규약이 이미 같으므로 하는 일은 두 가지뿐이다:
      1. **유효 depth 로 마스킹.** GraspGenX 는 depth 역투영으로 점군을 만든다. depth 가 0 인
         픽셀에 라벨이 붙어 있으면 그 물체의 점 수만 부풀고 점군에는 안 들어간다.
      2. 남은 라벨을 label_map 으로 정리.
    기하 경로와 같은 작업공간 박스·반경 크롭을 적용한다. 역할 분담은
    **YOLO 가 "어느 물체인지", 기하가 "닿을 수 있는 곳인지"** 다.
    """
    if labels is None:
        return None, None, ('seg_source=yolo 인데 라벨맵을 못 받았다. '
                            'yolo_seg_node 가 떠 있는지, label_topic 이 맞는지 확인할 것')
    if labels.shape != depth.shape:
        return None, None, (f'라벨맵 {labels.shape} != depth {depth.shape}. '
                            'yolo_seg 의 image_topic 이 depth 와 같은 정렬 해상도인지 확인할 것')

    # 유효 depth **와 작업공간 박스**의 교집합만 남긴다.
    # 박스를 안 걸면 배경(바닥·벽·먼 물체)까지 라벨에 들어가 물체 점군이 폭발한다 —
    # 2026-08-06 에 GraspGenX 가 41.7GB 를 할당하려다 죽었다.
    # 역할 분담: **YOLO 는 "어느 물체인지", 기하는 "닿을 수 있는 곳인지"** 를 정한다.
    in_box, X, Y, _ = workspace_mask(depth, K, T_base_cam, p)
    if not in_box.any():
        return None, None, '작업공간 박스 안에 유효 depth 가 0개다 — bounds 나 TF 를 의심한다'
    seg = np.where(in_box, labels, 0).astype(np.uint8)
    label_map, kept = {}, []
    radius = p['obj_radius_m']
    for v in sorted(int(x) for x in np.unique(seg) if x > LABEL_OBJ_BASE):
        m = seg == v
        px = int(m.sum())
        if np.isfinite(radius) and px >= p['min_pixels']:
            # 기하 경로와 **같은 반경 크롭**. COCO 라벨은 dining table 처럼 화면의 상당 부분을
            # 한 인스턴스로 덮는데, 그 점군을 그대로 넘기면 GraspGenX 가 죽는다
            # (2026-08-06: 67,879 px 라벨에서 41.7GB 할당 시도).
            # RG2 개구가 0.102 m 라 반경 0.05 m 를 넘는 물체는 어차피 못 잡는다.
            mx, my = float(np.median(X[m])), float(np.median(Y[m]))
            m = m & (np.hypot(X - mx, Y - my) <= radius)
            seg[(seg == v) & ~m] = 0
            px = int(m.sum())
        if px < p['min_pixels']:
            seg[seg == v] = 0
            continue
        idx = v - LABEL_OBJ_BASE
        label_map[f'obj_{idx}'] = v
        kept.append((idx, px))

    diag = [f'yolo 라벨맵 사용: 박스 안 픽셀 {int(in_box.sum())}, '
            f'라벨 {len(kept)}개 채택 (min_pixels={p["min_pixels"]})']
    for idx, px in kept:
        diag.append(f'  obj_{idx}: {px:5d} px')
    if not kept:
        diag.append('  ⛔ 채택 0개 — yolo 가 아무것도 못 잡았거나 depth 와 겹치는 픽셀이 없다')
    return seg, label_map, '\n'.join(diag)


def best_labels(frames):
    """`[(stamp_ns, 라벨맵), ...]` 중 물체 픽셀(라벨값 > 100)이 가장 많은 것 -> 같은 튜플.

    grasp 연산(수십 초)에 비하면 프레임 몇 장 더 보는 비용은 무시할 만하다 — 탐지가
    한 프레임에서만 흔들려도(조명 반사, 순간 블러) 최선의 컷을 쓰게 한다.
    ponytail: 프레임을 픽셀별로 합치지 않는다(depth median과 다르게 라벨은 정수 클래스ID라
    두 프레임을 섞으면 의미 없는 값이 나온다) — "라벨 픽셀 수 최대"인 프레임을 그대로 쓴다.
    stamp 를 같이 돌려주는 이유: 클래스맵이 별도 토픽이라 **고른 프레임의** stamp 로
    찾아야 짝이 맞는다. 배열만 돌려주면 어느 프레임이었는지 되찾을 수 없다.
    """
    if not frames:
        return None, None
    return max(frames, key=lambda f: int((f[1] > LABEL_OBJ_BASE).sum()))


def filter_labels_by_class(labels, class_map, wanted):
    """`wanted` 클래스에 속하지 않는 라벨을 0 으로 지운다 -> (라벨맵, 진단문자열).

    **워커 호출 전에** 거른다. `select()` 의 `target` 은 이미 계산된 결과에서 고르는 것이라
    GraspGenX 가 물체 전부를 연산한 뒤다(물체당 수 초~수십 초). 여기서 지우면 그 연산
    자체가 사라진다 — "지정한 물체만 연산"의 의미가 이쪽이다.
    """
    keep, drop = [], []
    out = labels.copy()
    for v in (int(x) for x in np.unique(labels) if x > LABEL_OBJ_BASE):
        name = class_map.get(v)
        if name in wanted:
            keep.append(f'obj_{v - LABEL_OBJ_BASE}={name}')
        else:
            out[out == v] = 0
            drop.append(f'obj_{v - LABEL_OBJ_BASE}={name or "?"}')
    diag = (f'클래스 필터 target_classes={sorted(wanted)}: '
            f'남김 [{", ".join(keep) or "없음"}] / 버림 [{", ".join(drop) or "없음"}]')
    return out, diag


def segment(depth, K, T_base_cam, p, yolo_labels=None):
    """작업공간 박스 + 높이 임계 + connectedComponents -> (seg uint8, label_map, 진단문자열).

    p['seg_source'] == 'yolo' 면 기하 대신 yolo_labels 를 쓴다 (segment_from_labels).
    """
    if p.get('seg_source') == 'yolo':
        return segment_from_labels(yolo_labels, depth, K, T_base_cam, p)
    H, W = depth.shape
    in_box, X, Y, Z = workspace_mask(depth, K, T_base_cam, p)
    if not in_box.any():
        return None, None, '작업공간 박스 안에 유효 depth 가 0개다 — bounds 나 TF 를 의심한다'

    table_z = p['table_z']
    auto = not np.isfinite(table_z)
    if auto:
        table_z = float(np.median(Z[in_box]))

    obj = in_box & (Z > table_z + p['obj_min_h'])
    max_h = p['obj_max_h']
    if np.isfinite(max_h):
        # 로봇 팔은 작업공간 박스 안에 있어 xy 로 뺄 수 없다. 높이로 자른다.
        obj &= Z < table_z + max_h
    # 3x3 열림 — depth 가장자리의 한두 픽셀짜리 점들이 덩어리로 잡히는 걸 막는다
    obj = cv2.morphologyEx(obj.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
    n, comp = cv2.connectedComponents(obj.astype(np.uint8), connectivity=8)

    seg = np.zeros((H, W), np.uint8)
    # `~obj` 로 칠하면 obj_max_h 에 잘린 키 큰 것(로봇 팔·사람)까지 table 라벨을 받는다.
    # 실제로 table 라벨의 z 최대가 +0.376 m 로 찍혔다(2026-08-05). 낮은 것만 테이블이다.
    seg[in_box & (Z <= table_z + p['obj_min_h'])] = LABEL_TABLE
    label_map = {'ground': 0, 'table': LABEL_TABLE}
    kept = []
    radius = p['obj_radius_m']
    for c in range(1, n):
        m = comp == c
        px = int(m.sum())
        if px < p['min_pixels']:
            continue
        if np.isfinite(radius):
            # 중앙값 중심 — 꼬리가 붙어 있어도 평균보다 덜 끌려간다(2026-08-05 실측:
            # 꼬리 포함 덩어리의 중앙값이 실제 사과 중심에서 1.7cm, 평균은 그 이상).
            mx, my = float(np.median(X[m])), float(np.median(Y[m]))
            m &= np.hypot(X - mx, Y - my) <= radius
            px = int(m.sum())
            if px < p['min_pixels']:
                continue
        if len(kept) >= MAX_OBJECTS:      # uint8 랩어라운드 방지. 예외 없이 조용히 틀린다
            break
        idx = len(kept) + 1
        seg[m] = LABEL_OBJ_BASE + idx
        label_map[f'obj_{idx}'] = LABEL_OBJ_BASE + idx
        # 중심 좌표가 없으면 "어느 게 사과인지" 를 알 방법이 없다 — 튜닝의 유일한 근거다
        kept.append((idx, px, float(Z[m].max() - table_z),
                     float(X[m].mean()), float(Y[m].mean()), float(Z[m].mean())))

    diag = [f"table_z={table_z:.4f} m ({'자동추정' if auto else '지정'}), "
            f"박스 안 픽셀 {int(in_box.sum())}, 물체 후보 덩어리 {n - 1}개 중 {len(kept)}개 채택"]
    # ⚠️ "표면중심"이지 물체 중심이 아니다. 단일 시점이라 보이는 면만 평균되고,
    #    반지름 r 인 구라면 시선축을 따라 카메라 쪽으로 약 2r/3 만큼 앞에 앉는다
    #    (사과 r=2.5cm -> 1.7cm). **이 편차를 캘리브 오차로 오해해 보정하지 말 것.**
    for idx, px, h, ox, oy, oz in kept:
        diag.append(f'  obj_{idx}: {px:5d} px  표면중심 base=({ox:+.3f}, {oy:+.3f}, {oz:+.3f}) m  '
                    f'테이블 위 최대높이 {h * 100:.1f} cm')
    if not kept:
        diag.append('  ⛔ 채택 0개 — obj_min_h 를 낮추거나 min_pixels 를 줄이거나 bounds 를 확인한다')
    return seg, label_map, '\n'.join(diag)


def default_out_dir():
    """`out_dir` 파라미터가 비었을 때 쓰는 저장 위치.

    설치 경로가 계정마다 다를 수 있어(cobot2_ws 위치) 하드코딩하지 않는다 — 이 파일
    자신의 경로에서 두 단계 위로 올라가 패키지 소스 루트를 찾는다.

    ⚠️ `abspath`가 아니라 **`realpath`를 써야 한다.** `--symlink-install`(이 워크스페이스
    기본값)에서 `install/setup.bash`는 `build/<pkg>/<pkg>/`를 PYTHONPATH에 먼저 얹고,
    그 경로의 각 파일은 `src/`를 가리키는 심볼릭 링크다. `abspath(__file__)`는 심볼릭
    링크를 풀지 않으므로 결과가 `build/graspgenx_perception/data/graspgenx_scene`가
    되는데, 이 디렉토리는 `colcon build`/`rm -rf build`로 지워지는 임시 산출물이라
    저장한 씬이 조용히 사라진다(2026-08-07, `python3 -c` 로 직접 재현·확인). `realpath`로
    링크를 풀면 진짜 소스 트리(`src/graspgenx_perception`) 밑으로 저장된다.
    """
    repo = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    return os.path.join(repo, 'data', 'graspgenx_scene')


def write_scene(out, depth, color_rgb, seg, K, T, label_map, bounds):
    """loader가 요구하는 파일 4개를 쓴다. 카메라 없이도 테스트할 수 있게 분리했다."""
    os.makedirs(out, exist_ok=True)
    np.save(os.path.join(out, 'depth.npy'), depth.astype(np.float32))
    # imwrite 는 실패해도 예외 없이 False 만 준다 — 그냥 두면 파일 2개짜리 디렉토리를
    # 만들어 놓고 "저장 완료"라고 보고하고, 실패는 한참 뒤 GraspGenX 로더에서 터진다.
    for name, img in (('rgb.png', cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)),
                      ('seg.png', seg)):
        if not cv2.imwrite(os.path.join(out, name), img):
            raise IOError(f'{os.path.join(out, name)} 쓰기 실패')
    with open(os.path.join(out, 'meta_data.json'), 'w') as f:
        json.dump({
            'intrinsics': np.asarray(K).tolist(),
            'camera_pose': np.asarray(T).tolist(),
            'label_map': label_map,
            'scene_bounds': list(bounds),
        }, f, indent=2)
    return out


def run(node):
    """캡처 본체. 성공 0, 실패 1. 정리(shutdown)는 main 이 책임진다."""
    p = node.params()
    n_frames = max(1, p['frames'])
    end = node.get_clock().now() + rclpy.duration.Duration(seconds=p['timeout_sec'])
    while rclpy.ok() and not node.ready(n_frames, p) and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready(n_frames, p):
        node.get_logger().error(
            f"수신 실패 (depth {len(node.depths)}/{n_frames} 프레임, "
            f"color={node.color is not None}, info={node.K is not None}, "
            f"tf={node.tf_ready(p)}) — 카메라 런치와 ROS_DOMAIN_ID 를 확인한다. "
            f"tf 만 False 면 {p['base_frame']} <- {p['camera_frame']} 이 없는 것이다")
        return 1

    T = node.camera_pose()
    if T is None:
        return 1

    depth, used = node.merged_depth(n_frames, p['min_valid_ratio'])
    color, K = node.color, node.K
    node.get_logger().info(
        f'depth {used} 프레임 중앙값 병합 — 유효 픽셀 '
        f'{100.0 * (depth > 0).mean():.1f}% (단일 프레임 '
        f'{100.0 * (node.depths[-1] > 0).mean():.1f}%)')

    if color.shape[:2] != depth.shape[:2]:
        node.get_logger().error(
            f'컬러 {color.shape[:2]} != depth {depth.shape[:2]} — '
            'aligned_depth_to_color 토픽이 맞는지 확인한다')
        return 1
    # camera_info 가 depth 와 짝인지 본다. 엉뚱한 info(예: depth/camera_info)를 물리면
    # fx 가 ~640 vs ~900 으로 달라 grasp 가 통째로 어긋나는데 아무 에러도 안 난다.
    if node.info_wh != (depth.shape[1], depth.shape[0]):
        node.get_logger().error(
            f'camera_info {node.info_wh} != depth {(depth.shape[1], depth.shape[0])} — '
            f"info_topic({p['info_topic']}) 이 depth 와 짝이 아니다")
        return 1
    if p['use_tf'] and node.info_frame != p['camera_frame']:
        node.get_logger().warn(
            f"camera_info.frame_id='{node.info_frame}' 인데 camera_frame 파라미터는 "
            f"'{p['camera_frame']}' 이다 — TF 를 다른 프레임에서 뜨고 있을 수 있다")

    # yolo 는 최신 한 장이 아니라 최근 n_frames 장 중 탐지 픽셀이 가장 많은 프레임을 쓴다
    # (best_labels 참고) — depth 를 n_frames 만큼 모으는 동안 어차피 라벨도 같이 쌓인다.
    _, labels = (best_labels(node.yolo_labels_history[-n_frames:])
                 if p.get('seg_source') == 'yolo' else (None, None))
    seg, label_map, diag = segment(depth, K, T, p, labels)
    if seg is None:
        node.get_logger().error(diag)
        return 1

    out = os.path.join(p['out_dir'] or default_out_dir(), p['scene'])
    write_scene(out, depth, color, seg, K, T, label_map,
                [p['x_min'], p['y_min'], p['z_min'],
                 p['x_max'], p['y_max'], p['z_max']])

    node.get_logger().info(
        f"{diag}\n저장: {out}\n"
        f"  depth {depth.shape} {depth.dtype} 범위 {depth[depth > 0].min():.3f}~{depth.max():.3f} m\n"
        f"  camera_pose t={np.round(T[:3, 3], 4).tolist()}\n"
        f"다음: cd isaac_ros-dev/src/GraspGenX && uv run python scripts/demo_scene_pc.py "
        f"--sample_data_dir {os.path.dirname(out)} --gripper_name onrobot_RG2 "
        f"--scene {p['scene']} --moe_obb_density dense")
    return 0


def main():
    rclpy.init()
    node = SceneCapture()
    try:
        return run(node)
    finally:
        # 예외(잘못된 -p 값, imwrite 실패 등)로 빠져나가도 컨텍스트를 남기지 않는다
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
