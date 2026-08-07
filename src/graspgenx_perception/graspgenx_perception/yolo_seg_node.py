"""YOLO 인스턴스 세그멘테이션을 ROS 토픽으로 붙이는 노드.

원본 실험 스크립트(`yoloseg.py`)는 pyrealsense2 로 카메라를 직접 열고 `show=True` 로
띄웠다. 여기서는 두 가지를 바꿨다:

  1. **카메라를 직접 열지 않는다.** RealSense 장치는 한 프로세스만 잡을 수 있는데
     이 워크스페이스는 `realsense2_camera` 가 이미 물고 있다(graspx 파이프라인이
     `/camera/camera/aligned_depth_to_color/*` 를 쓴다). 직접 열면 둘 중 하나가 죽는다.
     그래서 컬러 토픽을 구독한다.
  2. **`show=True` 대신 overlay 토픽.** GUI 창은 컨테이너 안에서 X11 포워딩에 묶이고
     헤드리스에서 죽는다. 오버레이는 파라미터로 켜는 이미지 토픽으로 뺐다.

ultralytics 는 **호스트 시스템 파이썬에 없다** (torch 가 numpy 를 끌어올려 apt
cv_bridge 를 깬다 — `~/.claude/CLAUDE.md` §3). 도커 컨테이너 안에서만 돈다.
그래서 import 를 `_load_model()` 안으로 미룬다 — 이 모듈 자체는 호스트에서도
import 되고, 순수 함수 테스트가 GPU 없이 돈다.
"""

import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

# 라벨맵 규약은 형제 모듈 capture_graspgenx_scene.py 와 맞춘다.
# GraspGenX 로더가 `obj_` 접두어 라벨만 보므로 obj_1 -> 101, obj_2 -> 102 ...
LABEL_OBJ_BASE = 100
# 라벨맵이 uint8 이라 100+156 은 조용히 0 으로 랩어라운드한다.
# 형제 파일 capture_graspgenx_scene.py:84 와 **같은 값이어야 한다**.
MAX_OBJECTS = 155

# 컬러 스트림 QoS. depth=1 인 이유: 추론이 입력 fps 보다 느릴 때 depth 가 크면
# 큐에 쌓인 **묵은** 프레임을 순서대로 처리하게 되고, 그 마스크가 최신 depth 와 짝지어진다.
# 최신 프레임만 보는 게 맞다 (sensor_data 기본 depth 는 5 = 30fps 에서 최대 166ms 지연).
IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# 가중치가 없으면 여기서 찾는다. object_detection 패키지가 .pt 를 install 에 심는다
# (`.gitignore` 의 `*.pt` 때문에 커밋되지는 않는다 — README "가중치" 절 참고).
DEFAULT_WEIGHT_PKG = 'object_detection'
DEFAULT_WEIGHT_NAME = 'yolo11n-seg.pt'


def build_label_map(masks: np.ndarray, height: int, width: int,
                    max_objects: int = MAX_OBJECTS, min_pixels: int = 0) -> np.ndarray:
    """(N,H,W) 인스턴스 마스크 -> (H,W) uint8 라벨맵.

    - 입력은 **신뢰도 내림차순**(ultralytics 규약)이라고 본다. 역순으로 칠해서
      고신뢰 인스턴스가 최상위 z-order 를 갖게 한다.
    - 겹침으로 픽셀이 0개가 되거나 `min_pixels` 미만으로 깎인 인스턴스는 버리고,
      남은 것에 **101 부터 연속으로** 다시 매긴다. 라벨이 비면(101,103,…) 소비자의
      "obj_N = N번째 물체" 가정이 깨진다.
    """
    labels = np.zeros((height, width), dtype=np.uint8)
    if masks is None or len(masks) == 0:
        return labels

    # retina_masks=True 가 보장하는 것을 코드로 확인한다. 어긋나면 아래 불리언 인덱싱이
    # IndexError 로 터지는데, 여기서 먼저 잡아 원인이 드러나는 메시지를 남긴다.
    if tuple(masks.shape[1:]) != (height, width):
        raise ValueError(
            f'마스크 해상도 {tuple(masks.shape[1:])} != 이미지 {(height, width)}. '
            'predict(retina_masks=True) 가 빠졌는지 확인할 것')

    max_objects = max(0, min(int(max_objects), MAX_OBJECTS))
    sel = masks[:max_objects] > 0.5

    # uint8 로는 겹침 해소 중간값(최대 155)을 담을 수 있지만, 압축 전 임시 id 라
    # 의미가 다르므로 별도 배열을 쓴다.
    tmp = np.zeros((height, width), dtype=np.uint8)
    for i in range(len(sel) - 1, -1, -1):    # 뒤(저신뢰)부터 칠하고 앞(고신뢰)이 덮는다
        tmp[sel[i]] = i + 1

    next_id = 0
    for i in range(len(sel)):
        keep = tmp == i + 1
        if int(keep.sum()) < max(1, min_pixels):
            continue                          # 완전히 덮였거나 너무 작다 — 버린다
        next_id += 1
        labels[keep] = LABEL_OBJ_BASE + next_id
    return labels


class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('mask_topic', '/yolo_seg/mask')
        self.declare_parameter('label_topic', '/yolo_seg/labels')
        self.declare_parameter('overlay_topic', '/yolo_seg/overlay')
        self.declare_parameter('publish_overlay', False,
                               ParameterDescriptor(dynamic_typing=True))
        # 오버레이는 기본으로 JPEG 로 낸다. 848x480 bgr8 은 1.16MB 라 UDP 전용 경로
        # (fastdds_udp_only.xml)로는 15Hz 를 못 버틴다 — 실측 3.75Hz 까지 떨어졌다.
        # 같은 화면이 JPEG q80 이면 36KB 다(실제 씬 기준, 33배). 토픽 이름을
        # `<overlay_topic>/compressed` 로 두면 rqt_image_view 가 compressed 전송으로 인식한다.
        # dynamic_typing: launch 나 CLI 로 넘어온 값은 YAML 로 파싱된다.
        # `device:=0` 은 STRING 선언에 INTEGER 가 들어와 InvalidParameterTypeException 이고,
        # `conf:=1` 은 DOUBLE 선언에 INTEGER 다. 타입은 선언이 아니라 읽을 때 맞춘다
        # (capture_graspgenx_scene.py:107 과 같은 이유).
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('overlay_compressed', True, dyn)
        self.declare_parameter('overlay_jpeg_quality', 80, dyn)
        self.declare_parameter('conf', 0.25, dyn)
        self.declare_parameter('device', '0', dyn)   # ultralytics 가 'cpu'/'0,1' 도 받는다
        # 빈 리스트를 그냥 넘기면 rclpy 가 BYTE_ARRAY 로 추론해 정수 목록을 못 넣는다
        # (`-p classes:="[0,39]"` -> InvalidParameterTypeException).
        # capture_graspgenx_scene.py:111 이 같은 이유로 dynamic_typing 을 쓴다.
        self.declare_parameter('classes', [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('max_objects', MAX_OBJECTS, dyn)
        self.declare_parameter('min_pixels', 0, dyn)

        self.conf = float(self.get_parameter('conf').value)
        self.device = str(self.get_parameter('device').value)
        classes = self.get_parameter('classes').value
        self.classes = [int(c) for c in classes] if classes else None
        self.max_objects = max(0, min(int(self.get_parameter('max_objects').value), MAX_OBJECTS))
        self.min_pixels = int(self.get_parameter('min_pixels').value)
        self.publish_overlay = bool(self.get_parameter('publish_overlay').value)
        self.overlay_compressed = bool(self.get_parameter('overlay_compressed').value)
        self.jpeg_quality = int(self.get_parameter('overlay_jpeg_quality').value)
        self._warned_truncate = False

        self.bridge = CvBridge()
        self.model = self._load_model(self.get_parameter('model_path').value)

        self.mask_pub = self.create_publisher(Image, self.get_parameter('mask_topic').value, 10)
        self.label_pub = self.create_publisher(Image, self.get_parameter('label_topic').value, 10)
        overlay_topic = self.get_parameter('overlay_topic').value
        self.overlay_pub = None
        if self.publish_overlay:
            if self.overlay_compressed:
                self.overlay_pub = self.create_publisher(
                    CompressedImage, overlay_topic + '/compressed', 10)
                self.overlay_topic_name = overlay_topic + '/compressed'
            else:
                self.overlay_pub = self.create_publisher(Image, overlay_topic, 10)
                self.overlay_topic_name = overlay_topic
        self.image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, self.image_topic, self.image_callback, IMAGE_QOS)

        # "노드는 떴는데 아무것도 안 나온다"가 가장 흔한 실패다 (카메라 미기동,
        # 토픽명 불일치, 컨테이너 경계에서 데이터가 안 넘어옴). 조용히 기다리지 않는다.
        self._frames = 0
        self.create_timer(5.0, self._watchdog)

        self.get_logger().info(
            f'yolo_seg_node: {self.image_topic} -> mask/labels '
            f'(device={self.device}, conf={self.conf}, overlay={bool(self.overlay_pub)})')
        if self.overlay_pub is None:
            self.get_logger().info(
                '오버레이는 꺼져 있다 — /yolo_seg/overlay 토픽이 없다. '
                '보려면 -p publish_overlay:=true')
        else:
            self.get_logger().info(f'overlay -> {self.overlay_topic_name}')

    def _watchdog(self):
        if self._frames:
            self._frames = 0
            return
        self.get_logger().warn(
            f'5초간 {self.image_topic} 를 한 장도 못 받았다. 카메라가 떠 있는지, 토픽명이 맞는지, '
            '컨테이너면 FASTRTPS_DEFAULT_PROFILES_FILE 이 양쪽에 걸렸는지 확인할 것')

    def _load_model(self, model_path: str):
        from ultralytics import YOLO  # 호스트엔 없다 — 모듈 로드 시점이 아니라 여기서 터뜨린다

        if not model_path:
            share = get_package_share_directory(DEFAULT_WEIGHT_PKG)
            model_path = os.path.join(share, 'resource', DEFAULT_WEIGHT_NAME)
        # 존재 확인을 우리가 먼저 한다. ultralytics 는 basename 이 공식 에셋명이면
        # 없는 경로를 받아도 조용히 네트워크에서 받아온다 — 그러면 어떤 가중치가
        # 올라갔는지 로그만 보고는 알 수 없다.
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f'가중치가 없다: {model_path}\n'
                '`.gitignore` 의 `*.pt` 때문에 커밋되지 않는다 — README "가중치" 절 참고')
        model = YOLO(model_path)
        if model.task != 'segment':
            raise RuntimeError(
                f"'{model_path}' 는 task={model.task} 다. seg 가중치가 아니면 마스크가 안 나온다.")
        self.get_logger().info(f'model={model_path} classes={len(model.names)}')
        return model

    def image_callback(self, msg: Image) -> None:
        # 콜백에서 나간 예외는 rclpy 가 잡지 않아 spin() 밖으로 튀고 노드가 죽는다.
        # 한 프레임 실패(인코딩 불일치·CUDA OOM)로 노드를 잃지 않는다.
        self._frames += 1
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = bgr.shape[:2]

            # retina_masks=True 가 필수다. 없으면 masks.data 가 letterbox 된 모델 입력
            # 해상도로 나온다 (194x259 입력 -> 480x640 마스크, ultralytics 8.4.113 실측).
            # 그 상태로 라벨맵에 인덱싱하면 IndexError 다 — build_label_map 이 먼저 잡는다.
            res = self.model.predict(bgr, retina_masks=True, conf=self.conf, device=self.device,
                                     classes=self.classes, verbose=False)[0]

            masks = None if res.masks is None else res.masks.data.cpu().numpy()
            if masks is not None and len(masks) > self.max_objects and not self._warned_truncate:
                self.get_logger().warn(
                    f'인스턴스 {len(masks)}개 > max_objects={self.max_objects} — 나머지는 버린다')
                self._warned_truncate = True
            labels = build_label_map(masks, h, w, self.max_objects, self.min_pixels)
        except Exception as exc:                       # noqa: BLE001 - 프레임 하나 버리고 계속
            self.get_logger().error(f'프레임 처리 실패: {exc}')
            return

        label_msg = self.bridge.cv2_to_imgmsg(labels, encoding='mono8')
        label_msg.header = msg.header
        self.label_pub.publish(label_msg)

        # labels > 0 과 정보량이 같다. sam_mask_node 의 0/255 mono8 계약과 맞춰
        # 세그멘테이션 백엔드를 바꿔 끼울 수 있게 남긴다 (README "토픽" 참고).
        mask_msg = self.bridge.cv2_to_imgmsg(
            np.where(labels > 0, 255, 0).astype(np.uint8), encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        if self.overlay_pub is not None:
            plotted = res.plot()          # 박스 + 마스크 + 클래스명/점수를 함께 그린다
            if self.overlay_compressed:
                ok, buf = cv2.imencode('.jpg', plotted,
                                       [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if ok:
                    overlay_msg = CompressedImage()
                    overlay_msg.format = 'jpeg'
                    overlay_msg.data = buf.tobytes()
                else:
                    self.get_logger().warn('JPEG 인코딩 실패 — 오버레이를 건너뛴다')
                    return
            else:
                overlay_msg = self.bridge.cv2_to_imgmsg(plotted, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        # 생성도 try 안이다. 가중치 부재·detect 가중치·ultralytics 부재는 **정상 실패
        # 경로**인데, 밖에 두면 그때 rclpy.shutdown() 이 실행되지 않는다.
        node = YoloSegNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                   # Ctrl-C / SIGTERM 은 정상 종료다
    finally:
        if node is not None:
            node.destroy_node()
        # SIGTERM(`timeout`, docker stop)으로 죽으면 시그널 핸들러가 이미 컨텍스트를
        # 내린 뒤라 다시 부르면 RCLError 로 터진다 — 정상 종료가 스택트레이스를 남긴다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
