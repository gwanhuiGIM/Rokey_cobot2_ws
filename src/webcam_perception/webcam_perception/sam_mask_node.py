import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def auto_box_from_color(bgr_image: np.ndarray, corner_frac: float = 0.05,
                         diff_thresh: float = 30.0, min_area_frac: float = 0.01):
    """네 모서리 색을 배경으로 가정하고, 배경과 색이 다른 최대 덩어리의 bounding box를 반환.
    물체가 없거나 너무 작으면 None."""
    h, w = bgr_image.shape[:2]
    cf = max(1, int(min(h, w) * corner_frac))
    corners = np.concatenate([
        bgr_image[:cf, :cf].reshape(-1, 3),
        bgr_image[:cf, -cf:].reshape(-1, 3),
        bgr_image[-cf:, :cf].reshape(-1, 3),
        bgr_image[-cf:, -cf:].reshape(-1, 3),
    ]).astype(np.float32)
    bg_color = np.median(corners, axis=0)

    diff = np.linalg.norm(bgr_image.astype(np.float32) - bg_color, axis=2)
    fg = (diff > diff_thresh).astype(np.uint8) * 255
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))

    contours, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area_frac * h * w:
        return None

    x, y, bw, bh = cv2.boundingRect(largest)
    # GrabCut은 rect 바깥을 확정 배경으로 쓴다. rect가 이미지 가장자리에 닿으면
    # 확정 배경 표본이 없어 initGMMs에서 죽는다 — 최소 1px 여유를 강제한다.
    margin = 1
    x = max(x, margin)
    y = max(y, margin)
    bw = min(bw, w - x - margin)
    bh = min(bh, h - y - margin)
    if bw <= 0 or bh <= 0:
        return None
    return (x, y, bw, bh)


def compute_mask(bgr_image: np.ndarray, box: tuple) -> np.ndarray:
    """GrabCut 기반 전경 마스크. box=(x, y, w, h). 반환: 0/255 mono8."""
    mask = np.zeros(bgr_image.shape[:2], np.uint8)
    bgd_model = np.zeros((1, 65), np.float64)
    fgd_model = np.zeros((1, 65), np.float64)
    cv2.grabCut(bgr_image, mask, box, bgd_model, fgd_model, 5, cv2.GC_INIT_WITH_RECT)
    return np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)


class SamMaskNode(Node):
    # ponytail: GrabCut + 색상 기반 자동 박스는 SAM/MobileSAM 자리에 넣은 CPU 전용 임시 백엔드다.
    # segment_anything/mobile_sam은 pip 전용 배포라 apt로 못 깐다 — 실제 SAM은
    # 별도 격리 venv/conda + 토픽 브릿지로 붙일 때 compute_mask()만 교체하면 된다.
    # 배경이 단색/균일하다는 가정이 깨지면(잡동사니 배경) auto_box_from_color가 실패하고
    # fallback_box로 되돌아간다.

    def __init__(self):
        super().__init__('sam_mask_node')
        self.declare_parameter('image_topic', '/webcam/image_raw')
        self.declare_parameter('mask_topic', '/webcam/segmentation_mask')
        self.declare_parameter('diff_thresh', 30.0)
        self.declare_parameter('min_area_frac', 0.01)
        self.declare_parameter('fallback_box_x', 160)
        self.declare_parameter('fallback_box_y', 120)
        self.declare_parameter('fallback_box_w', 320)
        self.declare_parameter('fallback_box_h', 240)

        image_topic = self.get_parameter('image_topic').value
        mask_topic = self.get_parameter('mask_topic').value
        self.diff_thresh = self.get_parameter('diff_thresh').value
        self.min_area_frac = self.get_parameter('min_area_frac').value
        self.fallback_box = (
            self.get_parameter('fallback_box_x').value,
            self.get_parameter('fallback_box_y').value,
            self.get_parameter('fallback_box_w').value,
            self.get_parameter('fallback_box_h').value,
        )

        self.bridge = CvBridge()
        self.mask_pub = self.create_publisher(Image, mask_topic, 10)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(f'sam_mask_node: {image_topic} -> {mask_topic} (색상 기반 자동 박스)')

    def image_callback(self, msg: Image) -> None:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        box = auto_box_from_color(bgr, diff_thresh=self.diff_thresh, min_area_frac=self.min_area_frac)
        if box is None:
            box = self.fallback_box
        mask = compute_mask(bgr, box)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SamMaskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
