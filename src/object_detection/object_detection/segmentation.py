import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Header

from od_msg.srv import SrvSegmentation
from od_msg.msg import SegmentationResult
from object_detection.realsense import ImgNode
from object_detection.yolo_seg import YoloSegModel


class SegmentationNode(Node):
    """
    graspgenx가 호출하는 'get_segmentation' 서비스를 제공하는 노드.

    - RealSense는 별도 터미널(별도 프로세스)에서 realsense2_camera로 구동된다고 가정합니다.
      (/camera/camera/color/image_raw, /camera/camera/aligned_depth_to_color/image_raw,
       /camera/camera/color/camera_info 토픽을 이 노드가 구독)
    - 요청이 오면 최신 컬러 프레임 1장에 대해 YOLO-seg를 돌려
      class_name/box/score/mask를 담은 SegmentationResult[] 를 반환합니다.
    - 기존 detection.py(get_3d_position, bbox 전용) 노드와는 별개 노드이며,
      동시에 실행해도 서비스 이름이 겹치지 않습니다.
    """

    def __init__(self):
        super().__init__('segmentation_node')
        self.img_node = ImgNode()
        self.bridge = CvBridge()
        self.model = YoloSegModel()
        self.create_service(
            SrvSegmentation,
            'get_segmentation',
            self.handle_get_segmentation
        )
        self.get_logger().info(
            "SegmentationNode initialized. Waiting for graspgenx's 'get_segmentation' calls..."
        )

    def handle_get_segmentation(self, request, response):
        target = request.target if request.target else None
        self.get_logger().info(f"Received segmentation request (target='{target or 'ALL'}')")

        rclpy.spin_once(self.img_node)
        frame, detections = self.model.get_all_detections(self.img_node, target)

        if frame is None:
            self.get_logger().warn("No color frame available yet.")
            response.success = False
            return response

        response.header = self._make_header()
        response.color_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        response.color_image.header = response.header

        segments = []
        for det in detections:
            seg = SegmentationResult()
            seg.class_name = det["class_name"]
            seg.class_id = det["label"]
            seg.score = det["score"]
            seg.box = [float(v) for v in det["box"]]
            seg.mask = self.bridge.cv2_to_imgmsg(det["mask"], encoding='mono8')
            seg.mask.header = response.header
            segments.append(seg)

        response.segments = segments
        response.success = len(segments) > 0

        if not response.success:
            self.get_logger().warn("No matching detections found.")
        else:
            self.get_logger().info(f"Returning {len(segments)} segment(s) to caller.")

        return response

    def _make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_color_optical_frame'
        return header


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
