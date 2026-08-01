"""
[라이브러리] RealSense ROS 토픽 구독 노드 — 직접 실행하지 않는다

용도: verify.py 등이 `from realsense import ImgNode`로 가져다 쓴다.
구독 토픽:
  /camera/camera/color/image_raw                    → get_color_frame()      (bgr8)
  /camera/camera/aligned_depth_to_color/image_raw   → get_depth_frame()      (passthrough, mm)
  /camera/camera/color/camera_info                  → get_camera_intrinsic() ({fx,fy,ppx,ppy})

주의:
- 자체 스레드를 돌리지 않는다. 호출자가 rclpy.spin_once(img_node)를 해줘야 프레임이 갱신된다.
- 첫 spin_once 직후에는 아직 None일 수 있다. None 검사 없이 쓰면 터진다.
- aligned_depth 토픽은 align_depth.enable:=true로 띄웠을 때만 나온다.
- 토픽 이름의 camera/camera 중복은 launch의 camera_name/camera_namespace 기본값 때문이다.
  네임스페이스를 바꿔 띄웠다면 여기도 같이 고쳐야 한다.
"""

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = None
        self.color_frame_stamp = None
        self.depth_frame = None
        self.intrinsics = None
        self.color_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

    def camera_info_callback(self, msg):
        self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_frame_stamp = str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def get_color_frame(self):
        return self.color_frame

    def get_color_frame_stamp(self):
        return self.color_frame_stamp

    def get_depth_frame(self):
        return self.depth_frame

    def get_camera_intrinsic(self):
        return self.intrinsics