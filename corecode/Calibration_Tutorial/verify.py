"""
[핸드아이 캘리브레이션 3단계] 결과 검증 — 클릭한 물체를 실제로 집는다

⚠️ 실기가 움직인다. movel + 그리퍼 개폐가 실행되므로 주변을 비우고 E-stop을 잡은 채로 실행할 것.

실행: python3 verify.py
사전 조건:
  1) ros2 launch dsr_bringup2 ... (m0609, 네임스페이스 dsr01)
  2) ros2 launch realsense2_camera rs_align_depth_launch.py align_depth.enable:=true
  3) 같은 디렉토리에 T_gripper2camera.npy 존재
  4) 그리퍼 Modbus 도달 가능 (192.168.1.1:502)

조작: "Webcam" 창에서 물체를 좌클릭 → 픽셀 depth → 카메라 좌표 → 베이스 좌표 → 집어서 JReady로 이동 후 놓음. ESC로 종료.
좌표 변환: base2cam = base2gripper(현재 posx) @ gripper2cam(npy)

주의:
- 픽셀 하나의 depth를 그대로 쓴다. 물체 가장자리나 반사면을 클릭하면 z가 0이나 튄 값이 되어 엉뚱한 곳으로 간다.
- posj가 __init__에서 쓰이는데 import는 __main__ 블록에 있다. 이 파일을 모듈로 import하면 NameError가 난다.
- 오차가 크면 코드보다 먼저 의심할 것: TCP 설정, 체커보드 실측 칸 크기, depth 정렬 여부.
"""

import cv2
import rclpy
from rclpy.node import Node
from realsense import ImgNode
from scipy.spatial.transform import Rotation
from onrobot import RG

import time
import numpy as np


import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"


# 마우스 콜백 함수
class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.img_node = ImgNode()
        rclpy.spin_once(self.img_node)
        time.sleep(1)
        self.intrinsics = self.img_node.get_camera_intrinsic()
        self.gripper2cam = np.load("T_gripper2camera.npy")
        self.JReady = posj([0, 0, 90, 0, 90, 0])
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            depth_frame = self.img_node.get_depth_frame()
            while depth_frame is None or np.all(depth_frame == 0):
                self.get_logger().info("retry get depth img")
                rclpy.spin_once(self.img_node)
                depth_frame = self.img_node.get_depth_frame()

            print(f"img cordinate: ({x}, {y})")
            z = self.get_depth_value(x, y, depth_frame)
            camera_center_pos = self.get_camera_pos(x, y, z, self.intrinsics)
            print(f"camera cordinate: ({camera_center_pos})")

            robot_coordinate = self.transform_to_base(camera_center_pos)
            print(f"robot cordinate: ({robot_coordinate})")

            self.pick_and_drop(*robot_coordinate)
            print("=" * 100)

    def get_camera_pos(self, center_x, center_y, center_z, intrinsics):
        camera_x = (center_x - intrinsics["ppx"]) * center_z / intrinsics["fx"]
        camera_y = (center_y - intrinsics["ppy"]) * center_z / intrinsics["fy"]
        camera_z = center_z

        return (camera_x, camera_y, camera_z)

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def pick_and_drop(self, x, y, z):
        current_pos = get_current_posx()[0]
        pick_pos = posx([x, y, z, current_pos[3], current_pos[4], current_pos[5]])
        # TODO: Write pick andpos drop function
        movel(pick_pos, vel=VELOCITY, acc=ACC)

        pick_pos_down = posx(0, 0, -20, 0, 0, 0)
        movel(pick_pos_down, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

        self.gripper.close_gripper()
        wait(1)

        pick_pos_up = posx(0, 0, 20, 0, 0, 0)
        movel(pick_pos_up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

        movej(self.JReady, vel=VELOCITY, acc=ACC)
        self.gripper.open_gripper()
        wait(1)

    def transform_to_base(self, camera_coords):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        # gripper2cam = np.load(self.gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        base2gripper = self.get_robot_pose_matrix(*get_current_posx()[0])
        timer = time.time()

        base2cam = base2gripper @ self.gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def open_img_node(self):
        rclpy.spin_once(self.img_node)
        img = self.img_node.get_color_frame()

        cv2.setMouseCallback("Webcam", self.mouse_callback, img)
        cv2.imshow("Webcam", img)

        # if cv2.waitKey(1) & 0xFF == 27:  # ESC 키로 종료
        #     break

    def get_depth_value(self, center_x, center_y, depth_frame):
        height, width = depth_frame.shape
        if 0 <= center_x < width and 0 <= center_y < height:
            depth_value = depth_frame[center_y, center_x]
            return depth_value
        self.get_logger().warn(f"out of image range: {center_x}, {center_y}")
        return None


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            get_current_posx,
            movej,
            movel,
            wait,
            DR_MV_MOD_REL
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        exit(True)
    # rclpy.init()

    cv2.namedWindow("Webcam")

    test_node = TestNode()

    while True:
        test_node.open_img_node()

        if cv2.waitKey(1) & 0xFF == 27:  # ESC 키로 종료
            break

    cv2.destroyAllWindows()
