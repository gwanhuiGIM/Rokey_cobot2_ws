#!/usr/bin/env python3
"""camera_info_cb과 정확히 같은 대입을 해서 msg.k 타입이 np.ndarray인지 확인."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class InfoProbe(Node):
    def __init__(self):
        super().__init__('info_probe')
        self.n = 0
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
                                  self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.n += 1
        if self.n <= 2:
            print(f'type(msg.k)={type(msg.k)}  isinstance ndarray={isinstance(msg.k, __import__("numpy").ndarray)}', flush=True)


rclpy.init()
node = InfoProbe()
import time
t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.2)
print(f'received {node.n}')
