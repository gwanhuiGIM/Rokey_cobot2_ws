########## YoloSegModel ##########
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


PACKAGE_NAME = "object_detection"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

# 커스텀 학습 seg 가중치로 교체할 때는 이 파일명만 바꾸고
# resource/ 아래에 동일한 이름으로 넣으면 됩니다.
# resource/에 파일이 없으면 ultralytics가 이 이름으로 자동 다운로드를 시도합니다
# (사전학습 COCO 80종 yolov8n-seg.pt).
YOLO_SEG_MODEL_FILENAME = "yolov8n-seg.pt"
YOLO_SEG_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_SEG_MODEL_FILENAME)


class YoloSegModel:
    """YOLO 세그멘테이션 모델 래퍼.

    - 클래스 이름은 (기존 yolo.py처럼 별도 json을 쓰지 않고) 모델에 내장된
      names를 그대로 사용합니다. 커스텀 seg 가중치로 교체해도 그대로 동작합니다.
    - retina_masks=True 옵션으로 마스크를 원본 프레임 해상도로 바로 받습니다.
    """

    def __init__(self, confidence_threshold: float = 0.5):
        model_path = (
            YOLO_SEG_MODEL_PATH
            if os.path.exists(YOLO_SEG_MODEL_PATH)
            else YOLO_SEG_MODEL_FILENAME
        )
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        self.class_names = self.model.names  # {id: name}

    def get_frame(self, img_node, timeout: float = 2.0):
        """최신 컬러 프레임 1장을 획득합니다 (timeout 초과 시 None)."""
        end_time = time.time() + timeout
        frame = img_node.get_color_frame()
        while frame is None and time.time() < end_time:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
        return frame

    def get_all_detections(self, img_node, target: str = None):
        """
        컬러 프레임 1장에 대해 세그멘테이션을 수행합니다.

        Returns:
            frame: 사용된 원본 컬러 프레임 (np.ndarray, BGR) 또는 None
            detections: list of dict
                {
                    "label": int,
                    "class_name": str,
                    "score": float,
                    "box": [x1, y1, x2, y2],
                    "mask": np.ndarray (H, W) uint8, 0 또는 255
                }
        target이 주어지면 class_name이 일치하는 detection만 반환합니다.
        """
        frame = self.get_frame(img_node)
        if frame is None:
            return None, []

        results = self.model(
            frame, verbose=False, retina_masks=True, conf=self.confidence_threshold
        )
        result = results[0]

        detections = []
        if result.masks is None or len(result.boxes) == 0:
            return frame, detections

        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        labels = result.boxes.cls.cpu().numpy().astype(int)
        masks = result.masks.data.cpu().numpy()  # (N, H, W), retina_masks=True -> 원본 해상도

        h, w = frame.shape[:2]

        for box, score, label, mask in zip(boxes, scores, labels, masks):
            class_name = self.class_names.get(int(label), str(int(label)))
            if target and class_name != target:
                continue

            mask_u8 = (mask > 0.5).astype(np.uint8) * 255
            if mask_u8.shape != (h, w):
                mask_u8 = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_NEAREST)

            detections.append(
                {
                    "label": int(label),
                    "class_name": class_name,
                    "score": float(score),
                    "box": box.tolist(),
                    "mask": mask_u8,
                }
            )

        return frame, detections
