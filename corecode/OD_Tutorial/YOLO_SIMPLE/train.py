"""
[객체 인식 학습 - 최소 예제] 클래스 없이 5줄로 끝내는 버전

실행: python3 train.py
입력: datasets_seg/data.yaml   ← 이 디렉토리에 없다. 직접 준비해야 한다.
출력: runs/detect/yolov8_tool_seg_0123/weights/

YOLO/yolo_train.py와의 차이: 설정 파일 없이 인자로 다 넘긴다(epochs 300, save_period 10).
구조를 보려면 이 파일, 실제로 굴리려면 YOLO/ 쪽을 쓴다.

주의:
- yolov8n-det.pt는 공식 배포명이 아니다(정식은 yolov8n.pt). 파일이 없으면 다운로드에 실패한다.
- GPU 없는 이 호스트에서 300 epoch는 현실적이지 않다.
"""

from ultralytics import YOLO
import os

model = YOLO("yolov8n-det.pt")
model.train(data="datasets_seg/data.yaml", epochs=300, save_period=10, name="yolov8_tool_seg_0123")
