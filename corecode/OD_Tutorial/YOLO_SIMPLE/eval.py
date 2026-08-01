"""
[객체 인식 추론 - 최소 예제] 사전학습 COCO 모델로 바로 검출

실행: python3 eval.py   → sample.jpg / sample2.jpg 결과 창이 뜬다
학습 없이 동작한다. yolov8n.pt(COCO 80클래스)를 자동으로 내려받아 쓴다.

용도: ultralytics 설치가 제대로 됐는지 확인하는 첫 관문.
      여기서 창이 뜨면 환경은 정상이고, 그 다음 커스텀 학습으로 넘어간다.

주의: COCO 클래스라 hammer/screwdriver/wrench는 못 잡는다. 공구 검출은 커스텀 학습이 필요하다.
"""

from ultralytics import YOLO

# 사전학습된 YOLO 모델 로드 (예: YOLOv8n)
model = YOLO("yolov8n.pt")

# 이미지에 대해 inference 실행
results = model(["sample2.jpg", "sample.jpg"], imgsz=640)

# 결과 시각화

for result in results:
    result.show()

# 추가적으로 결과 정보 출력 (옵션)
print(results)