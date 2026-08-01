"""
[객체 인식 학습] 공구 데이터셋으로 YOLOv8 커스텀 학습

실행: python3 yolo_train.py
사전: data_download.ipynb로 Roboflow 데이터셋을 받아 둘 것 (Mechanical-tools-10000-3/)
입력: <dataset>/data.yaml (클래스 이름·경로), custom_config.yaml (epochs 100, imgsz 640, batch 16)
출력: runs/detect/yolo_custom/weights/best.pt → yolo_eval.py가 이걸 읽는다

주의:
- 이 호스트에는 NVIDIA GPU가 없다(Intel 내장 그래픽). CPU 학습이라 100 epoch는 매우 오래 걸린다.
  수업용으로 돌려볼 때는 custom_config.yaml의 epochs를 먼저 줄인다.
- data 경로는 절대경로로 바꿔서 넘긴다. ultralytics가 상대경로를 자기 기준으로 해석해 엉뚱한 데를 본다.
- cfg와 인자를 같이 주면 인자가 우선한다. 하이퍼파라미터는 custom_config.yaml 한 곳에서 관리할 것.
"""

from ultralytics import YOLO


class YoloTrain:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

    def train(self, data_path, is_absolute_path=False):
        if not is_absolute_path:
            import os
            data_path = os.path.abspath(data_path)



        self.model.train(
            data=data_path,    # 데이터 경로
            name='yolo_custom',  # 실험 이름
            pretrained=True,     # 사전학습 모델 사용 여부
            cfg='custom_config.yaml' # 하이퍼파라미터 파일
        )


if __name__ == "__main__":
    yolo_train = YoloTrain("yolov8n.yaml")
    yolo_train.train("Mechanical-tools-10000-3/data.yaml")