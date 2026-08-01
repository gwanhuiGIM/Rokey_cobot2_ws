"""
[객체 인식 추론] 학습된 가중치로 이미지 검출

실행: python3 yolo_eval.py
입력: runs/detect/yolo_custom/weights/best.pt (yolo_train.py 산출물), sample_test.jpg
출력: result_0.jpg (img_save=True일 때만 저장)

사용: YoloEval(weights).eval([이미지경로들], conf=0.25, img_save=True)
conf를 올리면 오탐이 줄고 미검출이 는다. 실기 픽앤플레이스에서는 오탐 하나가 헛집기이므로 보통 올려 잡는다.

주의: eval()이 결과를 저장만 하고 반환하지 않는다. 좌표가 필요하면 result.boxes를 리턴하도록 고쳐야 한다.
"""

from ultralytics import YOLO


class YoloEval:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

    def eval(self, image_path, conf = 0.25, img_save = False):
        results = self.model.predict(source=image_path, conf=conf)
        for i, result in enumerate(results):
            if img_save:
                result.save(filename=f'result_{i}.jpg')  # 결과 이미지 저장


if __name__ == "__main__":
    yolo_eval = YoloEval("runs/detect/yolo_custom/weights/best.pt")
    yolo_eval.eval(["sample_test.jpg"], img_save=True)