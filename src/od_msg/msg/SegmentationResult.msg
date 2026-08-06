# 세그멘테이션된 객체 1개에 대한 결과
string class_name          # 예: "person", "cup" (모델에 내장된 클래스명)
int32 class_id             # 클래스 인덱스
float32 score               # confidence score (0.0 ~ 1.0)
float64[4] box               # bounding box [x1, y1, x2, y2] (픽셀 좌표)
sensor_msgs/Image mask     # binary mask (mono8, 0 또는 255), color 이미지와 동일 해상도
