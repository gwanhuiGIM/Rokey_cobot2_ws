"""
[핸드아이 캘리브레이션 1단계] 체커보드 이미지 + 로봇 자세 동시 수집

실행: python3 data_recording.py   (dsr_bringup2 실행 중이어야 함, 실기/가상 무관하게 posx 필요)
입력: DEVICE_NUMBER(=6) V4L2 카메라. RealSense ROS 토픽이 아니라 /dev/videoN을 직접 연다.
출력: ./data/<x>_<y>_<z>.jpg 와 ./data/calibrate_data.json (poses[] + file_name[])
조작: 카메라 창에서 'q' → 현재 프레임 저장 + 현재 posx 기록. 자세를 바꿔가며 15~20회 반복.

주의:
- 'q'는 저장이지 종료가 아니다. 루프 탈출 코드가 없으므로 Ctrl+C로 끝낸다.
- set_tool/set_tcp 이름("Tool Weight_2FG", "2FG_TCP")이 티치펜던트에 등록돼 있어야 한다.
  RG2를 쓰면 이 두 줄을 실제 등록명으로 바꿔야 원점이 맞는다.
- 자세는 회전을 충분히 섞어야 한다. 평행이동만 하면 캘리브레이션이 수렴하지 않는다.
"""

import os
import cv2
import json
import rclpy
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
DEVICE_NUMBER = 6

DR_init.dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    # 로봇 제어 모듈 가져오기
    try:
        from DSR_ROBOT2 import (
            get_current_posx,
            set_tool,
            set_tcp,
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    # 공구 및 TCP 설정
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # 데이터 저장 경로 설정
    source_path = "./data"
    os.makedirs(source_path, exist_ok=True)
    # 카메라 연결
    print(f"현재 선택된 device number는 {DEVICE_NUMBER}입니다.")
    cap = cv2.VideoCapture(DEVICE_NUMBER)  # 4 is camera number, set your camera number

    write_data = {}
    write_data["poses"] = []
    write_data["file_name"] = []

    while True:
        ret, frame = cap.read()

        if not ret:
            print("카메라를 찾을 수 없습니다. DEVICE_NUMBER를 변경해주세요.")
            exit(True)
        cv2.imshow("camera", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            pos = get_current_posx()[0]
            file_name = f"{pos[0]}_{pos[1]}_{pos[2]}.jpg"
            # 현재 위치 기반 이미지 저장
            cv2.imwrite(f"{source_path}/{file_name}", frame)
            print("current position1 : ", pos)
            write_data["file_name"].append(file_name)
            write_data["poses"].append(pos)
            print(f"save img to {source_path}/{file_name}")
            with open(f"{source_path}/calibrate_data.json", "w") as json_file:
                json.dump(write_data, json_file, indent=4)

    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
