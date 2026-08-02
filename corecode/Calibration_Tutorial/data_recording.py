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
from pathlib import Path

import cv2
import json
import rclpy
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
DEVICE_NUMBER = 6

# 어느 카메라로 수집할지. eye-to-hand(D435i)는 True, eye-in-hand(C270)는 False.
#   True  → ROS 토픽 /camera/camera/color/image_raw (realsense2_camera 실행 중이어야 함)
#   False → V4L2 /dev/video<DEVICE_NUMBER>
USE_REALSENSE_TOPIC = True

# True면 set_tcp을 걸지 않는다 → posx가 flange 기준이 되어 결과를 CAD/줄자로 검산할 수 있다.
# False면 아래 TOOL_NAME/TCP_NAME이 티치펜던트 등록명과 일치해야 한다.
RECORD_IN_FLANGE_FRAME = True
TOOL_NAME = "Tool Weight_2FG"   # RG2 실제 등록명으로 교체
TCP_NAME = "2FG_TCP"

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
    if RECORD_IN_FLANGE_FRAME:
        print("[frame] set_tcp 미적용 — posx는 flange 기준. 결과 부모 프레임도 flange다.")
    else:
        set_tool(TOOL_NAME)
        set_tcp(TCP_NAME)
        print(f"[frame] set_tcp('{TCP_NAME}') 적용 — posx는 TCP 기준. 결과 부모 프레임도 TCP다.")

    # 데이터 저장 경로 설정
    source_path = str(Path(__file__).resolve().parent / "data")
    os.makedirs(source_path, exist_ok=True)

    # 카메라 연결
    cap = img_node = None
    if USE_REALSENSE_TOPIC:
        from realsense import ImgNode

        img_node = ImgNode()
        print("RealSense 토픽(/camera/camera/color/image_raw)에서 수집합니다.")
    else:
        print(f"현재 선택된 device number는 {DEVICE_NUMBER}입니다.")
        cap = cv2.VideoCapture(DEVICE_NUMBER)

    def read_frame():
        """수집 소스와 무관하게 BGR 프레임 한 장을 돌려준다. 없으면 None."""
        if img_node is not None:
            rclpy.spin_once(img_node, timeout_sec=0.1)
            return img_node.get_color_frame()
        ok, f = cap.read()
        return f if ok else None

    write_data = {}
    write_data["poses"] = []
    write_data["file_name"] = []

    print("'q' = 현재 자세 저장, ESC = 종료. 자세마다 회전을 30도 이상 섞을 것.")
    while True:
        frame = read_frame()

        if frame is None:
            print("프레임을 받지 못했습니다. 카메라 설정을 확인하세요.")
            continue
        cv2.imshow("camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC — json이 잘리지 않게 정상 종료
            print(f"종료. 총 {len(write_data['poses'])}개 수집.")
            break
        if key == ord("q"):
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
