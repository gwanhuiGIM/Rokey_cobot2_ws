"""
[핸드아이 캘리브레이션 1단계] 체커보드 이미지 + 로봇 자세 동시 수집

실행: python3 data_recording.py   (dsr_bringup2 실행 중이어야 함, 실기/가상 무관하게 posx 필요)
입력: DEVICE_NUMBER(=6) V4L2 카메라. RealSense ROS 토픽이 아니라 /dev/videoN을 직접 연다.
출력: ./data/<x>_<y>_<z>.jpg 와 ./data/calibrate_data.json (poses[] + file_name[])
조작: 카메라 창에서 'q' → 현재 프레임 저장 + 현재 posx 기록. 자세를 바꿔가며 15~20회 반복.

주의:
- 'q'는 저장이지 종료가 아니다. 루프 탈출 코드가 없으므로 Ctrl+C로 끝낸다.
- TOOL_NAME/TCP_NAME이 티치펜던트에 등록돼 있어야 한다. 없으면 set_* 가 -1을 돌려주고
  조용히 무시되므로, 아래에서 반환값 검사 + get_tcp() 되읽기로 끊는다.
- 자세는 회전을 충분히 섞어야 한다. 평행이동만 하면 캘리브레이션이 수렴하지 않는다.
- ⚠️ **카메라를 1280x720으로 띄운 뒤 수집한다.** 이 스크립트는 해상도를 지정하지 않고
  구독만 하므로, 런치 기본값(`camera.launch.py` = 424x240)으로 띄운 채 수집하면
  코너 검출 정밀도가 그대로 떨어져 내부파라미터가 망가진다:
    ros2 launch m0609_rg2_bringup camera.launch.py color_profile:=1280x720x15
  (`data/` 34장 전수가 1280x720임을 확인 — 2026-08-03)
- 수집 후 `python3 eye2hand_calibration.py`가 RMS 재투영오차·AX=XB 잔차·LOO 안정성을
  찍는다. **그 숫자를 보고 재수집 여부를 정한다.** 결과 행렬만 보고 넘어가지 말 것.
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
DEVICE_NUMBER = 2

# 어느 카메라로 수집할지. eye-to-hand(D435i)는 True, eye-in-hand(C270)는 False.
#   True  → ROS 토픽 /camera/camera/color/image_raw (realsense2_camera 실행 중이어야 함)
#   False → V4L2 /dev/video<DEVICE_NUMBER>
USE_REALSENSE_TOPIC = True

# True면 set_tcp을 걸지 않는다 → posx가 flange 기준이 되어 결과를 CAD/줄자로 검산할 수 있다.
# False면 아래 TOOL_NAME/TCP_NAME이 티치펜던트 등록명과 **정확히** 일치해야 한다.
#
# 참고: eye-to-hand(판을 그리퍼에 물림)에서는 판↔그리퍼 변환 G가 AX=XB 유도에서 소거되므로
#       이 플래그가 True든 False든 T_cam2base 결과는 수학적으로 같다
#       (`python3 eye2hand_calibration.py --selfcheck`로 확인 가능. TCP 220mm도 마찬가지).
#       False로 두는 이유는 정확도가 아니라 **이후 pick 코드와 프레임을 통일**하기 위해서다.
#       진짜 중요한 건 수집 도중에 바꾸지 않는 것이다 — 바꾸면 G가 상수가 아니게 되어 전부 깨진다.
RECORD_IN_FLANGE_FRAME = False
TOOL_NAME = "Tool Weight"   # RG2만 등록된 값. 캘리브 판(폼보드+종이 1장)은 무게 무시 가능해
                            # 재등록하지 않았다 (2026-08-03 사용자 확인). 무거운 판으로 바꾸면 재등록할 것.
TCP_NAME = "GripperDA_v1"

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
            get_tool,
            get_tcp,
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    # 공구 및 TCP 설정
    if RECORD_IN_FLANGE_FRAME:
        print("[frame] set_tcp 미적용 — posx는 flange 기준. 결과 부모 프레임도 flange다.")
    else:
        # set_tool/set_tcp는 등록명이 없으면 -1을 돌려주고 **아무것도 바꾸지 않는다**.
        # 반환값을 안 보면 flange 기준으로 조용히 수집해버리므로 여기서 끊는다.
        # (수집을 다 끝낸 뒤에야 알아채면 실기 시간이 통째로 날아간다.)
        if set_tool(TOOL_NAME) != 0:
            raise SystemExit(f"set_tool('{TOOL_NAME}') 실패 — 티치펜던트 등록명을 확인할 것")
        if set_tcp(TCP_NAME) != 0:
            raise SystemExit(f"set_tcp('{TCP_NAME}') 실패 — 티치펜던트 등록명을 확인할 것")
        # 되읽어 실제 적용값을 확인한다 (반환값만 믿지 않는다)
        print(f"[frame] tool='{get_tool()}', tcp='{get_tcp()}' 적용 — posx는 TCP 기준.")

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
