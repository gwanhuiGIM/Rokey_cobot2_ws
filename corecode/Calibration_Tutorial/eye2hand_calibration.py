"""
[핸드아이 캘리브레이션 2단계 - eye-to-hand] 카메라가 삼각대 등에 고정된 경우

실행: python3 eye2hand_calibration.py   (ROS 불필요, 오프라인 계산)
입력: data/calibrate_data.json + data/*.jpg  (data_recording.py 산출물, 체커보드는 그리퍼에 부착)
출력: T_cam2base.npy — 카메라→베이스 4x4 변환
방법: AX=XB를 Park-Martin 방식으로 직접 구현(Calibrate 함수). cv2.calibrateHandEye를 쓰지 않는다.

handeye_calibration.py와의 차이: 카메라 장착 위치가 반대다. 둘 중 하나만 쓴다.
카메라가 그리퍼에 달렸으면 handeye_calibration.py, 고정돼 있으면 이 파일.

--------------------------------------------------------------------------------
왜 X가 T_base<-cam 인가 (출력 방향을 헷갈리면 포인트클라우드가 로봇 옆에 떨어진다)

기호: T_a<-b = "b 좌표를 a 좌표로 옮기는 4x4". 상수 2개:
    G = T_gripper<-board   판을 그리퍼로 물고 있으므로 수집 내내 고정
    C = T_base<-cam        카메라를 삼각대에 고정했으므로 고정  ← 우리가 구하려는 값

    
i번째 자세에서 판의 베이스 좌표는 두 경로로 같다:
    T_base<-gripper(i) · G  =  C · T_cam<-board(i)                        ...(*)

코드가 만드는 두 리스트:
    T_gripper2base_list[i] = inv(로봇 posx) = T_gripper<-base(i)
    T_checker2cam_list[i]  = inv(solvePnP)  = T_board<-cam(i)
연속 자세쌍:
    A_i = inv(T_gripper<-base(i)) · T_gripper<-base(i+1)
    B_i = inv(T_board<-cam(i))    · T_board<-cam(i+1)
(*)를 대입하면 G가 통째로 소거되고  A_i = C · B_i · inv(C),  즉 A_i·C = C·B_i.
Calibrate()가 푸는 AX=XB의 해 X가 곧 C = T_base<-cam 이다.

따라서:
- **set_tcp / set_tool은 결과에 영향이 없다.** G가 소거되기 때문. flange 기준으로 수집해도 된다.
  단 수집 도중에 TCP를 바꾸면 G가 상수가 아니게 되어 전부 깨진다.
- 판이 그리퍼 축과 삐뚤게 물려도 상관없다(그것도 G에 흡수). 대신 **수집 중 재파지 금지.**
- 출력 npy는 그대로 static TF의 base_link->camera_link로 들어간다
  (scripts/calib_npy_to_tf.py, 단위 mm->m + OpenCV optical->REP-103 변환은 거기서 한다).
검산: `python3 eye2hand_calibration.py --selfcheck` — 합성 데이터로 C를 복원해본다.
수집 진단: `python3 eye2hand_calibration.py --pose-quality` — 풀기 전에 자세 분포만 채점한다
           (보드 기울기 / 상대회전 축 다양성 / 거리 다양성). 읽기 전용, npy를 건드리지 않는다.
--------------------------------------------------------------------------------

주의:
- (2026-08-02 수정됨) find_checkerboard_pose의 25mm 하드코딩은 square_size를 쓰도록 고쳤다.
- logR()은 theta가 0에 가까우면(회전이 거의 없는 자세쌍) 0으로 나눠 NaN이 된다.
  수집할 때 자세마다 회전을 충분히 줘야 한다.
- 특이행렬 자세는 det 검사로 자동 제외되며 콘솔에 경고가 찍힌다.
"""

import os
from pathlib import Path

# 경로는 cwd가 아니라 이 파일 위치를 기준으로 잡는다.
# (VS Code는 워크스페이스 루트에서 실행하고 터미널은 이 디렉토리에서 실행해 서로 어긋났다.)
DATA_DIR = Path(__file__).resolve().parent / "data"
import json
from datetime import datetime
from scipy.spatial.transform import Rotation
import numpy as np
import cv2

# ---- RealSense D435i 공장 내부파라미터 --------------------------------------
# 왜 이걸 쓰나: 이미지셋으로 추정하면(cv2.calibrateCamera) 카메라가 고정이라 보드 거리
# 다양성이 부족해 추정이 나쁘다. 2026-08-03 실측 RMS 1.725 px(data/) / 2.597 px(data1/)로
# 둘 다 "재수집" 수준이었다. 공장값은 그 조건에 영향받지 않는다.
#
# 값의 출처: 이 랩탑에 물린 D435i의 /camera/camera/color/camera_info 를 프로파일별로 직접
# 띄워 읽은 것(2026-08-03). **장비 고유값이라 카메라를 교체하면 다시 읽어야 한다:**
#   ros2 launch m0609_rg2_bringup camera.launch.py color_profile:=1280x720x15
#   ros2 topic echo /camera/camera/color/camera_info --once
#
# ⚠️ 해상도별로 따로 적어둔다. 스케일 환산은 하지 않는다 — fx/width가 1280x720에서 0.71057,
#    424x240에서 0.71504로 0.6% 어긋나 정확한 선형 스케일이 아니다(실측).
FACTORY_INTRINSICS = {          # (width, height): (fx, fy, cx, cy)
    (1280, 720): (909.52978515625, 909.1972045898438, 659.5433959960938, 370.19598388671875),
    (424, 240): (303.1766052246094, 303.06573486328125, 218.51446533203125, 123.39866638183594),
}
# color 스트림은 드라이버가 이미 보정해 내보내므로 왜곡계수가 전부 0으로 발행된다(실측).
FACTORY_DIST = np.zeros((1, 5))

# False로 두면 예전처럼 이미지셋 추정값을 쓴다. 어느 쪽이 나은지 잔차로 비교할 수 있게 남긴다.
USE_FACTORY_INTRINSICS = True

# 자세쌍의 상대회전 A가 이보다 작으면 logR()이 0/0에 가까워 회전 추정에 기여를 못 한다.
# 두 곳(report_residuals / report_pose_quality)에서 같은 양을 재므로 한 곳에 둔다 —
# 예전엔 15°와 30°로 갈려 있어 같은 자세쌍이 한쪽에선 정상, 한쪽에선 회전부족이었다.
# UNVERIFIED: digest의 "자세마다 회전 30° 이상"에서 온 값이고 실측 근거는 없다.
A_ROT_MIN_DEG = 30.0

# 1) 로봇 그리퍼의 절대 좌표 (x, y, z, rx, ry, rz)를 행렬로 변환하는 함수
def get_robot_pose_matrix(x, y, z, rx, ry, rz):
    """
    베이스->그리퍼 변환행렬 (4x4)을 반환.
    """
    R = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

# 2) 체커보드 코너 검출 (카메라→체커보드 변환 구하기)
def find_checkerboard_pose(
    image, board_size, square_size, camera_matrix, dist_coeffs
):
    """
    board_size: 내부 코너 개수 (cols, rows) — 칸 개수가 아니다
    square_size: 한 칸 크기 (mm)
    이미지에서 체커보드를 찾고, solvePnP로 카메라→체커보드 변환(R, t)을 구함.
    반환값: (R_camera2checker, t_camera2checker)
    """
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    # 예: x 방향으로 square_size씩 증가, y 방향으로 square_size씩 증가
    objp[:, :2] = (
        np.mgrid[0 : board_size[0], 0 : board_size[1]].T.reshape(-1, 2) * square_size
    )

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray,
        board_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_FAST_CHECK
        + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return None, None

    # 코너 좌표를 더 정확히
    corners_sub = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )

    # solvePnP
    retval, rvec, tvec = cv2.solvePnP(objp, corners_sub, camera_matrix, dist_coeffs)
    if not retval:
        return None, None

    # 회전벡터 -> 회전행렬
    R, _ = cv2.Rodrigues(rvec)

    return R, tvec

# 체커보드 이미지를 이용한 카메라 보정
def calibrate_camera_from_chessboard(
    image_folder_path,
    board_size,  # (7, 5)처럼 내부 코너 개수
    square_size,  # mm 단위
):
    """
    지정된 폴더 안의 체커보드 이미지를 읽고, 카메라 행렬(camera_matrix)와 왜곡 계수(dist_coeffs)를 추정한다.
    board_size: 체커보드 내부 코너 수 (cols, rows)
    square_size: 체커보드 한 칸 크기 (mm)
    """
    # 3D 세계 좌표계에 대한 좌표 생성 (z=0 평면 상에 체커보드)
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    # 예: x 방향으로 square_size씩 증가, y 방향으로 square_size씩 증가
    objp[:, :2] = (
        np.mgrid[0 : board_size[0], 0 : board_size[1]].T.reshape(-1, 2) * square_size
    )

    # 모든 이미지에 대해 3D / 2D 포인트 누적
    obj_points = []  # 3D world points
    img_points = []  # 2D image points
    image_shape = None

    # 폴더 내에 있는 이미지 파일 읽기
    image_paths = image_folder_path  # JPG, PNG 등 확장자 맞춰서
    # 필요하면 jpg 등 다른 확장자도 처리 가능

    for fname in image_paths:
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_shape is None:
            image_shape = gray.shape[::-1]  # (width, height)
        elif gray.shape[::-1] != image_shape:
            # 공장값 도입 전엔 calibrateCamera 인자일 뿐이었지만, 지금은 FACTORY_INTRINSICS의
            # 딕셔너리 키다 — 해상도가 섞이면 나머지 전부에 틀린 내부파라미터가 적용된다.
            raise SystemExit(f"해상도가 섞였다: {image_shape} vs {gray.shape[::-1]} ({fname}). "
                             "한 해상도로 다시 수집할 것.")

        # 체커보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        if ret:
            # 코너를 더 정밀하게
            corners_sub = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            # 누적
            obj_points.append(objp)
            img_points.append(corners_sub)

    # 내부 파라미터, 왜곡 계수, 외부 파라미터 구하기
    if len(obj_points) < 1:
        print("체커보드 코너를 충분히 찾지 못하였습니다.")
        return None, None, None, None

    # flags = cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K3 등 필요에 따라 추가
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points,  # 3D 실세계 점
        img_points,  # 2D 이미지 점
        image_shape,  # (width, height)
        None,  # 초기 camera_matrix
        None,  # 초기 dist_coeffs
    )

    # ret은 bool이 아니라 **RMS 재투영오차(px)** 다. 예전엔 truthiness로만 봐서 사실상
    # 죽은 검사였고(0.0일 때만 걸린다), 정작 유일한 내부파라미터 품질 지표를 버리고 있었다.
    print(f"[내부파라미터] 코너검출 {len(obj_points)}/{len(image_paths)}장, "
          f"RMS 재투영오차 = {ret:.3f} px  (0.5 이하 양호 / 1.0 넘으면 재수집)")
    # 공장값을 쓰는 동안 이 RMS는 **결과 품질이 아니라 추정이 얼마나 나빴는지의 지표**다.
    # (추정값을 실제로 쓰던 시절의 "믿을 수 없다" 경고를 그대로 두면 오독을 부른다.)
    use_factory = USE_FACTORY_INTRINSICS and image_shape in FACTORY_INTRINSICS
    if ret > 1.0:
        print("  ⚠️ 추정 내부파라미터의 재투영오차가 크다 — "
              + ("공장값을 쓰므로 결과에는 반영되지 않는다(참고용)."
                 if use_factory else
                 "이 값으로 푼 손-눈 결과는 믿을 수 없다."))

    # 공장값으로 교체. 추정값은 버리지 않고 나란히 찍어 얼마나 어긋났는지 보이게 한다 —
    # 차이가 크면 추정이 나빴다는 뜻이고, 그게 손-눈 결과 오차의 상류 원인이다.
    if USE_FACTORY_INTRINSICS:
        factory = FACTORY_INTRINSICS.get(image_shape)
        if factory is None:
            print(f"  ⚠️ {image_shape} 해상도의 공장값이 없다 → 추정값을 그대로 쓴다. "
                  f"camera_info를 읽어 FACTORY_INTRINSICS에 추가할 것.")
        else:
            fx, fy, cx, cy = factory
            est = camera_matrix
            print(f"  공장값 대체:  fx {est[0,0]:7.2f}→{fx:7.2f}   fy {est[1,1]:7.2f}→{fy:7.2f}   "
                  f"cx {est[0,2]:7.2f}→{cx:7.2f}   cy {est[1,2]:7.2f}→{cy:7.2f}")
            # 왜곡계수도 버려진다. 주석만 "나란히 찍는다"고 해놓고 이건 조용히 갈아치우고
            # 있었다(2026-08-03 리뷰). 얼마나 버리는지 보이게 한다.
            print(f"  왜곡계수 폐기: 추정 k1,k2={dist_coeffs.flatten()[:2]} → 공장값 0 "
                  f"(color 스트림은 드라이버가 이미 보정해 내보낸다)")
            # 추정과 공장값이 크게 어긋나면 둘 중 하나가 틀렸다는 신호다. 조용히 넘기지 않는다.
            dev = max(abs(est[0, 0] / fx - 1), abs(est[1, 1] / fy - 1))
            if dev > 0.10:
                print(f"  ⚠️ 추정 초점거리가 공장값과 {dev * 100:.0f}% 어긋난다. 보드 거리 다양성이"
                      " 심하게 부족하거나 square_size가 틀렸을 수 있다 — 공장값을 쓰더라도"
                      " 코너/자세 데이터 자체를 의심할 것.")
            camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
            dist_coeffs = FACTORY_DIST

    return camera_matrix, dist_coeffs, rvecs, tvecs

from scipy.linalg import sqrtm
from numpy.linalg import inv

# 4) 여러 개의 변환 행렬 조합 함수
def compose_transformation_matrices(R_list, t_list):
    T_list = []
    for R, t in zip(R_list, t_list):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.ravel(t)  # t가 벡터 형태여야 합니다.
        T_list.append(T)
    return T_list

# 회전행렬 -> 회전벡터(축*각도). Park-Martin이 회전을 벡터로 다뤄야 해서 필요하다.
def logR(T):
    R = T[0:3, 0:3]                              # 4x4에서 회전부만
    theta = np.arccos((np.trace(R) - 1) / 2)     # trace(R)=1+2cos(theta) 에서 회전각
    logr = np.array([
        R[2, 1] - R[1, 2],                       # 반대칭부 (R - R^T)/2 의 x 성분 *2
        R[0, 2] - R[2, 0],                       #                        y
        R[1, 0] - R[0, 1]                        #                        z
    ]) * theta / (2 * np.sin(theta))             # ⚠️ theta≈0이면 0/0 → NaN. 자세쌍에 회전을 충분히 줄 것
    return logr

# AX = XB 를 Park-Martin 방식으로 푼다. 회전(theta)을 먼저 닫힌형으로, 그 다음 병진(b_x)을 최소제곱으로.
def Calibrate(A, B):
    n_data = len(A)
    M = np.zeros((3, 3))                         # M = sum(beta * alpha^T) — 회전 추정의 상관행렬

    # [1단계] 회전. alpha(=logR(A))와 beta(=logR(B))는 같은 물리 회전을 서로 다른 프레임에서 본 것이므로
    #         alpha = theta @ beta 를 만족하는 theta를 찾는다. 축 2개만으론 3차원이 안 잡혀서
    #         외적으로 만든 세 번째 축까지 넣는다(연속한 두 자세쌍을 함께 쓰는 이유).
    for i in range(n_data - 1):
        alpha  = logR(A[i])                      # 로봇 쪽 i번째 상대회전 (base 프레임에서 본 것)
        beta   = logR(B[i])                      # 카메라 쪽 i번째 상대회전 (cam 프레임에서 본 것)
        alpha2 = logR(A[i + 1])                  # 다음 쌍 — 축이 다른 회전이어야 의미가 있다
        beta2  = logR(B[i + 1])

        alpha3 = np.cross(alpha, alpha2)         # 두 축의 외적 = 세 번째 독립 축
        beta3  = np.cross(beta, beta2)           # ⚠️ alpha//alpha2(같은 축 회전만)면 0벡터 → rank 부족

        M1 = np.dot(beta.reshape(3, 1), alpha.reshape(1, 3))    # beta·alpha^T (3x3)
        M2 = np.dot(beta2.reshape(3, 1), alpha2.reshape(1, 3))
        M3 = np.dot(beta3.reshape(3, 1), alpha3.reshape(1, 3))

        M += M1 + M2 + M3                        # 모든 자세쌍에 대해 누적

    theta = np.dot(sqrtm(inv(np.dot(M.T, M))), M.T)   # (M^T M)^(-1/2) M^T = M의 가장 가까운 직교행렬

    # [2단계] 병진. AX=XB의 병진부는  R_a·b_x + t_a = theta·t_b + b_x
    #         → (I - R_a)·b_x = t_a - theta·t_b   를 모든 자세에 쌓아 최소제곱으로 푼다.
    C = np.zeros((3 * n_data, 3))                # 좌변 계수 (3n x 3)
    d = np.zeros((3 * n_data, 1))                # 우변 (3n x 1)
    for i in range(n_data):
        rot_a = A[i][:3, :3]                     # R_a
        trans_a = A[i][:3, 3]                    # t_a (mm)
        trans_b = B[i][:3, 3]                    # t_b (mm)
        C[3 * i:3 * i + 3, :] = np.eye(3) - rot_a
        d[3 * i:3 * i + 3, 0] = trans_a - np.dot(theta, trans_b)

    b_x = np.dot(inv(np.dot(C.T, C)), np.dot(C.T, d))  # 정규방정식 해 = X의 병진 (mm)
    return theta, b_x

def report_residuals(A, B, theta, b_x):
    """푼 X가 AX=XB를 얼마나 만족하는지 자세쌍별로 찍는다.

    왜 필요한가: 이 스크립트는 지금까지 결과 행렬만 출력했다. 그래서 같은 카메라를
    30분 간격으로 두 번 캘리브해 148 mm / 3.3° 벌어져도(2026-08-03 data vs data1)
    **어느 자세가 범인인지 알 방법이 없었다.** 잔차가 큰 쌍이 범인이다.

    보는 법 — **병진잔차가 주 지표다. LOO는 보조 지표이며 아래 한계가 있다.**
      - 회전량(A)이 작은 쌍: logR이 0/0에 가까워 회전 추정에 기여를 못 한다.
        수집할 때 자세마다 회전을 크게 주라는 신호.
      - 병진잔차가 큰 쌍: 그 자세의 posx 또는 코너검출이 틀렸다. 그 장을 지우고 재계산.
      - LOO 퍼짐: 쌍 하나를 빼면 결과가 얼마나 흔들리는지.

    ⚠️ **LOO를 "재현성"으로 읽지 말 것** (2026-08-03 리뷰에서 반증됨):
      ① `Calibrate`의 회전 추정이 리스트 **인접 순서**에 의존한다(`alpha2 = logR(A[i+1])`).
         데이터를 하나도 안 바꾸고 순서만 섞어도 결과가 중앙값 2.0 / 최대 12.6 mm 움직인다.
         LOO 수치가 이 페어링 재편성 노이즈와 구분되지 않는다.
      ② LOO는 **계통오차에 원리적으로 눈이 멀다.** N개 중 하나만 빼므로 항상 작게 나온다.
         실제로 잔차 큰 쌍들을 한꺼번에 빼면 결과가 80 mm 옮겨간 반면 LOO는 3.6 mm였다.
      → LOO가 작다는 건 "자세 하나에 끌려가지는 않는다"까지만 말한다. 정확도의 근거가 아니다.

    ⚠️ **임계값의 근거 (2026-08-09 재감사 — 이전 주석은 낡아 있었다)**
      - 20 mm의 근거로 적혀 있던 `octomap_resolution: 0.02`는 **더 이상 유효하지 않다.**
        그 값은 `moveit.launch.py:162`의 `setdefault` 폴백일 뿐이고, 실제로는
        `sensors_3d.yaml:30`의 **0.05**가 이긴다(2026-08-09 실행중 move_group에
        `ros2 param get /move_group octomap_resolution` → 0.05 로 확인).
      - 그렇다고 임계를 50 mm로 **늘리면 안 된다.** octomap voxel은 장애물 회피 쪽 요구고,
        이 캘리브의 진짜 소비자는 **grasp 좌표**다(pick_fsm). 둘 중 **엄한 쪽**이 임계를 정한다.
    # UNVERIFIED: 20/30 mm, 회전 15°, RMS 1.0/0.5 px 모두 이제 실측 근거가 없다.
    # RG2 grasp 성공률로 "몇 mm까지 허용되는가"를 재서 교체할 것. 그전까지는 보수적으로 20 mm 유지.
    """
    print("\n[AX=XB 잔차] (X = 방금 푼 base<-cam)")
    print("  쌍   A회전량   회전잔차   병진잔차")
    rot_res, trans_res = [], []
    for i, (Ai, Bi) in enumerate(zip(A, B)):
        Ra, ta = Ai[:3, :3], Ai[:3, 3]
        Rb, tb = Bi[:3, :3], Bi[:3, 3]
        # 회전: Ra·theta 와 theta·Rb 가 같아야 한다 → 둘 사이 각도가 잔차
        dR = (theta @ Rb).T @ (Ra @ theta)
        e_rot = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
        # 병진: (I-Ra)·b_x = ta - theta·tb 의 좌우 차이
        e_t = np.linalg.norm((np.eye(3) - Ra) @ b_x.flatten() - (ta - theta @ tb))
        a_mag = np.degrees(np.arccos(np.clip((np.trace(Ra) - 1) / 2, -1, 1)))
        # 두 조건은 배타가 아니다. 예전엔 회전부족을 먼저 보고 else로 넘겨서,
        # **병진잔차가 가장 큰 쌍이 '회전 부족'이라는 약한 라벨만 받고** 빼야 할 후보에서
        # 빠졌다(2026-08-03 리뷰: 143.7 mm 쌍이 그렇게 가려졌다). 둘 다 붙인다.
        flag = ('  ← 회전부족' if a_mag < A_ROT_MIN_DEG else '') + ('  ← 병진잔차 큼' if e_t > 30 else '')
        print(f"  {i:3d}  {a_mag:6.1f}°  {e_rot:7.2f}°  {e_t:7.1f} mm{flag}")
        rot_res.append(e_rot)
        trans_res.append(e_t)
    med_t, n_bad = np.median(trans_res), sum(e > 30 for e in trans_res)
    print(f"  중앙값: 회전 {np.median(rot_res):.2f}°, 병진 {med_t:.1f} mm"
          f"   (병진잔차 30 mm 초과: {n_bad}/{len(trans_res)}쌍)")

    # leave-one-out: 쌍 하나씩 빼고 다시 풀어 결과가 얼마나 움직이는지 본다.
    # ponytail: 부트스트랩 대신 LOO. 재계산이 싸다(이미지 처리는 이미 끝났다).
    # 다만 docstring의 한계 두 가지 때문에 **판정 근거로는 쓰지 않는다** — 참고 출력이다.
    if len(A) - 1 >= 3:
        origins = np.array([Calibrate(A[:i] + A[i + 1:], B[:i] + B[i + 1:])[1].flatten()
                            for i in range(len(A))])
        spread = np.linalg.norm(origins - np.asarray(b_x).flatten(), axis=1)
        print(f"\n[LOO 참고] 쌍 하나를 빼면 카메라 위치가 중앙값 {np.median(spread):.1f} mm, "
              f"최대 {spread.max():.1f} mm 움직인다 (쌍 {int(np.argmax(spread))})")
        print("  ※ 이 값이 작아도 정확하다는 뜻이 아니다 — docstring의 ①② 참고.")

    # ---- 최종 판정 ----------------------------------------------------------
    # 예전엔 LOO만 보고 판정해서, 병진잔차 40 mm에 21/31쌍이 이상인 데이터에도 마지막 줄이
    # "양호"로 나갔다(2026-08-03 리뷰). 마지막 줄만 읽는 사람에게 통과로 보이는 게 가장 위험하다.
    #
    # 2026-08-09 재감사에서 그 위험이 **세 군데 더** 남아 있던 것을 고쳤다:
    #   ① 표본 수 하한이 없었다 — 32장을 찍고 코너가 5장에서만 잡혀 **4쌍**으로 푼 데이터가
    #      그대로 판정을 받았다. 4개짜리 중앙값은 이미지 한 장이 지배한다.
    #   ② `med_t < 20`이면 **이상치 개수를 보지 않고** 곧장 "양호"였다. 잔차 큰 쌍이 절반이어도
    #      중앙값만 작으면 통과한다 — 위 2026-08-03 버그와 같은 형태가 첫 분기에 남아 있었다.
    #   ③ **회전잔차가 판정에 전혀 안 들어갔다.** 고정 카메라에서 회전오차는 거리로 증폭된다:
    #      1.65 m에서 2.8°면 횡방향 81 mm다. 병진잔차보다 크게 나올 수 있는데 무시되고 있었다.
    #      → 각도가 아니라 **그 각도가 실제로 만드는 mm**로 환산해 같은 잣대에 올린다.
    n = len(trans_res)
    med_r = float(np.median(rot_res))
    cam_dist_mm = float(np.linalg.norm(np.asarray(b_x).flatten()))
    lat_r = cam_dist_mm * np.tan(np.radians(med_r))   # 회전잔차의 횡방향 환산 (mm)

    fails = []
    # UNVERIFIED: 8쌍. AX=XB 자체는 회전축이 다른 2쌍이면 풀리지만, "풀린다"와 "중앙값이
    # 통계로 의미가 있다"는 다르다. 실측 근거는 없고 4쌍이 명백히 부족하다는 사실만 있다.
    if n < 8:
        fails.append(f"표본부족 {n}쌍 (<8) — 중앙값을 신뢰할 수 없다")
    if med_t >= 20:
        fails.append(f"병진잔차 중앙값 {med_t:.1f} mm (≥20)")
    if n_bad > n / 3:
        fails.append(f"이상치 {n_bad}/{n}쌍이 30 mm 초과 (>1/3)")
    if lat_r >= 20:
        fails.append(f"회전잔차 {med_r:.2f}° = {cam_dist_mm/1000:.2f} m에서 횡방향 {lat_r:.0f} mm (≥20)")

    print(f"\n[판정] 병진 중앙값 {med_t:.1f} mm / 회전 {med_r:.2f}°(≈{lat_r:.0f} mm 횡) / {n}쌍 — ", end='')
    if not fails:
        verdict = "양호"
        print("양호")
    else:
        verdict = "불합격"
        print("⚠️ 불합격")
        for f in fails:
            print(f"         - {f}")
        print("         이상치 쌍을 빼고 재계산하거나 재수집할 것.\n"
              "         (그 쌍들을 빼면 결과가 수십 mm 옮겨간 전례가 있다 — 무시하고 쓰지 말 것)")

    # 이 숫자를 stdout에만 두면 터미널을 닫는 순간 없어진다. 실제로 커밋된 npy가
    # 어느 잔차로 나왔는지 추적 불가능했다(2026-08-06 감사). main()이 npy 옆에 같이 저장한다.
    return {
        "n_pairs": n,
        "median_trans_mm": round(float(med_t), 2),
        "median_rot_deg": round(med_r, 2),
        "rot_lateral_mm": round(float(lat_r), 1),   # 회전잔차를 카메라 거리에서 mm로 환산한 값
        "max_trans_mm": round(float(np.max(trans_res)), 2),
        "n_trans_over_30mm": int(n_bad),
        "verdict": verdict,
        "fail_reasons": fails,
    }


def report_pose_quality(samples, A_list):
    """수집한 자세가 캘리브에 쓸 만한 분포인지 **결과를 풀기 전에** 채점한다 (읽기 전용).

    왜 필요한가: 잔차(report_residuals)는 이미 푼 뒤에야 나온다. 20분 걸려 수집하고
    "40 mm 불합격"을 보는 대신, 수집 직후에 자세 분포만 보고 재수집을 결정하려는 것이다.
    2026-08-03 수집은 34장으로 26장보다 **늘렸는데 잔차가 나빠졌다** — 양이 아니라
    분포의 문제였고, 그 분포를 보는 눈이 코드에 없었다.

    두 가지 조건은 **별개**다. 섞어 읽지 말 것:

      [1] 보드 기울기 (Zhang 1998, MSR-TR-98-71) — *내부파라미터* 추정 조건.
          Proposition 1: "If the model plane at the second position is parallel to its
          first position, then the second homography does not provide additional
          constraints." 평행한 평면은 무한원면에서 같은 circular point와 만나므로
          같은 제약을 반복할 뿐이다. 논문 실험(Sect 5.1): θ=5°에서 시행의 40%가 실패,
          "Best performance seems to be achieved with an angle around 45 deg".
          ⚠️ 단 논문 각주: 각도가 커지면 foreshortening으로 코너검출이 나빠진다(실험에
          미반영). 그래서 45°가 상한 목표지 클수록 좋은 게 아니다.
          여기서 재는 θ = 보드 법선과 카메라 광축(z) 사이 각 = 보드면과 이미지면 사이 각.

      [2] 상대회전 A의 축 다양성 — *AX=XB(Park-Martin)* 조건. 이쪽이 지금 잔차를 지배한다
          (USE_FACTORY_INTRINSICS=True라 [1]은 결과에 직접 안 들어간다).
          Calibrate()가 alpha3 = cross(alpha, alpha2)로 세 번째 축을 만드는데,
          인접한 두 상대회전의 축이 평행하면 이 외적이 0벡터가 되어 M의 rank가 준다.

    조인트를 크게 움직이는 것과 무관하다. J1을 크게 휘둘러도 손목이 보정하면 보드는
    카메라에 대해 그대로다. 손목(J4/J5/J6)으로 보드를 비트는 것이 두 조건 모두의 수단이다.
    """
    # UNVERIFIED: 20/50°(기울기 목표대), 30°(A 회전량), 0.95(축 평행 판정), 10°(법선 쌍각),
    # 0.10(법선 rank), 5°(축이 의미를 갖는 최소 회전량)은 구현자가 정한 값이다.
    # 근거가 있는 건 Zhang의 45°(위 인용)와 digest의 "회전 30° 이상"뿐이다.
    TILT_LO, TILT_HI, AXIS_PARALLEL, NORMAL_MIN = 20.0, 50.0, 0.95, 10.0
    A_MIN = A_ROT_MIN_DEG    # report_residuals와 같은 임계값을 쓴다(모듈 상단)
    RANK_MIN = 0.10          # 법선 산포의 최소/최대 특이값 비. 작으면 법선이 한 평면·한 선에 눕는다
    AXIS_VALID = 5.0         # 이보다 작은 회전의 축은 로봇 반복도 노이즈다 — 축으로 취급하지 않는다

    if len(samples) < 2:
        print(f"\n[자세 품질] 코너검출된 장이 {len(samples)}개뿐이다 — 분포를 잴 수 없다. "
              "보드 제원(내부 코너 수/칸 크기)과 이미지를 먼저 확인할 것.")
        return

    print("\n[자세 품질] 푸는 것과 무관한 수집 분포 진단 (읽기 전용)")
    fx_note = ("공장 내부파라미터 사용 중(USE_FACTORY_INTRINSICS=True) — "
               "[1]은 내부파라미터 추정에 직접 반영되지 않는다. 그래도 보는 이유: "
               "기울기가 0에 가까우면 solvePnP의 평면 자세 추정이 불안정해져 B가 나빠진다."
               if USE_FACTORY_INTRINSICS else
               "추정 내부파라미터 사용 중 — [1]이 결과에 직접 들어간다.")
    print(f"  ※ {fx_note}")
    print("\n  [1] 보드 기울기 — 이미지면 대비 보드면 각도 (Zhang: 45° 부근 최적, 0°는 퇴화)")
    print("    장   기울기   거리      비고")
    tilts, dists, n_behind = [], [], 0
    for i, (path, T_cam2board) in enumerate(samples):
        # 보드 법선: 보드 프레임의 +z를 카메라 프레임으로. 광축은 카메라 프레임의 +z.
        n_cam = T_cam2board[:3, :3] @ np.array([0.0, 0.0, 1.0])
        # 코너 10x7(짝x홀)이라 180° 모호성이 없어 법선은 카메라 쪽(n_z>0)이어야 한다.
        # 예전엔 abs()로 예각을 취했는데, 그러면 n_z<0인 장 — 즉 **PnP가 틀린 장** — 이
        # 그럴듯한 예각으로 바뀌어 조용히 통과한다(cross-review 지적). 세어서 드러낸다.
        behind = n_cam[2] < 0
        n_behind += behind
        tilt = np.degrees(np.arccos(np.clip(abs(n_cam[2]), 0.0, 1.0)))
        dist = np.linalg.norm(T_cam2board[:3, 3])
        flag = ('  ← 거의 평행(퇴화)' if tilt < TILT_LO else
                '  ← 과도(코너검출 열화)' if tilt > TILT_HI else '')
        flag += '  ← ⚠️ 법선이 카메라 반대쪽(PnP 의심)' if behind else ''
        print(f"    {i:3d}  {tilt:6.1f}°  {dist:7.1f} mm{flag}   {Path(path).name[:28]}")
        tilts.append(tilt)
        dists.append(dist)
    n_flat = sum(t < TILT_LO for t in tilts)
    print(f"    중앙값 {np.median(tilts):.1f}°, 범위 {min(tilts):.1f}~{max(tilts):.1f}°"
          f"   (20° 미만: {n_flat}/{len(tilts)}장)")

    # 위의 기울기만으로는 Zhang의 조건을 못 잰다. 기울기는 법선의 극각(polar)만 보므로
    # **둘 다 43°인데 법선 방향이 같은**(= 서로 평행한) 두 장을 통과시킨다.
    # Proposition 1이 금지하는 건 "평평함"이 아니라 "서로 평행함"이다 → 법선 쌍각을 직접 잰다.
    normals = np.array([T[:3, :3] @ np.array([0.0, 0.0, 1.0]) for _, T in samples])
    pair_angles = [
        np.degrees(np.arccos(np.clip(abs(float(np.dot(normals[i], normals[j]))), 0.0, 1.0)))
        for i in range(len(normals)) for j in range(i + 1, len(normals))
    ]
    n_par_board = sum(a < NORMAL_MIN for a in pair_angles)
    frac_par = n_par_board / len(pair_angles)
    print(f"    법선 쌍각(서로 평행한가): 최소 {min(pair_angles):.1f}°, 중앙값 "
          f"{np.median(pair_angles):.1f}°   ({NORMAL_MIN:.0f}° 미만 쌍: {n_par_board}"
          f"/{len(pair_angles)} = {100 * frac_par:.0f}%)")

    # 쌍각만으로는 부족하다(cross-review 지적): 손목 한 축만 흔들면 법선들이 **한 평면을
    # 쓸고 지나가** 쌍각은 전부 크게 나오는데도 내부파라미터가 ambiguous하다
    # (Sturm-Maybank류 퇴화). 법선 집합이 3차원을 채우는지 특이값으로 직접 본다.
    sv = np.linalg.svd(normals, compute_uv=False)
    rank_ratio = float(sv[2] / sv[0])
    print(f"    법선 산포 특이값 {np.array2string(sv, precision=2)} → 최소/최대 "
          f"{rank_ratio:.3f}   ({RANK_MIN} 미만이면 법선이 한 평면·한 선에 눕는다)")
    print(f"    거리 범위 {min(dists):.0f}~{max(dists):.0f} mm "
          f"(비 {max(dists) / min(dists):.2f}배 — 1.0에 가까우면 fx/Z 스케일 모호성)")

    print("\n  [2] 상대회전 A — 회전량과 인접 축의 평행도 (Park-Martin의 rank 조건)")
    print("    쌍   A회전량   인접축각   비고")
    axes, mags = [], []
    for Ai in A_list:
        Ra = Ai[:3, :3]
        ang = np.degrees(np.arccos(np.clip((np.trace(Ra) - 1) / 2, -1, 1)))
        v = np.array([Ra[2, 1] - Ra[1, 2], Ra[0, 2] - Ra[2, 0], Ra[1, 0] - Ra[0, 1]])
        n = np.linalg.norm(v)
        # 가드가 부동소수 하한(1e-9)뿐이면 θ=1.2°짜리 축도 통과해 표에 소수점까지 찍힌다 —
        # 그 축은 로봇 반복도 노이즈다(cross-review 지적). **물리적 하한**으로 자른다.
        valid = ang >= AXIS_VALID and n > 1e-9
        axes.append(v / n if valid else np.zeros(3))
        mags.append(ang)
    n_par = 0
    for i in range(len(A_list)):
        if i + 1 < len(A_list) and np.linalg.norm(axes[i]) > 0 and np.linalg.norm(axes[i + 1]) > 0:
            c = abs(float(np.dot(axes[i], axes[i + 1])))
            sep = f"{np.degrees(np.arccos(np.clip(c, 0, 1))):6.1f}°"
            par = c > AXIS_PARALLEL
            n_par += par
        else:
            sep, par = "  회전<5°" if mags[i] < AXIS_VALID else "     -", False
        flag = ('  ← 회전부족' if mags[i] < A_MIN else '') + ('  ← 인접축 평행(외적≈0)' if par else '')
        print(f"    {i:3d}  {mags[i]:6.1f}°  {sep}{flag}")
    n_small = sum(m < A_MIN for m in mags)
    print(f"    중앙값 {np.median(mags):.1f}°   (30° 미만: {n_small}/{len(mags)}쌍, "
          f"인접축 평행: {n_par}쌍)")

    # 판정은 **계산한 지표를 전부** 쓴다. 예전 버전은 n_par_board(법선 쌍각)와 TILT_HI를
    # 계산해놓고 처방에서 n_flat만 봤다 — 계산한 것과 판정하는 것이 달랐다(cross-review 지적).
    n_hi = sum(t > TILT_HI for t in tilts)
    findings = []
    if n_flat > len(tilts) / 3:
        findings.append("보드가 카메라와 거의 평행한 장이 많다 → 손목으로 20~50° 비틀어 재수집.")
    if frac_par > 0.25:
        findings.append(f"서로 평행한 보드 자세쌍이 {100 * frac_par:.0f}%다 (Zhang Prop 1: "
                        "평행하면 새 제약을 못 준다) → 법선 방향 자체를 흩을 것.")
    if rank_ratio < RANK_MIN:
        findings.append(f"법선이 3차원을 못 채운다(비 {rank_ratio:.3f}) → 손목 한 축만 흔든 "
                        "수집이다. 회전축을 두 축 이상으로 바꿀 것.")
    if n_hi > len(tilts) / 3:
        findings.append(f"기울기 {TILT_HI:.0f}° 초과가 {n_hi}장 → foreshortening으로 코너검출이 "
                        "나빠진다(Zhang 각주). 45°를 넘기지 말 것.")
    if n_behind:
        findings.append(f"법선이 카메라 반대쪽인 장 {n_behind}개 → 그 장의 solvePnP가 틀렸다. "
                        "빼고 재계산할 것.")
    if n_small > len(mags) / 3 or n_par > len(mags) / 3:
        findings.append(f"상대회전이 작거나({n_small}쌍) 축이 겹친다({n_par}쌍) → 자세마다 "
                        f"{A_MIN:.0f}° 이상, 회전축을 X/Y/Z로 바꿔가며.")
    if max(dists) / min(dists) < 1.3:
        findings.append("보드 거리가 한 값에 몰려 있다 → 내부파라미터를 재추정할 거면 거리도 흩을 것.")
    # 🔴 `samples`에는 **코너 검출에 성공한 장만** 들어온다. 검출 실패로 대부분이 빠져도
    # 남은 소수의 분포는 얼마든지 좋을 수 있어 "결함 0건"이 나온다 — 2026-08-09에 실제로
    # 32장 수집 / 5장 생존인 데이터가 이 함수에서 "뚜렷한 결함 없음"을 받았다.
    # report_residuals의 표본수 하한과 같은 이유로, 여기서도 생존 장수를 먼저 본다.
    if len(samples) < 8:
        findings.insert(0, f"🔴 코너 검출에 성공한 장이 {len(samples)}장뿐이다(<8) — 분포 이전에 "
                           "표본이 없다. 해상도(1280x720)·조명·보드 평면도부터 확인하고 재수집할 것.")

    # 마지막 줄만 읽는 사람에게 잘못된 통과 신호를 주지 않는다 — report_residuals와 같은 이유로
    # [판정]을 명시한다. 임계값 대부분이 UNVERIFIED라는 사실도 여기서 한 번 더 말한다.
    print("\n  [처방]")
    for f in findings:
        print(f"    · {f}")
    print(f"\n  [판정] 분포 결함 {len(findings)}건 — ", end='')
    if not findings:
        print("뚜렷한 결함 없음. 잔차가 크면 원인은 분포 밖에 있다\n"
              "         (보드 강성/평면도, posx 기록 시점, square_size, 카메라 마운트 흔들림).")
    else:
        print("위 항목을 고쳐 재수집할 것.")
    print("  ※ 이 판정은 **수집 분포만** 본다. 정확도의 근거가 아니다 — 잔차는 "
          "--pose-quality 없이 돌려 report_residuals로 확인할 것.\n"
          "  ※ 임계값 대부분이 코드 주석의 UNVERIFIED다. 현장 요구정밀도로 교체할 것.")


def _selfcheck():
    """합성 데이터로 파이프라인 방향(X == T_base<-cam)이 맞는지 확인한다.

    실기 없이 도는 유일한 검산이다. 여기서 깨지면 데이터가 아니라 코드가 틀린 것.
    """
    rng = np.random.default_rng(0)

    C_true = np.eye(4)                                        # 정답 T_base<-cam
    C_true[:3, :3] = Rotation.from_euler("xyz", [160, 5, -95], degrees=True).as_matrix()
    C_true[:3, 3] = [800.0, -100.0, 900.0]                    # mm

    G_true = np.eye(4)                                        # 정답 T_gripper<-board (소거되어야 함)
    G_true[:3, :3] = Rotation.from_euler("xyz", [3, -7, 20], degrees=True).as_matrix()
    G_true[:3, 3] = [12.0, -5.0, 140.0]

    T_g2b_list, T_ck2cam_list = [], []
    for _ in range(12):
        # 임의의 로봇 자세 T_base<-gripper 를 만들고
        T_b2g = np.eye(4)
        T_b2g[:3, :3] = Rotation.from_rotvec(rng.uniform(-1.2, 1.2, 3)).as_matrix()
        T_b2g[:3, 3] = rng.uniform(-400, 400, 3) + [400, 0, 400]
        # (*)식으로 그 자세에서 카메라가 볼 판의 pose T_cam<-board 를 역산한다
        T_cam2ck = np.linalg.inv(C_true) @ T_b2g @ G_true
        T_g2b_list.append(np.linalg.inv(T_b2g))               # 본편과 동일하게 inv를 담는다
        T_ck2cam_list.append(np.linalg.inv(T_cam2ck))

    A_list = [inv(T_g2b_list[i]) @ T_g2b_list[i + 1] for i in range(11)]
    B_list = [inv(T_ck2cam_list[i]) @ T_ck2cam_list[i + 1] for i in range(11)]
    theta, b_x = Calibrate(A_list, B_list)

    assert np.allclose(theta, C_true[:3, :3], atol=1e-6), f"회전 복원 실패\n{theta}\n{C_true[:3,:3]}"
    assert np.allclose(b_x.flatten(), C_true[:3, 3], atol=1e-3), f"병진 복원 실패 {b_x.flatten()}"
    print("selfcheck OK — X = T_base<-cam 이 맞고, G(판↔그리퍼)는 소거된다")


def main(save=True, pose_quality=False):
    # ---- 입력 로드 ----------------------------------------------------------
    data = json.load(open(DATA_DIR / "calibrate_data.json"))   # data_recording.py 산출물
    robot_poses = np.array(data["poses"])                      # (N, 6) = x,y,z,rx,ry,rz (mm, deg, ZYZ)
    image_paths = [str(DATA_DIR / d) for d in data["file_name"]]  # 같은 순서의 이미지 경로

    # ---- 자세 유효성 검사 ---------------------------------------------------
    # 동차변환의 det는 회전행렬 det와 같아 항상 1이어야 한다. 아니면 posx 파싱이 깨진 것.
    # 회전행렬로 만든 것이라 det는 사실상 항상 1이다 — 자세마다 한 줄씩 찍으면
    # 정작 중요한 잔차 출력이 스크롤 밖으로 밀린다. 걸린 것만 찍는다.
    valid_indices = []
    for i, pose in enumerate(robot_poses):
        T_base2gripper = get_robot_pose_matrix(*pose)          # T_base<-gripper
        if np.abs(np.linalg.det(T_base2gripper)) > 1e-6:
            valid_indices.append(i)
        else:
            print(f"⚠️ Warning: Singular T_base2gripper at index {i}!")
    print(f"[입력] {DATA_DIR.name}: 자세 {len(robot_poses)}개 중 {len(valid_indices)}개 유효")

    robot_poses = robot_poses[valid_indices]                   # 걸러진 것만 남긴다
    image_paths = [image_paths[i] for i in valid_indices]      # 이미지도 같은 인덱스로 맞춘다

    # ---- 보드 제원 ----------------------------------------------------------
    # 실물 보드: 11x8 칸 = 내부 코너 10x7, 한 칸 24mm (2026-08-03 사용자 캘리퍼스 실측 확인)
    checkerboard_size = (10, 7)  # 내부 코너 개수 (칸 개수 아님). 짝x홀이라 180° 모호성 없음
    square_size = 24.0           # mm. 이 값이 1% 틀리면 거리 추정이 통째로 1% 틀어진다

    # ---- 카메라 내부 파라미터 ------------------------------------------------
    # 같은 이미지셋으로 추정한다. 고정 카메라라 보드 거리 다양성이 부족하면 품질이 떨어지므로,
    # 결과가 나쁘면 RealSense 공장값(/camera/camera/color/camera_info)으로 대체하는 것을 검토.
    camera_matrix, dist_coeffs, rvecs, tvecs = calibrate_camera_from_chessboard(
        image_paths, checkerboard_size, square_size
    )

    R_gripper2base_list = []      # T_gripper<-base 의 회전부
    t_gripper2base_list = []      # 〃 병진부
    R_camera2checker_list = []    # (미사용 — 유지만)
    t_camera2checker_list = []    # (미사용 — 유지만)
    R_checker2camera_list = []    # T_board<-cam 의 회전부
    t_checker2camera_list = []    # 〃 병진부
    samples = []                  # (경로, T_cam<-board) — 자세 품질 진단용

    for img_path, pose in zip(image_paths, robot_poses):
        # 1) 로봇 posx → T_base<-gripper (판을 물고 있는 그리퍼의 베이스 기준 pose)
        T_base2gripper = get_robot_pose_matrix(*pose)

        # 2) 같은 순간에 찍은 이미지
        image = cv2.imread(img_path)
        if image is None:
            continue                                            # 파일 깨짐 → 이 자세 통째로 버린다

        # 3) 이미지에서 판의 pose 검출 → T_cam<-board (solvePnP 출력 방향)
        R_cam2checker, t_cam2checker = find_checkerboard_pose(
            image, checkerboard_size, square_size, camera_matrix, dist_coeffs
        )
        if R_cam2checker is None:
            continue                                            # 코너 미검출 → 자세/이미지 쌍이 깨지지 않게 같이 스킵

        # 4) 로봇 쪽을 뒤집어 T_gripper<-base 로 (AX=XB 유도에서 이 방향이 필요하다)
        T_gripper2base = np.linalg.inv(T_base2gripper)

        R_gripper2base = T_gripper2base[:3, :3]
        t_gripper2base = T_gripper2base[:3, 3]

        R_gripper2base_list.append(R_gripper2base.copy())
        t_gripper2base_list.append(t_gripper2base.reshape(-1, 1).copy())

        # 5) 카메라 쪽도 뒤집어 T_board<-cam 으로
        T_cam2checker = np.eye(4)
        T_cam2checker[:3, :3] = R_cam2checker
        T_cam2checker[:3, 3] = t_cam2checker.flatten()          # mm (square_size 단위를 그대로 따라감)
        T_checker2cam = np.linalg.inv(T_cam2checker)
        samples.append((img_path, T_cam2checker.copy()))

        R_checker2camera_list.append(T_checker2cam[:3, :3].copy())
        t_checker2camera_list.append(T_checker2cam[:3, 3].copy())

    # R,t 조각을 다시 4x4로 합친다
    T_gripper2base_list = compose_transformation_matrices(R_gripper2base_list, t_gripper2base_list)
    T_checker2cam_list = compose_transformation_matrices(R_checker2camera_list, t_checker2camera_list)
    A_list = []
    B_list = []
    # 3)/4)에서 스킵된 자세 때문에 길이가 어긋날 수 있어 짧은 쪽에 맞춘다
    num_pairs = min(len(T_gripper2base_list), len(T_checker2cam_list))

    for i, T in enumerate(T_gripper2base_list):
        det = np.linalg.det(T)
        if np.abs(det) < 1e-6:
            print(f"⚠️ Warning: T_gripper2base_list[{i}] is singular or nearly singular!")

    # 절대 pose가 아니라 **연속 자세 사이의 상대 이동**을 쓴다. 여기서 미지수 G가 소거된다(docstring 참고).
    for i in range(num_pairs - 1):
        A_i = np.dot(inv(T_gripper2base_list[i]), T_gripper2base_list[i + 1])   # 로봇 쪽 상대 이동
        B_i = np.dot(inv(T_checker2cam_list[i]), T_checker2cam_list[i + 1])     # 카메라 쪽 상대 이동
        A_list.append(A_i)
        B_list.append(B_i)

    # --pose-quality: 분포만 채점하고 끝낸다. 푸는 것도, 저장도 하지 않는다 —
    # "읽기 의도의 실행에 쓰기 부작용을 두지 않는다"(--no-save를 만든 것과 같은 이유).
    if pose_quality:
        report_pose_quality(samples, A_list)
        return

    theta, b_x = Calibrate(A_list, B_list)                      # AX=XB 풀이
    X = np.eye(4)
    X[:3, :3] = theta                                           # 회전 (base<-cam)
    X[:3, 3] = b_x.flatten()                                    # 병진 (mm, base 기준 카메라 위치)
    T_cam2base = X                                              # 이름은 cam2base지만 내용은 T_base<-cam

    stats = report_residuals(A_list, B_list, theta, b_x)         # 이 결과를 믿어도 되는지의 근거

    print("\n[결과] T_base<-cam")
    print(T_cam2base)
    print("카메라 위치(mm, base 기준):", T_cam2base[:3, 3],
          f" 거리 {np.linalg.norm(T_cam2base[:3, 3]) / 1000:.3f} m")  # 눈으로 검산: 삼각대 실측과 대충 맞는가
    # 저장 → src/.../m0609_rg2_bringup/config/ 로 복사하면 camera.launch.py가 static TF로 발행한다
    # 기본 data/ 만 정식 파일에 쓴다. 과거 버전(data1, data2...)을 재계산할 땐 이름을 나눠
    # 저장해야 비교하려다 현행 캘리브를 덮어쓰는 사고가 안 난다.
    suffix = '' if DATA_DIR.name == 'data' else f'_{DATA_DIR.name}'
    out = DATA_DIR.parent / f"T_cam2base{suffix}.npy"
    if not save:
        print(f"저장 안 함 (--no-save). 저장했다면: {out}")
        return
    np.save(out, T_cam2base)
    print(f"저장: {out}")

    # npy 옆에 근거를 같이 남긴다. npy는 값만 있어서 "이게 몇 mm 잔차로 나온 건지"가
    # 커밋 이력에서 사라진다 — 재캘리브마다 같은 공백이 반복됐다(2026-08-06 감사).
    # 해상도를 같이 적는 이유: 424x240으로 수집하면 코너 검출이 무너진다(2026-08-09 실측).
    h, w = cv2.imread(samples[0][0]).shape[:2] if samples else (0, 0)
    report = {
        "generated": datetime.now().isoformat(timespec="seconds"),
        "dataset": DATA_DIR.name,
        "output": out.name,
        "image_size": f"{w}x{h}",
        "n_poses": len(robot_poses),
        "n_usable_images": len(samples),      # 코너 검출 성공 = AX=XB에 실제로 기여한 장수
        "checkerboard": f"{checkerboard_size[0]}x{checkerboard_size[1]}@{square_size}mm",
        "camera_xyz_mm": [round(float(v), 2) for v in T_cam2base[:3, 3]],
        **stats,
    }
    (out.parent / f"calib_report{suffix}.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"저장: {out.parent / f'calib_report{suffix}.json'}  (판정 {stats['verdict']})")


# Main Function
if __name__ == "__main__":
    import sys
    if "--selfcheck" in sys.argv:
        _selfcheck()
    else:
        # --no-save: 진단만 보고 npy는 건드리지 않는다.
        # 왜 있나: 2026-08-03에 리뷰용으로 이 스크립트를 "읽기만" 하려고 돌렸다가
        # 실기 TF의 소스인 T_cam2base.npy가 조용히 덮어써졌다(값은 결정적이라 같았지만
        # 운이 좋았을 뿐이다). 진단 실행에 쓰기 부작용이 있는 게 원인이므로 끌 수 있게 한다.
        # --pose-quality: 수집 분포만 채점(읽기 전용). 풀지도 저장하지도 않는다.
        pose_quality = "--pose-quality" in sys.argv
        save = "--no-save" not in sys.argv
        args = [a for a in sys.argv[1:] if a not in ("--no-save", "--pose-quality")]
        if any(a.startswith("-") for a in args):       # -h 등 모르는 플래그를 디렉토리로 읽지 않게
            raise SystemExit(__doc__)
        # 버전 디렉토리를 인자로 받는다: python3 eye2hand_calibration.py data1
        # (data_recording.py는 항상 data/에 쓰므로, 과거 수집분과 비교할 때 쓴다)
        if args:
            DATA_DIR = Path(__file__).resolve().parent / args[0]
            if not DATA_DIR.is_dir():
                raise SystemExit(f"디렉토리 없음: {DATA_DIR}")
        main(save=save, pose_quality=pose_quality)
