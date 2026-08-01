---
tags: [math, 면접, 좌표변환]
축: 1 (내 코드에 있는 수학)
---

# 좌표 변환 — 내 코드에 실제로 있는 선형대수

> 이 문서는 `cobot2_ws` 저장소의 코드에서 **실제로 확인된 것만** 적는다.
> 파일:라인이 없는 개념은 [[ws/cobot2/math/없는-것|축 2]]로 미룬다.
> NotebookLM 소스로 올릴 때는 이 파일 그대로 올린다.

## 0. 한 장 요약 — 컵 하나 집는 데 쓰이는 변환 사슬

```
YOLO bbox 중심 (u, v) + depth z
        │  ① 핀홀 역투영  (내부 파라미터 fx, fy, ppx, ppy)
        ▼
카메라 좌표계  P_cam = (X, Y, Z)
        │  ② gripper2cam  (hand-eye 캘리브 결과, .npy로 저장된 상수)
        ▼
그리퍼 좌표계
        │  ③ base2gripper (로봇이 매 순간 알려주는 현재 자세, ZYZ 오일러)
        ▼
베이스 좌표계  → movel(...)로 이 좌표에 간다
```

코드에서는 ②③이 한 줄로 합쳐져 있다:

```python
base2cam = base2gripper @ gripper2cam      # robot_control.py:87
td_coord = np.dot(base2cam, coord)         # robot_control.py:88
```

**면접에서 이 그림 하나를 그릴 수 있으면 절반은 끝난다.**

---

## 1. 동차변환행렬 (Homogeneous Transformation Matrix)

`src/robot_control/robot_control/robot_control.py:68-73`

```python
def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
    R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T
```

$$
T = \begin{bmatrix} R & t \\ 0^\top & 1 \end{bmatrix} \in SE(3),
\qquad R \in SO(3),\ t \in \mathbb{R}^3
$$

**왜 4×4로 부풀리나?** 회전은 행렬 곱, 평행이동은 덧셈이라 $Rp + t$ 형태다.
연산이 섞이면 여러 변환을 하나로 합칠 수 없다. 마지막에 1을 붙여
$\tilde p = (x,y,z,1)^\top$ 로 만들면 **평행이동도 행렬 곱이 되어**,
변환 합성이 그냥 행렬 곱 하나가 된다.

```python
coord = np.append(np.array(camera_coords), 1)   # robot_control.py:81 — 이 1이 그 1이다
```

**합성 순서가 전부다.** `base2cam = base2gripper @ gripper2cam`은
"카메라 좌표를 먼저 그리퍼로, 그다음 베이스로" 라는 뜻이다.
오른쪽이 먼저 적용된다. 순서를 뒤집으면 조용히 엉뚱한 곳으로 간다 —
**에러가 안 난다.** 이게 좌표 변환 버그가 잡기 어려운 이유다.

### 성질 (면접에서 묻는 것)
- $R^\top R = I$ (직교행렬), $\det R = +1$ → **회전만 있고 반사·크기변화 없음**
- $R^{-1} = R^\top$ → 역변환이 공짜. 4×4 역행렬은
  $T^{-1} = \begin{bmatrix} R^\top & -R^\top t \\ 0^\top & 1\end{bmatrix}$
- $\det T = \det R \cdot 1 = 1$ — **항상 1이다.** (§6에서 이게 왜 중요한지 나온다)

---

## 2. ZYZ 오일러각 → 회전행렬

`robot_control.py:69` · `pick_and_place_text/robot_move.py:60` · `corecode/Calibration_Tutorial/eye2hand_calibration.py:11`

```python
Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
```

$$R = R_z(\alpha)\,R_y(\beta)\,R_z(\gamma)$$

**왜 ZYZ인가?** 두산 로봇 컨트롤러가 자세를 ZYZ 오일러각(도 단위)으로 준다.
내가 고른 게 아니라 **하드웨어 규약을 따른 것**이다. 면접에서 "왜 쿼터니언 안 썼냐"고
물으면 이렇게 답하면 된다 — 입력이 ZYZ로 들어오므로 변환 지점이 여기 한 곳뿐이다.

**대문자 `"ZYZ"`가 중요하다.** scipy에서
- 대문자 = **intrinsic** (회전할 때마다 움직인 축을 다시 기준으로)
- 소문자 `"zyz"` = **extrinsic** (고정된 세계 축 기준)

둘은 다른 행렬을 낸다. 이 한 글자가 로봇을 다른 곳으로 보낸다.

**짐벌락**: ZYZ에서 두 번째 각 $\beta = 0$ 또는 $\pi$ 이면 첫 번째와 세 번째 축이
겹쳐서 자유도 하나를 잃는다. 그 자세 근처에서 각도가 튄다.
→ 이래서 내부 표현으로는 회전행렬·쿼터니언을 쓰고, 오일러각은 **입출력 경계에서만** 쓴다.
이 코드가 정확히 그렇게 되어 있다 (받자마자 `as_matrix()`로 바꾼다).

---

## 3. 핀홀 카메라 역투영 (픽셀 → 카메라 3D)

`src/pick_and_place_text/pick_and_place_text/detection.py:79-89`
(`object_detection/detection.py:80-90`에 같은 코드)

```python
def _pixel_to_camera_coords(self, x, y, z):
    fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
    ppx, ppy = self.intrinsics['ppx'], self.intrinsics['ppy']
    return ((x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z)
```

정투영은 내부 파라미터 행렬 $K$로 쓴다:

$$
K = \begin{bmatrix} f_x & 0 & p_{px} \\ 0 & f_y & p_{py} \\ 0 & 0 & 1 \end{bmatrix},
\qquad
z\begin{bmatrix} u \\ v \\ 1\end{bmatrix} = K \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
$$

코드는 이걸 **역으로** 푼 것이다: $P_{cam} = z\,K^{-1}\tilde u$

$$X = \frac{(u - p_{px})\,z}{f_x}, \qquad Y = \frac{(v - p_{py})\,z}{f_y}, \qquad Z = z$$

**핵심**: 픽셀 하나만으로는 3D 점을 못 정한다 — 광선(ray) 하나가 나올 뿐이다.
**깊이 $z$가 있어야 그 광선 위의 한 점으로 확정된다.** 그래서 RealSense가 필요하다.
이게 "단안 카메라로 왜 거리를 못 재나"의 답이다.

$K$는 어디서 오나 → ROS `CameraInfo` 메시지의 `k` 배열:

```python
self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4],
                   "ppx": msg.k[2], "ppy": msg.k[5]}   # realsense.py:23
```

`msg.k`는 $K$를 **행 우선(row-major)으로 편 9개 값**이다. 그래서 인덱스가 0,4,2,5다.

---

## 4. Hand-eye Calibration — $AX = XB$

`corecode/Calibration_Tutorial/eye2hand_calibration.py`

### 무엇을 구하나
그리퍼에 카메라가 붙어 있다(eye-in-hand). **그리퍼와 카메라 사이의 상대 자세 $X$는
로봇이 어떻게 움직이든 변하지 않는 상수**다. 그 상수를 구하는 게 hand-eye calibration이다.
이 $X$가 §0 그림의 `gripper2cam`이다.

> ⚠️ **단, 런타임이 쓰는 값은 이 스크립트가 뽑은 게 아니다.**
> `src/pick_and_place_text/resource/T_gripper2camera.npy`는 **교육자료로 제공된 파일**이고,
> 이 스크립트는 `T_cam2base.npy`라는 다른 이름으로 저장한다. 즉 절차는 학습용으로 돌려봤지만
> 실제 로봇이 쓰는 캘리브 값은 내가 뽑은 값이 아니다. §5·§6 참조.

### 그래서 그 상수가 실제로 뭔데 — `.npy`를 열어본 결과

`src/pick_and_place_text/resource/T_gripper2camera.npy` (= `corecode/Calibration_Tutorial/`의 것과 md5 동일)

```
[[  -1.      -0.0084   0.0017    33.0937]
 [   0.0084  -0.9997   0.0227    75.7056]
 [   0.0015   0.0227   0.9997  -233.5689]
 [   0.       0.       0.         1.    ]]      det(R) = 1.0000000000000009
```

**읽는 법:**

- 회전부 $R \approx \mathrm{diag}(-1, -1, +1)$ → **Z축 180° 회전**이다.
  $$R_z(\pi) = \begin{bmatrix} -1 & 0 & 0 \\ 0 & -1 & 0 \\ 0 & 0 & 1\end{bmatrix}$$
  물리적 의미: **카메라가 그리퍼에 180° 돌아간 방향으로 장착돼 있다.**
  나머지 $0.008 \sim 0.023$ 성분은 장착 오차 + 캘리브 잡음(≈1.3°).
- 평행이동 $t = (33.1,\ 75.7,\ -233.6)$ → 카메라 광학 중심이 그리퍼 원점에서
  x로 33mm, y로 76mm, z로 **−234mm** 떨어져 있다.
- $\det R = 1.0000000000000009$ → 부동소수점 오차 범위에서 정확히 1. **유효한 회전행렬이다.**

> **단위가 여기서 확정된다: mm.** 234라는 값이 미터면 234m라 물리적으로 불가능하고,
> mm면 그리퍼에서 카메라까지 23cm — 실제 장착 거리로 타당하다.

면접에서 "그 캘리브 파일 안에 뭐가 들었는지 아세요?"에
**"Z축 180도 회전과 (33, 76, −234)mm 오프셋입니다"** 라고 답할 수 있으면 끝난 거다.

### 왜 $AX = XB$ 꼴이 되나
두 자세 $i, i{+}1$ 사이의 **상대** 이동을 각각 로봇 쪽과 카메라 쪽에서 계산한다:

```python
A_i = inv(T_gripper2base[i]) @ T_gripper2base[i+1]     # 로봇이 얼마나 움직였나 (line 271)
B_i = inv(T_checker2cam[i])  @ T_checker2cam[i+1]      # 카메라가 보기에 얼마나 움직였나 (line 272)
```

같은 물리적 움직임을 두 좌표계에서 본 것이므로 $A_i X = X B_i$가 성립한다.
$X$ 하나에 대해 식이 여러 개 → 최소자승으로 푼다.

### 푸는 방법 (코드가 실제로 하는 것)

**1단계 — 회전 $\theta$**: 각 상대변환의 회전을 축·각 벡터로 로그 사상한다.

```python
def logR(T):                                            # line 147
    R = T[0:3, 0:3]
    theta = np.arccos((np.trace(R) - 1) / 2)            # 회전각
    logr = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) \
           * theta / (2 * np.sin(theta))                # 회전축 × 각
    return logr
```

$$\theta = \arccos\!\left(\frac{\mathrm{tr}(R) - 1}{2}\right)$$

**대각합이 회전각을 담고 있다**는 게 SO(3)의 핵심 성질이다.
$R$의 반대칭 부분 $\frac{R - R^\top}{2}$이 회전축 방향을 준다.
이게 로드리게스 공식의 역방향(로그 사상, $SO(3) \to \mathfrak{so}(3)$)이다.

그다음 $M = \sum \beta \alpha^\top$를 쌓고:

```python
theta = np.dot(sqrtm(inv(np.dot(M.T, M))), M.T)         # line 177
```

$$\hat R_X = (M^\top M)^{-1/2} M^\top$$

이건 **$M$에 가장 가까운 직교행렬을 찾는 직교 프로크루스테스(Procrustes) 해**다.
잡음 섞인 $M$은 정확한 회전행렬이 아니므로 $SO(3)$로 투영해야 한다.
(Park & Martin 1994 계열의 닫힌 해)

**2단계 — 평행이동 $b_x$**: 회전을 알면 평행이동은 **선형 최소자승**이 된다.

```python
C[3i:3i+3, :] = np.eye(3) - rot_a                       # line 185
d[3i:3i+3, 0] = trans_a - theta @ trans_b               # line 186
b_x = inv(C.T @ C) @ (C.T @ d)                          # line 188 — 정규방정식
```

$$(I - R_A)\,b_x = t_A - R_X t_B \quad\Rightarrow\quad b_x = (C^\top C)^{-1} C^\top d$$

`inv(C.T @ C) @ C.T`는 **무어–펜로즈 유사역행렬** $C^+$다.
식(3n개)이 미지수(3개)보다 많은 과결정계를 최소자승으로 푸는 표준형.

### 앞단 — solvePnP와 로드리게스

```python
retval, rvec, tvec = cv2.solvePnP(objp, corners_sub, camera_matrix, dist_coeffs)  # line 54
R, _ = cv2.Rodrigues(rvec)                                                        # line 59
```

- `solvePnP`: 3D 점(체커보드 격자, $z{=}0$ 평면) ↔ 2D 픽셀 대응에서 카메라 외부 파라미터를 푼다 (PnP 문제)
- `Rodrigues`: 회전벡터(축×각, 3개) ↔ 회전행렬(9개) 변환.
  $$R = I + \sin\theta\,[k]_\times + (1-\cos\theta)[k]_\times^2$$
  회전의 자유도가 **3**인데 행렬은 9개 값을 쓴다 — 그래서 6개의 구속조건($R^\top R = I$)이 붙는다.

---

## 5. 면접 예상 질문 — 내 코드 기준 답

| 질문 | 답의 뼈대 |
|---|---|
| 카메라 좌표를 로봇 베이스로 어떻게 옮기나요 | `base2cam = base2gripper @ gripper2cam`. gripper2cam은 캘리브 결과 상수, base2gripper는 매 순간 로봇이 주는 값 |
| 왜 4×4 행렬을 쓰나요 | 평행이동을 곱셈으로 만들어 변환을 합성 가능하게. 마지막 1이 그 역할 |
| 왜 ZYZ인가요 | 두산 컨트롤러가 ZYZ(도)로 자세를 준다. 내부는 즉시 행렬로 바꾼다 |
| 짐벌락 겪어봤나요 | ZYZ는 두 번째 각이 0/π일 때 축이 겹친다. 그래서 오일러각은 입출력 경계에서만 쓰고 내부는 행렬 |
| 픽셀에서 3D 좌표 어떻게 얻나요 | $K^{-1}$로 광선을 만들고 depth를 곱해 한 점으로 확정. depth 없으면 광선까지만 |
| 캘리브레이션 어떻게 했나요 | **정직하게**: 절차는 튜토리얼 코드로 이해했다(체커보드 여러 자세 → solvePnP로 cam→checker → 상대변환 쌍으로 $AX=XB$ → 회전은 Procrustes, 평행이동은 최소자승). 다만 **현재 로봇이 쓰는 `T_gripper2camera.npy`는 제공받은 값**이고 내가 직접 뽑은 게 아니다 |
| $\det R = 1$이 왜 중요한가요 | −1이면 반사가 섞여 좌표계 손잡이(handedness)가 뒤집힌다. 로봇이 거울상으로 움직인다 |

---

## 6. 이 코드를 읽으며 내가 발견한 것

> **면접에서 제일 강한 카드는 "내 코드의 문제를 내가 안다"는 것이다.**

### ① `det` 검사가 아무것도 안 한다 — 선형대수로 증명됨
```python
det_T = np.linalg.det(T_base2gripper)          # line 202
if np.abs(det_T) > 1e-6: valid_indices.append(i)
```
$T = \begin{bmatrix} R & t \\ 0 & 1\end{bmatrix}$ 이고 $R$은 회전행렬이므로
$\det T = \det R = 1$ — **항상**이다. 이 조건은 절대 실패하지 않는다.
"특이(singular) 자세를 걸러내려는" 의도였겠지만, 걸러야 할 것은
**변환행렬의 행렬식이 아니라 자세들이 서로 충분히 다른가**(회전축이 평행하지 않은가)다.
$AX=XB$는 회전축이 다른 자세 쌍이 최소 2개 필요하다.

### ② `square_size`가 한 곳에서 무시된다
```python
objp[:, :2] = np.mgrid[...].T.reshape(-1,2) * 25          # line 30 — 하드코딩
objp[:, :2] = np.mgrid[...].T.reshape(-1,2) * square_size # line 78 — 파라미터
```
지금은 `square_size = 25`라 결과가 같지만, 체커보드를 바꾸면
`find_checkerboard_pose`만 25mm로 남는다. **스케일이 어긋나면 평행이동이 통째로 틀린다.**

### ③ 캘리브 스크립트의 출력은 실제로 쓰이지 않는다 (제일 중요)

```python
T_cam2base = X
np.save("T_cam2base.npy", T_cam2base)          # line 283
```

런타임 코드는 `T_gripper2camera.npy`를 로드한다 (`robot_control.py:132`).
**이 파일은 교육자료로 제공된 것이고, 위 스크립트가 만든 게 아니다.**

즉 `eye2hand_calibration.py`는 **절차를 배우는 튜토리얼**이고,
로봇이 실제로 쓰는 캘리브 값은 내가 뽑지 않았다.

> **면접에서 이걸 뭉개면 안 된다.** "캘리브레이션 했습니다"와
> "캘리브레이션 원리를 코드로 따라가 봤고 값은 제공받았습니다"는 다른 말이다.
> 뒤엣것을 정확히 말하는 쪽이 오히려 신뢰를 얻는다 —
> 그리고 곧바로 "직접 뽑으려면 이 스크립트의 ①②를 먼저 고쳐야 합니다"로 이어갈 수 있다.

**곁다리로, 그 스크립트의 변수 이름도 틀렸다.** eye-in-hand에서 base↔cam은 상수가 아니다
(카메라가 그리퍼와 같이 움직이므로). $AX=XB$가 내놓는 상수 $X$는 그리퍼↔카메라 변환이니
`T_cam2base`가 아니라 `T_gripper2cam`이어야 한다.
→ 좌표 변환 코드에서 이름은 주석이 아니라 **명세**다. `A2B` 규칙을 안 지키면 곱하는 순서를 틀린다.

### ④ 단위 — mm로 확정 (§4에서 확인)
`square_size` mm, 두산 로봇 좌표 mm, `T_gripper2camera.npy`의 평행이동도 mm
(z = −233.6 → 미터일 수 없다). RealSense depth(16UC1)도 mm.
**변환 사슬 전체가 mm로 일관된다.** 단, depth 토픽의 실제 인코딩은 실기로 확인할 것.
→ 실기로 확인하면 [[ws/cobot2/context/constraints|constraints]]에 적을 것.

### ⑤ 죽은 줄
```python
robot_poses[:, :3] = robot_poses[:, :3]        # line 196 — 자기 자신 대입
```
단위 변환(mm↔m)을 하려다 만 흔적으로 보인다.

---

## 7. 이 저장소엔 **없는** 것 (→ 축 2, 교재로 공부)

면접에서 "안 해봤다"고 정직하게 말하고 이론으로 답할 것들:

- **역기구학(IK) · 야코비안** — 두산 M0609는 `movej`/`movel`이 컨트롤러 펌웨어 안에서 푼다.
  내 파이썬 코드에 야코비안 계산이 한 줄도 없다.
- **특이점 회피** — `set_singularity_handling()` API를 호출만 한다. 내부 구현은 안 본다.
- **제어 이론** (임피던스/어드미턴스), **SLAM**, **칼만 필터** — 이 저장소에 없음.

이건 `docs/협동로봇2_강의자료.pdf`, `docs/협동2_AI_강의자료.pdf`와 ROS 2 / DRL 매뉴얼로
NotebookLM에서 채운다. **축 1과 섞어서 "해봤다"고 말하지 말 것.**

---

## 관련
- [[ws/cobot2/context/constraints|실기로 확인한 제약]]
- 원본 코드: `src/robot_control/robot_control/robot_control.py`,
  `src/pick_and_place_text/pick_and_place_text/detection.py`,
  `corecode/Calibration_Tutorial/eye2hand_calibration.py`
