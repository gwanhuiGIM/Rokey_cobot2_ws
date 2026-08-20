# depth_downsample_cpp

`cobot_rg2/rg2/m0609_rg2_bringup/scripts/depth_downsample_node.py`를 C++(`rclcpp` +
`cv_bridge` + OpenCV)로 옮겨보는 **학습용** 패키지다. 원본 Python 파일은 수정하지 않았으며,
입력과 출력 토픽을 같게 설정하면 두 노드를 동시에 실행할 경우 중복 publisher가 생길 수 있다.
학습할 때는 하나만 실행한다.

## 검증 상태

실기 카메라로 실행하거나 실제 depth pixel 값을 비교한 적은 없다. 이번 범위의 검증은 package
빌드와 executable 등록 확인까지다.

| 확인함 | 확인 안 함 |
|---|---|
| `colcon build --symlink-install --packages-select depth_downsample_cpp` | RealSense 카메라가 연결된 상태에서 실제 publish/subscribe |
| `ros2 pkg executables depth_downsample_cpp`에서 executable 등록 | Python 결과와 frame-by-frame pixel 및 CameraInfo 수치 비교 |
| 설치된 Humble `cv_bridge` C++ 헤더의 `toCvCopy()`/`CvImage::toImageMsg()` API | Python `cv2`보다 얼마나 빨라지는지에 대한 벤치마크 |

따라서 C++가 Python보다 빠를 수 있다는 말은 **추정**일 뿐이다. 입력 해상도, copy 횟수,
CPU/GPU 사용, downstream node를 포함한 end-to-end 조건을 고정해 측정하기 전에는 성능 수치를
단정하지 않는다.

## 실행

> ⚠️ 아래 명령은 이번 범위에서 실행하지 않았다. 이 호스트에는 현재 `ros2 run` subcommand가
> 없어 executable 등록까지만 확인했으며, 실제 카메라 연결 실행은 미검증이다.

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run depth_downsample_cpp depth_downsample_node --ros-args \
  -p target_width:=424 -p target_height:=240
```

launch 파일은 이번 학습 포팅 범위에 포함하지 않았다. 실기 카메라 실행은 미검증이며, 이 패키지는
실기 배포 목적이 아니다.

## 토픽과 파라미터

Python 원본과 이름 및 기본값이 동일하다. 모든 subscription/publisher는 Python의
`qos_profile_sensor_data`에 대응하는 `rclcpp::SensorDataQoS`를 사용한다.

| 구분 | 이름 | 기본값 |
|---|---|---|
| 입력 Image | `in_image_topic` | `/camera/camera/aligned_depth_to_color/image_raw` |
| 입력 CameraInfo | `in_info_topic` | `/camera/camera/aligned_depth_to_color/camera_info` |
| 출력 Image | `out_image_topic` | `/cumotion/depth_1/image` |
| 출력 CameraInfo | `out_info_topic` | `/cumotion/depth_1/camera_info` |
| 목표 가로 | `target_width` | `424` |
| 목표 세로 | `target_height` | `240` |

## 원본과 같은 처리 계약

1. CameraInfo callback은 최신 메시지만 저장한다.
2. Image callback은 아직 CameraInfo가 없으면 depth를 변환하거나 발행하지 않고 return한다.
3. depth를 같은 encoding의 `cv::Mat`으로 변환하고, 이미 목표 해상도면 `cv::resize`를 생략한다.
4. 크기가 다를 때는 반드시 `cv::INTER_NEAREST`로 리사이즈한다. 선형 보간은 depth edge의
   foreground/background 값을 섞어 존재하지 않는 깊이를 만들 수 있으므로 쓰지 않는다.
5. 출력 Image는 입력의 `header`와 `encoding`을 보존한다.
6. `K`와 `P`의 `fx`, `fy`, `cx`, `cy`만 `sx=target_width/source_width`, `sy=target_height/source_height`로
   스케일한다. `D`(distortion coefficients)와 `R`은 원본 값을 그대로 복사한다.

## rclpy + NumPy + cv2 → rclcpp + cv_bridge + OpenCV C++

Python 원본은 `CvBridge.imgmsg_to_cv2(..., desired_encoding='passthrough')`가 반환한 NumPy
배열에서 `depth.shape[:2]`를 읽고, `np.array(info.k).reshape(3, 3)` 및 `flatten()`으로
CameraInfo 배열을 다룬다.

C++에서는 `cv_bridge::toCvCopy(msg, "")`의 빈 encoding이 입력 encoding을 유지한다. 그 결과의
`cv::Mat`은 `rows`/`cols`로 높이·너비를 읽는다. `CameraInfo::k`와 `p`는 이미 고정 길이의
row-major C++ 배열이므로 NumPy의 `reshape()`가 필요 없다. 예를 들어 `K(1, 1)`은 3x3 row-major
배열의 `k[4]`, `P(1, 2)`는 3x4 row-major 배열의 `p[6]`이다. 구현은 이 index를 주석으로 명시해
행렬 모양을 잃지 않게 했다.

`cv_bridge::CvImage(header, encoding, mat).toImageMsg()`는 Python의
`cv2_to_imgmsg(resized, encoding=msg.encoding)`에 대응한다. C++ API는 `cv::Mat`과 ROS Image
사이의 copy 및 소유권을 명시적으로 보여주는 반면, Python API는 bridge 객체의 메서드 호출로
감춘다는 차이가 있다.
