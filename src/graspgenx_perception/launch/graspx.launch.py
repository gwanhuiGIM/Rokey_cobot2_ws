"""graspgenx_perception 의 인식·파지 계산 층을 띄운다.

**`run_yolo` 와 `run_bridge` 를 한 머신에서 동시에 true 로 두면 안 된다.**
`yolo_seg_node` 는 ultralytics 때문에 **컨테이너 전용**이고, `grasp_bridge_node` 는
GraspGenX 워커를 `uv` 로 띄우는데 **컨테이너에 uv 가 없다**. 그래서 반씩 나눠 띄운다:

  # A) 기하 세그 — 호스트 한 대로 끝난다 (기본값, 지금 유일하게 검증된 경로)
  ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false

  # B) YOLO 세그 — 컨테이너와 호스트를 각각 띄운다
  #   컨테이너: FASTRTPS_DEFAULT_PROFILES_FILE 필수 (없으면 프레임 0장)
  ros2 launch graspgenx_perception graspx.launch.py run_bridge:=false classes:='[46,47]'
  #   호스트:
  ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false seg_source:=yolo

카메라(`realsense2_camera`)와 로봇 bringup 은 **여기서 띄우지 않는다.** 로봇 bringup 은
실기 모션이라 사람이 직접 실행해야 하고, 카메라는 다른 파이프라인과 공유하기 때문이다.

전제:
  1. 호스트에서 `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true`
  2. 호스트에서 로봇 bringup (base_link <- camera_color_optical_frame TF 가 필요하다)
  3. 양쪽 `ROS_DOMAIN_ID=93` (컨테이너는 이미 박혀 있고, 호스트에서 export 를 빠뜨린다)
  4. **컨테이너**에 `FASTRTPS_DEFAULT_PROFILES_FILE` (호스트 쪽은 없어도 됐다 — README 실측)

seg_source:
  geometric — 작업공간 박스 + connectedComponents. 신경망 0개. **기본값**
  yolo      — yolo_seg_node 의 `/yolo_seg/labels` 를 그대로 쓴다
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'seg_source': ('geometric', "'geometric' 또는 'yolo'"),
    'run_yolo': ('true', 'yolo_seg_node 를 띄울지 (seg_source=yolo 면 반드시 true)'),
    'run_bridge': ('true', 'grasp_bridge_node 를 띄울지'),
    'image_topic': ('/camera/camera/color/image_raw', 'yolo_seg 가 구독할 컬러 토픽'),
    'publish_overlay': ('true', '오버레이(JPEG) 발행 — rqt 로 보려면 true'),
    'device': ('0', "'0'=첫 GPU, 'cpu'"),
    'conf': ('0.25', 'YOLO 신뢰도 임계'),
    # COCO 80종의 **인덱스** 목록. 비우면 전체. banana=46, apple=47, cup=41, bottle=39,
    # bowl=45, orange=49, scissors=76 (2026-08-07 yolo11n-seg.pt 의 model.names 로 확인).
    # 이 가중치엔 공구 5종이 없다 — 없는 물체는 어떤 인덱스로도 못 잡는다.
    'classes': ('[]', "COCO 클래스 인덱스 필터. 예: classes:='[46,47]' (banana, apple)"),
    'min_pixels': ('300', '이보다 작은 덩어리는 물체로 안 본다'),
    'out_dir': ('', '씬 4파일 저장 위치. 비우면 <repo>/data/graspgenx_scene '
                    '(2026-08-07부터 항상 영구 저장 — 임시 디렉토리 아님)'),
}


def generate_launch_description():
    cfg = {k: LaunchConfiguration(k) for k in ARGS}

    yolo = Node(
        package='graspgenx_perception', executable='yolo_seg_node', name='yolo_seg_node',
        output='screen',
        condition=IfCondition(cfg['run_yolo']),
        parameters=[{
            'image_topic': cfg['image_topic'],
            'publish_overlay': cfg['publish_overlay'],
            'device': cfg['device'],
            'conf': cfg['conf'],
            # 런치 인자는 문자열이라 그냥 넘기면 노드에 STRING 으로 도착하고
            # `[int(c) for c in classes]` 가 문자 단위로 돌다 ValueError 로 죽는다.
            # ParameterValue(value_type=List[int]) 가 YAML 로 파싱해 정수 배열로 만든다
            # (2026-08-07 `LaunchContext` 로 '[]'/'[46,47]' 둘 다 직접 평가해 확인).
            'classes': ParameterValue(cfg['classes'], value_type=List[int]),
            'min_pixels': cfg['min_pixels'],
        }],
    )

    # 이 노드는 로봇을 움직이지 않는다 — grasp 포즈를 계산해 발행할 뿐이다.
    # 실행 파일 이름에 `.py` 가 없다: 소스가 패키지 안으로 들어오면서 console_scripts
    # 진입점이 됐다 (2026-08-07, setup.py 주석 참고).
    bridge = Node(
        package='graspgenx_perception', executable='grasp_bridge_node', name='grasp_bridge_node',
        output='screen',
        condition=IfCondition(cfg['run_bridge']),
        parameters=[{
            'seg_source': cfg['seg_source'],
            'min_pixels': cfg['min_pixels'],
            'out_dir': cfg['out_dir'],
        }],
    )

    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
        + [yolo, bridge]
    )
