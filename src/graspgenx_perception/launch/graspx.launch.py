"""graspgenx_perception + graspx 를 한 번에 띄운다.

    ros2 launch graspgenx_perception graspx.launch.py

카메라(`realsense2_camera`)와 로봇 bringup 은 **여기서 띄우지 않는다.** 로봇 bringup 은
실기 모션이라 사람이 직접 실행해야 하고, 카메라는 다른 파이프라인과 공유하기 때문이다.
이 launch 는 그 위에 얹는 인식·파지 계산 층만 담당한다.

전제:
  1. 호스트에서 `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true`
  2. 호스트에서 로봇 bringup (base_link <- camera_color_optical_frame TF 가 필요하다)
  3. 컨테이너/호스트 양쪽에 `FASTRTPS_DEFAULT_PROFILES_FILE` (README 참고)

seg_source:
  geometric — 작업공간 박스 + connectedComponents. 신경망 0개. **기본값**
  yolo      — yolo_seg_node 의 `/yolo_seg/labels` 를 그대로 쓴다
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'seg_source': ('geometric', "'geometric' 또는 'yolo'"),
    'run_yolo': ('true', 'yolo_seg_node 를 띄울지 (seg_source=yolo 면 반드시 true)'),
    'run_bridge': ('true', 'grasp_bridge_node 를 띄울지'),
    'image_topic': ('/camera/camera/color/image_raw', 'yolo_seg 가 구독할 컬러 토픽'),
    'publish_overlay': ('true', '오버레이(JPEG) 발행 — rqt 로 보려면 true'),
    'device': ('0', "'0'=첫 GPU, 'cpu'"),
    'conf': ('0.25', 'YOLO 신뢰도 임계'),
    'min_pixels': ('300', '이보다 작은 덩어리는 물체로 안 본다'),
    'out_dir': ('', '비우면 임시 디렉토리. 씬 4파일을 남기려면 경로를 준다'),
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
