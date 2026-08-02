"""RealSense 드라이버 + 캘리브 static TF (base_link → camera_link).

로봇과 분리된 단위다. 순서:
  1) ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100   # 로봇만
  2) ros2 launch m0609_rg2_bringup camera.launch.py                                    # 카메라 + TF
  3) ros2 launch dsr_moveit_config_m0609 demo.launch.py                                # MoveIt

TF 값을 여기 하드코딩하지 않는다. config/T_cam2base.npy를 읽어 매 launch마다 계산한다.
재캘리브 후엔 npy만 갈아끼운다 (symlink-install이면 rebuild 불필요):
  cp corecode/Calibration_Tutorial/T_cam2base.npy \
     src/cobot_rg2/rg2/m0609_rg2_bringup/config/T_cam2base.npy

eye-to-hand 전제다 — 카메라가 로봇에 붙어 있지 않으므로 URDF가 아니라 static TF로 준다.
eye-in-hand(그리퍼 부착)로 바꾸면 이 launch를 쓰면 안 된다. camera_link의 부모가
URDF와 여기 둘로 갈려 TF 트리가 깨진다. → bringup_camera.launch.py 참고.
"""
import os
import sys

import numpy as np
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PARENT_FRAME = 'base_link'
CHILD_FRAME = 'camera_link'


def generate_launch_description():
    pkg_share = get_package_share_directory('m0609_rg2_bringup')
    sys.path.insert(0, os.path.join(pkg_share, 'scripts'))
    from calib_npy_to_tf import npy_to_tf_args  # noqa: E402

    # 해상도 기본값을 낮게 잡는다. 드라이버 기본(848x480x30)은 이 랩탑에서 안 돌아간다:
    # i7-10510U 15W / GPU 없음 / ros2_control_node가 상시 204% 인데
    # 848*480*30 = 12.2 M point/s 다. 424x240x15면 1/8 (약 1.5 M point/s).
    # 실측 근거는 md/context/constraints.md "octomap_server — 이 랩탑 리소스로는 ...".
    #
    # color도 같이 낮춘다: align_depth.enable=true면 depth를 **color 해상도로 리샘플**하므로
    # color만 크면 낮춘 의미가 없다.
    #
    # [튜닝] GPU PC나 여유 있는 머신에서는 인자로 올린다:
    #   ros2 launch ... camera.launch.py depth_profile:=848x480x30 color_profile:=848x480x30
    args = [
        DeclareLaunchArgument('driver', default_value='true',
                              description='RealSense 드라이버 spawn 여부 (false면 TF만)'),
        DeclareLaunchArgument('depth_profile', default_value='424x240x15',
                              description='depth 스트림 WxHxFPS. 올리면 move_group CPU가 같이 오른다'),
        DeclareLaunchArgument('color_profile', default_value='424x240x15',
                              description='color 스트림 WxHxFPS. align_depth가 이 해상도를 따라간다'),
    ]

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'enable_sync': True,
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
        }],
        condition=IfCondition(LaunchConfiguration('driver')),
        output='screen',
    )

    # npy가 없으면 TF 노드만 빠지고 카메라는 뜬다 (캘리브 전에도 영상은 봐야 하니까).
    calib_npy = os.path.join(pkg_share, 'config', 'T_cam2base.npy')
    calib_tf = []
    if os.path.exists(calib_npy):
        t, q = npy_to_tf_args(np.load(calib_npy), PARENT_FRAME, CHILD_FRAME)
        calib_tf = [Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_calib_tf',
            output='log',
            arguments=[f'{v:.6f}' for v in t] + [f'{v:.8f}' for v in q]
                      + [PARENT_FRAME, CHILD_FRAME],
        )]
    else:
        print(f'[camera.launch] ⚠️ {calib_npy} 없음 — '
              f'{PARENT_FRAME}→{CHILD_FRAME} TF를 발행하지 않는다 (포인트클라우드가 로봇과 안 붙는다)')

    return LaunchDescription(args + [realsense_node] + calib_tf)
