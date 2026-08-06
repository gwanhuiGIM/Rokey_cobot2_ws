"""task_manager + robot_safety_node 를 띄운다. 로봇·카메라·MoveIt 은 **일부러 포함하지 않았다.**

이 ws 는 런치를 단위로 쪼개 쓴다 (bringup / camera / moveit). 여기서 다시 include 하면
같은 노드가 두 번 뜨거나(robot_description 충돌) 이미 켜둔 걸 못 쓰게 된다.
기동 순서는 README "실행" 절이 단일 출처다.

`robot_safety_node`(안전정지·backdrive 서비스, 로봇상태 발행)는 **여기 포함시켰다.** UI(rqt
패널)는 있으나 없으나 자동중단(충돌 감지 시 task_manager 가 하던 작업을 abort)은 항상 동작해야
하기 때문이다 — rqt 패널은 이 런치와 별개로 필요할 때만 띄운다(README "UI" 절):
    rqt --standalone pick_fsm

    ros2 launch pick_fsm pick_fsm.launch.py                    # 계획만 (기본, 안전)
    ros2 launch pick_fsm pick_fsm.launch.py dry_run:=false     # 실행. 승인은 여전히 필요
    ros2 launch pick_fsm pick_fsm.launch.py voice:=false target:=apple
    ros2 launch pick_fsm pick_fsm.launch.py planning_pipeline:=isaac_ros_cumotion
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('pick_fsm')
    default_params = os.path.join(pkg, 'config', 'pick_fsm.yaml')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='파라미터 yaml. 값의 정본은 이 파일이다'),
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='true=계획만(plan_only). 실기 실행은 false'),
        DeclareLaunchArgument('require_approval', default_value='true',
                              description='false 로 두면 사람 승인 없이 실행한다'),
        DeclareLaunchArgument('voice', default_value='true',
                              description='false 면 get_keyword 를 건너뛰고 target 인자를 쓴다'),
        DeclareLaunchArgument('target', default_value='',
                              description='voice:=false 일 때의 고정 타겟'),
        DeclareLaunchArgument('grasp_source', default_value='compute_grasp',
                              description='compute_grasp | legacy_trigger | manual'),
        DeclareLaunchArgument('planning_pipeline', default_value='ompl',
                              description='ompl | isaac_ros_cumotion — move_group 에 그 파이프라인이 '
                                          '떠 있어야 한다(moveit.launch.py 쪽 설정)'),
        DeclareLaunchArgument('gripper_backend', default_value='real',
                              description='real | virtual. 숫자 명령의 단위가 다르다'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('robot_ns', default_value='dsr01',
                              description='robot_safety_node 가 Doosan 서비스를 찾을 네임스페이스'),
    ]

    # yaml 을 먼저 깔고 런치 인자를 그 위에 덮는다 (뒤에 오는 dict 가 이긴다).
    # ⚠️ LaunchConfiguration 은 **문자열**이다. 노드가 bool 로 선언한 파라미터에
    #    "true" 라는 문자열을 넣으면 타입 불일치로 노드가 뜨자마자 죽는다.
    #    ParameterValue(value_type=bool) 로 변환을 명시해야 한다.
    def as_bool(name):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)

    task_manager = Node(
        package='pick_fsm',
        executable='task_manager',
        name='task_manager',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'dry_run': as_bool('dry_run'),
                'require_approval': as_bool('require_approval'),
                'voice_enabled': as_bool('voice'),
                'target': ParameterValue(LaunchConfiguration('target'), value_type=str),
                'grasp_source': ParameterValue(LaunchConfiguration('grasp_source'),
                                               value_type=str),
                'planning_pipeline': ParameterValue(LaunchConfiguration('planning_pipeline'),
                                                    value_type=str),
                'gripper_backend': ParameterValue(LaunchConfiguration('gripper_backend'),
                                                  value_type=str),
            },
        ],
    )

    safety = Node(
        package='pick_fsm',
        executable='robot_safety_node',
        name='robot_safety_node',
        output='screen',
        parameters=[{'robot_ns': ParameterValue(LaunchConfiguration('robot_ns'),
                                                 value_type=str)}],
    )

    return LaunchDescription(args + [task_manager, safety])
