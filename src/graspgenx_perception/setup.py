import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'yolo_seg'

# graspx 배선 스크립트는 워크스페이스 `scripts/` 에 그대로 두고 여기서 실행 파일로만 심는다.
# 파일을 옮기면 scripts/test_*.py 의 import 가 깨지고 git 이력이 끊긴다 — 경로는 yolo_seg 로
# 통일하되(ros2 run yolo_seg <script>) 소스 위치는 건드리지 않는 절충이다.
# colcon 은 build/yolo_seg/ 를 cwd 로 setup.py 를 돌린다. setup.py 자신의 위치에서 거슬러
# 올라가야 cwd 에 무관하게 맞는다.
_WS = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
_GRASPX_SCRIPTS = [
    os.path.join(_WS, 'scripts', f)
    for f in ('grasp_bridge_node.py', 'capture_graspgenx_scene.py')
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    scripts=[p for p in _GRASPX_SCRIPTS if os.path.isfile(p)],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimkh',
    maintainer_email='wook9980@gmail.com',
    description='YOLO 인스턴스 세그멘테이션 노드 (GPU·도커 컨테이너 전용)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_seg_node = yolo_seg.yolo_seg_node:main',
        ],
    },
)
