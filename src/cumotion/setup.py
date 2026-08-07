import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cumotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일이 get_package_share_directory('cumotion')/config 에서 yaml 을 찾는다.
        # 둘 중 하나라도 빠지면 launch 가 FileNotFound 로 죽는다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joon',
    maintainer_email='joon5514@naver.com',
    description='MoveIt(+cuMotion+nvblox) 재계획 루프로 두산 M0609 를 제어한다',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ⚠️ 로봇이 실제로 움직이는 노드다. 항상 mode:=check 를 먼저 돌린다.
            'dynamic_avoid = cumotion.dynamic_avoid:main',
        ],
    },
)
