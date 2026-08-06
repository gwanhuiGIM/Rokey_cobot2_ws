from setuptools import find_packages, setup

package_name = 'yolo_seg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
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
