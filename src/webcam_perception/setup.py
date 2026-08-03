from setuptools import find_packages, setup

package_name = 'webcam_perception'

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
    description='웹캠(C270) 기반 CPU 전용 인식 실험 노드 모음',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sam_mask_node = webcam_perception.sam_mask_node:main',
        ],
    },
)
