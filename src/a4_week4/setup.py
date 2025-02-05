from setuptools import find_packages, setup
import os, glob

package_name = 'a4_week4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'weights'), glob.glob('weights/*')),
        (os.path.join('share', package_name, 'trackers'), glob.glob('trackers/*')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhj',
    maintainer_email='happyijun@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_pub = a4_week4.yolo_publisher:main',
            'img_sub = a4_week4.yolo_subscriber:main',
            'img_tracker_pub = a4_week4.yolo_tracker_publisher:main',
            'img_tracker_sub = a4_week4.yolo_tracker_subscriber:main',
            'box_info_pub = a4_week4.box_measure_size:main',
            'init_pose = a4_week4.init_pose:main',
            'send_goal_stop = a4_week4.send_goal_stop:main',
            'send_waypoint = a4_week4.send_waypoint:main',
        ],
    },
)
