import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 런치 디스크립션 생성
    ld = LaunchDescription(
        #[
        #DeclareLaunchArgument(
        #    'table_num',
        #    default_value='2',  # 기본값은 문자열로 설정
        #    description='Number of tables (1-9)'),
        #]
    )

    img_tracker_pub = Node(
            package='a4_week4',
            executable='img_tracker_pub',
            output='screen',
        )
    
    img_tracker_sub = Node(
            package='a4_week4',
            executable='img_tracker_sub',
            output='screen',
        )
    
    ld.add_action(img_tracker_pub)
    ld.add_action(img_tracker_sub)
    return ld