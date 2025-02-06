import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 런치 디스크립션 생성
    ld = LaunchDescription()

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='searching',
        description='Navigation mode: searching, wait, or tracking'
    )

    # init_pose 실행 (자동으로 초기 pose 설정)
    init_pose = Node(
        package='a4_week4',
        executable='init_pose',
        output='screen',
        parameters=[{'auto_init': True}]  # init_pose 실행 시 자동으로 동작하도록 매개변수 전달
    )

    # set_nav_mode 실행 (mode argument 전달)
    set_nav_mode = Node(
        package='a4_week4',
        executable='set_nav_mode',
        output='screen',
        parameters=[{'mode': LaunchConfiguration('mode')}]
    )

    ld.add_action(mode_arg)
    ld.add_action(init_pose)
    ld.add_action(set_nav_mode)

    return ld