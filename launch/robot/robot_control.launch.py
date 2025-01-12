from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serving_turtle',  # ✅ 패키지 이름 (수정 필요)
            executable='robot_main',  # ✅ 실행 파일 이름 (setup.py와 일치)
            name='waypoint_navigator',
            output='screen'
        )
    ])
