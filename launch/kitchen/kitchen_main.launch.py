from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serving_turtle',       # ✅ 패키지 이름
            executable='kitchen_main',         # ✅ setup.py에 등록한 실행 파일 이름
            name='kitchen_main_node',          # ✅ 노드 이름
            output='screen'                 # ✅ 실행 결과를 터미널에 출력
        )
    ])
