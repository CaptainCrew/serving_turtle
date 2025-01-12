from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serving_turtle',       # ✅ 패키지 이름
            executable='table_main',        # ✅ setup.py에 등록할 실행 파일 이름
            name='table_main_node',         # ✅ 노드 이름
            output='screen'                 # ✅ 터미널에 출력
        )
    ])
