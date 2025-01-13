from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serving_turtle',       # 패키지 이름
            executable='cost_layer',   # 실행할 노드(스크립트) 이름
            name='zone_controller_node',    # 노드 이름
            output='screen',                # 출력 화면에 표시
            parameters=[{
                'use_sim_time': True        # 시뮬레이션 시간 사용 여부 (필요시)
            }]
        )
    ])
