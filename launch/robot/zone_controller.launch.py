from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file = os.path.join(
        get_package_share_directory('serving_turtle'),
        'maps',
        'map.yaml'  # ✅ 맵 파일 경로 추가
    )

    return LaunchDescription([
        # ✅ Nav2 Bringup 실행 시 map 인자 추가
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true'
            }.items()
        ),

        # ✅ 마스크 발행 노드 실행
        Node(
            package='serving_turtle',
            executable='cost_layer',
            name='zone_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
