from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ✅ 테이블 ID를 런치 인자로 추가
    table_id_arg = DeclareLaunchArgument(
        'table_id',  # 런치 인자 이름
        default_value='1',  # 기본값
        description='테이블 ID'
    )

    return LaunchDescription([
        table_id_arg,
        Node(
            package='serving_turtle',
            executable='table_main',
            name='table_main_node',
            output='screen',
            parameters=[{'table_id': LaunchConfiguration('table_id')}]  # ✅ 테이블 ID 전달
        )
    ])
