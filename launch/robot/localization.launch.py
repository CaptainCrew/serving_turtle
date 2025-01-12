import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # TurtleBot3 패키지 이름 설정
    TURTLEBOT3_MODEL = os.environ.get("TURTLEBOT3_MODEL", "waffle_pi")

    # 맵 파일 경로
    map_path = os.path.join(
        get_package_share_directory("serving_turtle"), "map", "map.yaml"
    )

    # AMCL 매개변수 파일 경로
    param_file_path = os.path.join(
        "/home/rokey/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2",
        "param",
        f"{TURTLEBOT3_MODEL}.yaml",
    )

    # RViz 설정 파일 경로
    rviz_config_file = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "rviz",
        "nav2_default_view.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_path}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[param_file_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server", "amcl"],
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": use_sim_time}],
            ),           
            Node(
            package='serving_turtle',
            executable='initial_pose_publisher',
            name='initial_pose_publisher',
            output='screen',
        )


        ]
    )
