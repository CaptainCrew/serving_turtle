import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # 고정된 맵 경로: src/serving_turtle/map
    fixed_map_path = os.path.join(
        get_package_share_directory("serving_turtle"), "map", "map.yaml"  # 맵 파일 이름
    )



    param_file_name = TURTLEBOT3_MODEL + ".yaml"
    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("turtlebot3_navigation2"),
            "param",
            param_file_name,
        ),
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz"
    )


    print(f"🔍 param_dir 경로: {param_dir}")
    print(f"🔍 nav2_launch_file_dir 경로: {nav2_launch_file_dir}")
    print(f"🔍 rviz_config_dir 경로: {rviz_config_dir}")




    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_launch_file_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": fixed_map_path,  # 고정된 맵 경로를 전달
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_dir],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
