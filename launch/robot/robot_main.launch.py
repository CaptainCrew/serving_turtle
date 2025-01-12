from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ✅ 패키지 경로 가져오기
    package_dir = get_package_share_directory('serving_turtle')
    launch_dir = os.path.join(package_dir, 'launch')

    # ✅ 경로 출력 (디버깅)
    print(f"🔍 package_dir 경로: {package_dir}")
    print(f"🔍 launch_dir 경로: {launch_dir}")

    # 🏞️ Gazebo 실행
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo.launch.py')
        )
    )

    # 📍 Localization 실행
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization.launch.py')
        )
    )


    return LaunchDescription([
        gazebo_launch,           # 1️⃣ Gazebo 즉시 실행
        localization_launch     # 2️⃣ Localization 즉시 실행
    ])
