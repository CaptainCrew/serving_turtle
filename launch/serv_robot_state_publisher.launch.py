from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"publish_frequency": 50.0},
                    {"initial_pose_x": 0.0},
                    {"initial_pose_y": 0.0},
                    {"initial_pose_theta": 0.0},
                ],
                remappings=[
                    ("/tf", "/tf"),
                    ("/tf_static", "/tf_static"),
                ],
            )
        ]
    )
