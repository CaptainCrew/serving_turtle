from setuptools import find_packages, setup
from glob import glob  # glob 모듈을 import
package_name = 'serving_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + 'turtlebot3_msgs', glob('msg/*.msg')),  # ✅ 메시지 추가
        ('share/' + package_name + '/launch', glob('launch/**/*.launch.py')),
        ('share/' + package_name + '/map', glob('map/*')),
    ],
    install_requires=['setuptools','rclpy', 'turtle_bot3_msgs'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['initial_pose_publisher = serving_turtle.initial_pose_publisher:main',
        'waypoint_navigator = serving_turtle.waypoint_navigator:main',
        'table_main = serving_turtle.gui.table.main:main',
        'kitchen_main = serving_turtle.gui.kitchen.main:main',],
    },
)
