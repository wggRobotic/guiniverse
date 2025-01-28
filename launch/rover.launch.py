import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='guiniverse',
            executable='main',
            name='guiniverse',
        ),
        Node(
            package='n10_cam_dif',
            executable='cam_dif',
            name='cam_dif',
            remappings=[
                ('/n10/intel/color', 'n10/rear/color'),
            ]
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='n10_barcode_reader',
            remappings=[
                ('/image', 'n10/rear/color'),
            ]
        )
    ])
if __name__ == '__main__':
    generate_launch_description()