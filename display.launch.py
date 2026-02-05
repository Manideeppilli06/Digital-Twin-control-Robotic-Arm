from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('my_arm_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_arm.urdf')
    rviz_path = os.path.join(pkg_path, 'rviz', 'your_config.rviz')

    return LaunchDescription([

        # ✅ Use GUI version (THIS IS THE KEY FIX)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path).read()
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_path]
        )
    ])
