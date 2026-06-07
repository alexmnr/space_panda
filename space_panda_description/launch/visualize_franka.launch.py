import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    franka_xacro_file = os.path.join(get_package_share_directory('space_panda_description'), 'urdf', 'panda.urdf.xacro')
    robot_description = Command([FindExecutable(name='xacro'), ' ', franka_xacro_file, ' ', 'tf_prefix:=', 'panda_'])

    rviz_file = os.path.join(get_package_share_directory('space_panda_description'), 'rviz', 'visualize_franka.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])
    ])
