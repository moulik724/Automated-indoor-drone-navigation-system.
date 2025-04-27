from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('drone_sim'),
        'urdf',
        'drone.urdf.xacro'
    ])

    return LaunchDescription([
        # Launch robot_state_publisher with the drone URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'),
                    ' ',
                    urdf_file
                ])
            }]
        ),

        # Spawn the drone into Ignition Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_ign_gazebo', 'create',
                '-name', 'simple_drone',
                '-x', '0', '-y', '0', '-z', '0.5',
                '-topic', 'robot_description'
            ],
            output='screen'
        ),

        # Launch your drone control node
        Node(
            package='drone_sim',
            executable='simple_control.py',
            name='simple_control_node',
            output='screen'
        )
    ])

