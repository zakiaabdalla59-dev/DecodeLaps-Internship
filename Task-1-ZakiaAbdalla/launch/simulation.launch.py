import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_kinematics_planner')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_robot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Process URDF/Xacro file
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # RViz2 Node with custom view_robot.rviz config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Kinematics Planner Node (provides steady /joint_states stream)
    kinematics_planner_node = Node(
        package='arm_kinematics_planner',
        executable='kinematics_planner_node',
        name='kinematics_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nodes_to_launch = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        robot_state_publisher_node,
        rviz_node,
        kinematics_planner_node
    ]

    has_gz = shutil.which('gz') is not None
    has_gazebo = shutil.which('gazebo') is not None

    if has_gz:
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        )
        spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'robot_arm'],
            output='screen'
        )
        nodes_to_launch.extend([gz_sim, spawn_entity_node])
    elif has_gazebo:
        gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        )
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'robot_arm'],
            output='screen'
        )
        nodes_to_launch.extend([gazebo, spawn_entity_node])

    return LaunchDescription(nodes_to_launch)
