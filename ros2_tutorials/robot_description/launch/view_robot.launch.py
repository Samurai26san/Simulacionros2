"""
    Author: carlos
"""
from ament_index_python.packages import get_package_share_path

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    urdf_tutorial_path = get_package_share_path('robot_description')
    default_model_path = urdf_tutorial_path / 'urdf/tareaenclases.urdf'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/urdf.rviz'

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='true = joint_state_publisher_gui  |  false = pub_joints.py')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file')

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    # ── Nodos ────────────────────────────────────────────────────────────

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Slider manual — solo cuando gui:=true
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Tu nodo con matriz H — solo cuando gui:=false
    pub_joints_node = Node(
        package='visual_pubsub',
        executable='pub_joints',
        name='joint_state_publisher',
        output='screen',                  # muestra los logs en terminal
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,   # activo si gui:=true
        pub_joints_node,                  # activo si gui:=false
        rviz_node,
    ])