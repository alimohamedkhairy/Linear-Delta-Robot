import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    ld = LaunchDescription()

    control_package_path = get_package_share_directory("linear_delta_control")
    config_path = os.path.join(control_package_path, "config", "real_linear_delta_controller_manager.yaml")
    
    description_path = get_package_share_directory("linear_delta_description")
    robot_description_path = os.path.join(description_path, 'urdf', "linear_delta.urdf.xacro")
    robot_description_urdf = {"robot_description": ParameterValue(Command(['xacro ', robot_description_path, ' is_sim:=false']), value_type=str)}

    real_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="real",
        parameters=[robot_description_urdf, {"frame_prefix": "real/"}],
        remappings=[
            ("/tf", "/real/tf"),
            ("/tf_static", "/real/tf_static")
        ]
    )

    real_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='real', 
        parameters=[config_path, {"use_sim_time": False}]
    )

    real_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/real/controller_manager"],
    )

    real_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/real/controller_manager"]
    )

    motions_node = Node(
        package="linear_delta_control",
        executable="motions"
    )

    ld.add_action(real_state_publisher_node)
    ld.add_action(real_controller_manager)
    ld.add_action(real_joint_state_broadcaster_spawner)
    ld.add_action(real_joint_trajectory_controller_spawner)
    ld.add_action(motions_node)
    return ld