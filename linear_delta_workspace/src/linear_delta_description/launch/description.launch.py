import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

def generate_launch_description():
    
    ld = LaunchDescription()

    description_path = get_package_share_directory("linear_delta_description")
    robot_description_path = os.path.join(description_path, 'urdf', "linear_delta.urdf.xacro")
    robot_description_urdf = ParameterValue(Command(['xacro ', robot_description_path, ' is_sim:=true']), value_type=str)

    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description_urdf, 'use_sim_time': True}],
        remappings=[
            ("/robot_description", "/sim/robot_description"), 
            ("/joint_states", "/sim/joint_states")]
    )

    rviz_config_path = os.path.join(description_path, 'rviz', "config.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    gazebo_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', "gz_sim.launch.py")

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    gazebo_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-name', 'linear', 
                   '-topic', '/sim/robot_description'],
        parameters=[{'use_sim_time': True}]
    )

    description_parent_path = os.path.dirname(description_path)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=description_parent_path
    )

    bridge_node = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/world/default/model/linear/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model", "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    remappings=[("/world/default/model/linear/joint_state", "/sim/joint_states")])

    sim_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/sim/controller_manager"]
    )

    sim_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/sim/controller_manager"]
    )

    ld.add_action(set_gz_resource_path)
    ld.add_action(state_publisher_node)
    ld.add_action(gazebo_sim)
    ld.add_action(gazebo_spawn)
    ld.add_action(bridge_node)
    ld.add_action(TimerAction(
        period=5.0,
        actions=[
            sim_joint_state_broadcaster_spawner,
            sim_joint_trajectory_controller_spawner
        ]
    ))
    ld.add_action(TimerAction(period=3.0, actions=[rviz_node]))
    
    return ld