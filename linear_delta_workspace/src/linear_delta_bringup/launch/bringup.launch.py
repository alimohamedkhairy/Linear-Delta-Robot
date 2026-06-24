import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Get the launch directories of your packages
    # (Replace 'my_package_1' and 'my_package_2' with your actual package names)
    control_package_path = get_package_share_directory('linear_delta_control')
    description_path = get_package_share_directory('linear_delta_description')

    launch_file_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_path, 'launch', 'description.launch.py')
        )
    )

    launch_file_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_package_path, 'launch', 'control.launch.py')
        )
        # launch_arguments={'arg_name': 'value'}.items() # Optional arguments
    )

    delayed_launch_file_2 = TimerAction(
        period=10.0,
        actions=[launch_file_2]
    )

    # 4. Create and return the LaunchDescription containing both
    return LaunchDescription([
        launch_file_1,
        delayed_launch_file_2
    ])