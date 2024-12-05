import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import PushROSNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the ROS 2 package
    package_path = get_package_share_directory(
        'mirena_sim')  # Replace with your package name
    # Relative path to Godot project
    godot_project_path = os.path.join(package_path, 'MirenaSim')

    return LaunchDescription([
        ExecuteProcess(
            # Command to run Godot with the project path
            cmd=['godot', '--path', godot_project_path],
            output='screen',  # Display output in the terminal
        ),
    ])
