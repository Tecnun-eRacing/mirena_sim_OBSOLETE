import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import shutil

def generate_launch_description():
    # Get the path to the ROS 2 package
    package_path = get_package_share_directory('mirena_sim')  # Replace with your package name
    godot_project_path = os.path.join(package_path, 'MirenaSim/MirenaSim')  # Relative path to sim executable
    return LaunchDescription([
        ExecuteProcess(
            cmd=[godot_project_path],  # Command to run Godot with the project path
            output='screen'  # Display output in the terminal
        ),
    ])
