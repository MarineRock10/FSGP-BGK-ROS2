"""
Launch file for the FSGP-BGK traversability system.

Launches three components:
1. C++ node (fsgp_bgk_node) - preprocessing, global map, publishing
2. Python service (traversability_service) - core SGP algorithm
3. (Optional) Simulation environment
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    cpp_pkg_dir = get_package_share_directory('fsgp_bgk_cpp')

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(cpp_pkg_dir, 'config', 'params.yaml'),
        description='Path to the C++ node parameter file'
    )

    algorithm_config_arg = DeclareLaunchArgument(
        'algorithm_config',
        default_value='../config/params.yaml',
        description='Path to the Python algorithm parameter file (relative to fsgp_bgk/python/)'
    )

    # C++ infrastructure node
    cpp_node = Node(
        package='fsgp_bgk_cpp',
        executable='fsgp_bgk_node',
        name='fsgp_bgk_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    # Python traversability service
    # Note: This runs the Python script directly. The script must be on the PATH
    # or launched from the correct working directory.
    python_service = Node(
        package='fsgp_bgk_cpp',  # Uses the same package for launch convenience
        executable='traversability_service.py',
        name='traversability_service',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('algorithm_config')
        }]
    )

    return LaunchDescription([
        config_file_arg,
        algorithm_config_arg,
        cpp_node,
        python_service,
    ])
