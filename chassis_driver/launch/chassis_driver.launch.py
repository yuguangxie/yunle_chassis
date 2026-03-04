from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Create launch description that starts chassis_driver_node with network and driver parameters."""
    pkg_share = get_package_share_directory('chassis_driver')
    network_config = os.path.join(pkg_share, 'config', 'network.yaml')
    driver_config = os.path.join(pkg_share, 'config', 'driver.yaml')

    return LaunchDescription([
        Node(
            package='chassis_driver',
            executable='chassis_driver_node',
            name='chassis_driver_node',
            output='screen',
            parameters=[network_config, driver_config],
        )
    ])
