from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the keyboard SCU control helper node."""
    return LaunchDescription([
        Node(
            package='chassis_driver',
            executable='keyboard_scu_control_node',
            name='keyboard_scu_control_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'topic_name': '/yunle_chassis/control/scu_control_command',
                'publish_rate_hz': 20.0,
                'speed_step_kmh': 0.5,
                'default_speed_kmh': 1.0,
                'max_speed_kmh': 15.0,
                'steering_step_deg': 2.0,
                'max_steering_angle_deg': 27.0,
                'rear_steering_ratio': 0.0,
                'auto_publish_zero_on_exit': True,
            }],
        )
    ])
