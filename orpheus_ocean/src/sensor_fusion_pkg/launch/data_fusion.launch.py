from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    data_fusion_node = Node(
        package='sensor_fusion_pkg',
        executable='data_fusion',
        name='data_fusion_node',
        output="screen",
        emulate_tty=True,

    )

    ld.add_action(data_fusion_node)
    return ld