import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Path to /config, which is used to set the Nodes' parameters
    config = os.path.join(get_package_share_directory('multisim'),
                         'config',
                         'turtle_params.yaml'
                         )
    return LaunchDescription([
        Node(
            package='multisim',
            executable='turtle',
            parameters=[config],
            name='turtle1',
            )
    ])
