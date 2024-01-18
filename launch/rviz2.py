import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('pika_spark_bno085_driver'))

    # Create a RVIZ2 node
    node_rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_path, 'config', 'view_imu.rviz')]]
    )

    # Launch!
    return LaunchDescription([
        node_rviz2
    ])
