import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{time}]: {message}'

  return LaunchDescription([
    Node(
      package='pika_spark_bno085_driver',
      executable='pika_spark_bno085_driver_node',
      name='pika_spark_bno085_driver',
      namespace='',
      output='screen',
      emulate_tty=True,
      parameters=[
      ]
    )
  ])
