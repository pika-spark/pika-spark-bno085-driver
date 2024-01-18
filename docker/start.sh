#!/bin/sh
cd /tmp/colcon_ws
. /opt/ros/humble/setup.sh
. install/setup.sh
echo "Starting pika-spark-bno085-driver ..."
ros2 launch pika_spark_bno085_driver imu.py
