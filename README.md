<a href="https://pika-spark.io/"><img align="right" src="https://raw.githubusercontent.com/pika-spark/.github/main/logo/logo-pika-spark-bg-white.png" width="15%"></a>
:sparkles: `pika-spark-bno085-driver`
=====================================
[![Build Status](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/ros2.yml/badge.svg)](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/spell-check.yml/badge.svg)](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/spell-check.yml)

Linux user space ROS driver for the [BNO085](https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf) 9-DoF IMU.

**Note**: In order to run the BNO085 ROS driver on [Pika Spark](https://pika-spark.io/) take a look at ready-to-use build/run scripts at [pika-spark-container/ros-imu-bno085](https://github.com/pika-spark/pika-spark-containers/tree/main/ros-imu-bno085).

<p align="center">
  <a href="https://pika-spark.io/"><img src="https://raw.githubusercontent.com/pika-spark/.github/main/logo/logo-pika-spark-bg-white-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
cd $COLCON_WS/src
git clone --recursive https://github.com/pika-spark/pika-spark-bno085-driver
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select pika_spark_bno085_driver
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch pika_spark_bno085_driver imu.py
```

#### How-to-visualize your IMU data
```bash
sudo apt-get install ros-humble-imu-tools
ros2 launch pika_spark_bno085_driver rviz2.py
```
