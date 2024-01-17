<a href="https://pika-spark.io/"><img align="right" src="https://raw.githubusercontent.com/pika-spark/.github/main/logo/logo-pika-spark-bg-white.png" width="15%"></a>
:sparkles: `pika-spark-bno085-driver`
=====================================
[![Build Status](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/ros2.yml/badge.svg)](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/spell-check.yml/badge.svg)](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/spell-check.yml)

Linux user space ROS driver for the [BNO085](https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf) 9-DoF IMU.

<p align="center">
  <a href="https://pika-spark.io/"><img src="https://raw.githubusercontent.com/pika-spark/.github/main/logo/logo-pika-spark-bg-white-github.png" width="40%"></a>
</p>

### How-to-build
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
