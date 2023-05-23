<a href="https://pika-spark.io/"><img align="right" src="https://raw.githubusercontent.com/pika-spark/.github/main/logo/logo-pika-spark-bg-white.png" width="15%"></a>
:sparkles: `pika-spark-bno085-driver`
=====================================
[![Smoke test status](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/smoke-test.yml/badge.svg)](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/smoke-test.yml)
[![Spell Check status](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/spell-check.yml/badge.svg)](https://github.com/pika-spark/pika-spark-bno085-driver/actions/workflows/spell-check.yml)

Linux user space driver for the [BNO085](https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf) 9-DoF IMU driver.

<p align="center">
  <a href="https://pika-spark.io/"><img src="https://raw.githubusercontent.com/pika-spark/.github/main/logo/logo-pika-spark-bg-white-github.png" width="40%"></a>
</p>

### How-to-build
* Enable [spidev](https://www.kernel.org/doc/Documentation/spi/spidev) to access SPI from userspace
```bash
modprobe spidev
sudo chmod ugo+rw /dev/spidev1.0
```
* Build user space driver within Docker container with access to SPI
```bash
docker pull alpine:latest
docker run -it -u 0 --device /dev/spidev1.0 alpine:latest sh
apk add git g++ make cmake linux-headers
cd /tmp
git clone https://github.com/107-systems/pika-spark-bno085-driver && cd pika-spark-bno085-driver
mkdir build && cd build
cmake ..
make
```
