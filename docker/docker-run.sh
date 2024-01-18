#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ "$(id -u)" != "0" ]; then
  echo "This script must be run as root."
  exit 1
fi

# SYSFS_GPIO_NUMBER = ((GPIO_PORT - 1) * 32) + GPIO_PIN
# BNO085_nIRQ	= MX8MM_IOMUXC_SPDIF_TX_GPIO5_IO3
# BNO085_nRST =	MX8MM_IOMUXC_SAI5_RXD1_GPIO3_IO22
# BNO085_nBOOT = MX8MM_IOMUXC_SAI5_RXD2_GPIO3_IO23
GPIO_NIRQ_NUM=131
GPIO_NRST_NUM=86
GPIO_NBOOT_NUM=87

function finish {
  echo $GPIO_NIRQ_NUM > /sys/class/gpio/unexport
  echo $GPIO_NRST_NUM > /sys/class/gpio/unexport
  echo $GPIO_NBOOT_NUM > /sys/class/gpio/unexport
}
trap finish EXIT

echo $GPIO_NIRQ_NUM > /sys/class/gpio/export
echo $GPIO_NRST_NUM > /sys/class/gpio/export
echo $GPIO_NBOOT_NUM > /sys/class/gpio/export

modprobe spidev
chmod ugo+rw /dev/spidev0.0

sudo -u fio docker run -it --ulimit nofile=1024:1024 --rm -u 0 --privileged --device /dev/spidev0.0 -v /sys/class/gpio:/sys/class/gpio --network host pika_spark_bno085_driver
