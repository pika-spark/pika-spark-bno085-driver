/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/pika-spark-bno085-driver/graphs/contributors.
 */

#ifndef PIKA_SPARK_NCN26010_DRIVER_SPI_H
#define PIKA_SPARK_NCN26010_DRIVER_SPI_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdint>

#include <linux/spi/spidev.h>

#include <string>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SPI
{
public:
  SPI(std::string const & dev_name, uint8_t const mode, uint8_t const bits, uint32_t const speed_Hz);
  ~SPI();

  bool transfer(uint8_t const * tx_buf, uint8_t * rx_buf, size_t const num_bytes);

private:
  int _fd;
  uint8_t const _bits;
  uint32_t const _speed_Hz;
};

#endif /* PIKA_SPARK_NCN26010_DRIVER_SPI_H */
