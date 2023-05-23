/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/pika-spark-bno085-driver/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "spi.h"

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <stdexcept>

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

SPI::SPI(std::string const & dev_name, uint8_t const mode, uint8_t const bits, uint32_t const speed_Hz)
: _bits{bits}
, _speed_Hz{speed_Hz}
{
  _fd = open(dev_name.c_str(), O_RDWR);
  if (_fd < 0)
    throw std::runtime_error("could not open device \"" + dev_name + "\"");

  if (ioctl(_fd, SPI_IOC_WR_MODE, &mode) < 0)
    throw std::runtime_error("could not set SPI mode" + std::string(strerror(errno)));

  if (ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
    throw std::runtime_error("could not set SPI bits" + std::string(strerror(errno)));

  if (ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_Hz) < 0)
    throw std::runtime_error("could not set SPI bit rate" + std::string(strerror(errno)));
}

SPI::~SPI()
{
  close(_fd);
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool SPI::transfer(uint8_t const * tx_buf, uint8_t * rx_buf, size_t const num_bytes)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  struct spi_ioc_transfer tf =
  {
    /* .tx_buf = */(unsigned long)tx_buf,
    /* .rx_buf = */(unsigned long)rx_buf,
    /* .len = */static_cast<uint32_t >(num_bytes),
    /* .speed_hz = */_speed_Hz,
    /* .delay_usecs = */0,
    /* .bits_per_word = */_bits,
  };
#pragma GCC diagnostic pop

  if (ioctl(_fd, SPI_IOC_MESSAGE(1), &tf) < 0)
    return false;

  return true;
}
