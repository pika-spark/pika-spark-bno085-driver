/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/pika-spark-bno085-driver/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "bno085.h"

#include <cstring>

/**************************************************************************************
 * PRIVATE FUNCTION DECLARATIONS
 **************************************************************************************/

static int      priv_sh2_hal_open     (sh2_Hal_t * self);
static void     priv_sh2_hal_close    (sh2_Hal_t * self);
static int      priv_sh2_hal_read     (sh2_Hal_t * self, uint8_t * pBuffer, unsigned len, uint32_t * t_us);
static int      priv_sh2_hal_write    (sh2_Hal_t * self, uint8_t * pBuffer, unsigned len);
static uint32_t priv_sh2_hal_getTimeUs(sh2_Hal_t * self);

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

BNO085::BNO085(std::shared_ptr<SPI> const spi)
: _spi{spi}
, _start{std::chrono::steady_clock::now()}
{
  _sh2_hal.user_reference = reinterpret_cast<void *>(this);

  init();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

int BNO085::begin()
{
  if (auto const rc = open(); rc != SH2_OK)
    return rc;

  return SH2_OK;
}

int BNO085::readProductIds(sh2_ProductIds_t * prod_ids)
{
  memset(prod_ids, 0, sizeof(sh2_ProductIds_t));
  return sh2_getProdIds(prod_ids);
}

int BNO085::sh2_hal_read(uint8_t * pBuffer, unsigned len, uint32_t * /* t_us */)
{
  /* Read the sensor hub header. */
  uint8_t       sh_hdr_rx_buf[4] = {0};
  uint8_t const sh_hdr_tx_buf[4] = {0};

  if (!_spi->transfer(sh_hdr_tx_buf, sh_hdr_rx_buf, sizeof(sh_hdr_tx_buf)))
    return 0;

  /* Determine packet size (mask out continuous bit). */
  uint16_t const packet_size =
    (static_cast<uint16_t>(sh_hdr_rx_buf[0]) | static_cast<uint16_t>(sh_hdr_rx_buf[1]) << 8) & 0x7FFF;

  /* Return early if packet size exceeds provided buffer. */
  if (packet_size > len)
    return 0;

  /* Ensure that we only transmit zero's. */
  memset(pBuffer, 0, packet_size);

  /* Transmit zero's and read from the device. */
  if (!_spi->transfer(pBuffer, pBuffer, packet_size))
    return 0;

  return packet_size;
}

int BNO085::sh2_hal_write(uint8_t * pBuffer, unsigned len)
{
  if (!_spi->transfer(pBuffer, pBuffer, len))
    return 0;

  return len;
}

uint32_t BNO085::sh2_hal_getTimeUs()
{
  auto const now = std::chrono::steady_clock::now();
  auto const uptime = (now - _start);
  return std::chrono::duration_cast<std::chrono::microseconds>(uptime).count();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void BNO085::init()
{
  _sh2_hal.open      = priv_sh2_hal_open;
  _sh2_hal.close     = priv_sh2_hal_close;
  _sh2_hal.read      = priv_sh2_hal_read;
  _sh2_hal.write     = priv_sh2_hal_write;
  _sh2_hal.getTimeUs = priv_sh2_hal_getTimeUs;
}

int BNO085::open()
{
  return sh2_open(&_sh2_hal, nullptr, nullptr);
}

/**************************************************************************************
 * PRIVATE FUNCTION DEFINITIONS
 **************************************************************************************/

int priv_sh2_hal_open(sh2_Hal_t * /* self */)
{
  return 0;
}

void priv_sh2_hal_close(sh2_Hal_t * /* self */)
{
  /* Do nothing. */
}

int priv_sh2_hal_read(sh2_Hal_t * self, uint8_t * pBuffer, unsigned len, uint32_t * t_us)
{
  BNO085 * this_ptr = reinterpret_cast<BNO085 *>(self->user_reference);
  return this_ptr->sh2_hal_read(pBuffer, len, t_us);
}

int priv_sh2_hal_write(sh2_Hal_t * self, uint8_t * pBuffer, unsigned len)
{
  BNO085 * this_ptr = reinterpret_cast<BNO085 *>(self->user_reference);
  return this_ptr->sh2_hal_write(pBuffer, len);
}

uint32_t priv_sh2_hal_getTimeUs(sh2_Hal_t * self)
{
  BNO085 * this_ptr = reinterpret_cast<BNO085 *>(self->user_reference);
  return this_ptr->sh2_hal_getTimeUs();
}
