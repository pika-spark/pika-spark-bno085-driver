/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/pika-spark-bno085-driver/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "bno085_hal.h"

#include <cstring>

#include <iostream>

#include "bno085_exception.h"

/**************************************************************************************
 * PRIVATE FUNCTION DECLARATIONS
 **************************************************************************************/

static int      priv_sh2_hal_open     (sh2_Hal_t * self);
static void     priv_sh2_hal_close    (sh2_Hal_t * self);
static int      priv_sh2_hal_read     (sh2_Hal_t * self, uint8_t * pBuffer, unsigned len, uint32_t * t_us);
static int      priv_sh2_hal_write    (sh2_Hal_t * self, uint8_t * pBuffer, unsigned len);
static uint32_t priv_sh2_hal_getTimeUs(sh2_Hal_t * self);

static void     priv_sh2_hal_callback   (void * cookie, sh2_AsyncEvent_t * event);
static void     priv_sh2_sensor_callback(void * cookie, sh2_SensorEvent_t * event);

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

BNO085_HAL::BNO085_HAL(std::shared_ptr<SPI> const spi,
                       std::shared_ptr<SysGPIO> const nirq)
: _spi{spi}
, _nirq{nirq}
, _start{std::chrono::steady_clock::now()}
{
  _sh2_hal.user_reference = reinterpret_cast<void *>(this);

  _sh2_hal.open      = priv_sh2_hal_open;
  _sh2_hal.close     = priv_sh2_hal_close;
  _sh2_hal.read      = priv_sh2_hal_read;
  _sh2_hal.write     = priv_sh2_hal_write;
  _sh2_hal.getTimeUs = priv_sh2_hal_getTimeUs;

  sh2_open(&_sh2_hal, priv_sh2_hal_callback, reinterpret_cast<void *>(this));
  sh2_setSensorCallback(priv_sh2_sensor_callback, reinterpret_cast<void *>(this));
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

int BNO085_HAL::sh2_hal_read(uint8_t * pBuffer, unsigned len, uint32_t * t_us)
{
  /* Read the sensor hub header. */
  uint8_t       sh_hdr_rx_buf[4] = {0};
  uint8_t const sh_hdr_tx_buf[4] = {0};

  if (!waitForIrqLow())
    return 0;

  if (!_spi->transfer(sh_hdr_tx_buf, sh_hdr_rx_buf, sizeof(sh_hdr_tx_buf)))
    return 0;

  /* Determine packet size (mask out continuous bit). */
  uint16_t const packet_size =
    (static_cast<uint16_t>(sh_hdr_rx_buf[0]) | static_cast<uint16_t>(sh_hdr_rx_buf[1]) << 8) & 0x7FFF;

  /* Return early if packet size exceeds provided buffer. */
  if (packet_size > len)
    return 0;

  /* Ensure that we only transmit zero's. */
  auto const tx_buf = std::make_unique<uint8_t[]>(packet_size);
  memset(tx_buf.get(), 0, packet_size);

  if (!waitForIrqLow())
    return 0;

  /* Transmit zero's and read from the device. */
  if (!_spi->transfer(tx_buf.get(), pBuffer, packet_size))
    return 0;

  *t_us = sh2_hal_getTimeUs();

  return packet_size;
}

int BNO085_HAL::sh2_hal_write(uint8_t * pBuffer, unsigned len)
{
  if (!waitForIrqLow())
    return 0;

  auto const tx_buf = std::make_unique<uint8_t[]>(len);
  memcpy(tx_buf.get(), pBuffer, len);

  if (!_spi->transfer(tx_buf.get(), pBuffer, len))
    return 0;

  return len;
}

uint32_t BNO085_HAL::sh2_hal_getTimeUs()
{
  auto const now = std::chrono::steady_clock::now();
  auto const uptime = (now - _start);
  return std::chrono::duration_cast<std::chrono::microseconds>(uptime).count();
}

void BNO085_HAL::sh2_hal_callback(sh2_AsyncEvent_t * event)
{
  auto const SHTPEventToErrStr =[](sh2_ShtpEvent_t const evt)
  {
    switch(evt)
    {
      case SH2_SHTP_TX_DISCARD:
        return std::string_view("SH2_SHTP_TX_DISCARD");
        break;
      case SH2_SHTP_SHORT_FRAGMENT:
        return std::string_view("SH2_SHTP_SHORT_FRAGMENT");
        break;
      case SH2_SHTP_TOO_LARGE_PAYLOADS:
        return std::string_view("SH2_SHTP_TOO_LARGE_PAYLOADS");
        break;
      case SH2_SHTP_BAD_RX_CHAN:
        return std::string_view("SH2_SHTP_BAD_RX_CHAN");
        break;
      case SH2_SHTP_BAD_TX_CHAN:
        return std::string_view("SH2_SHTP_BAD_TX_CHAN");
        break;
      case SH2_SHTP_BAD_FRAGMENT:
        return std::string_view("SH2_SHTP_BAD_FRAGMENT");
        break;
      case SH2_SHTP_BAD_SN:
        return std::string_view("SH2_SHTP_BAD_SN");
        break;
      case SH2_SHTP_INTERRUPTED_PAYLOAD:
        return std::string_view("SH2_SHTP_INTERRUPTED_PAYLOAD");
        break;
      default:
        __builtin_unreachable();
        break;
    }
  };

  if (event->eventId == SH2_RESET) {
    std::cout << "BNO085_HAL::sh2_hal_callback(...) reset has occurred" << std::endl;
  }
  else if (event->eventId == SH2_SHTP_EVENT) {
    if (event->shtpEvent != SH2_SHTP_BAD_SN)
      std::cerr << "BNO085_HAL::sh2_hal_callback(...) SHTP error has occurred: \"" << SHTPEventToErrStr(event->shtpEvent) << "\"" << std::endl;
  }
  else if (event->eventId == SH2_GET_FEATURE_RESP) {
    std::cerr << "BNO085_HAL::sh2_hal_callback(...) unhandled get feature response received" << std::endl;
  }
  else {
    throw BNO085_Exception("BNO085_HAL::sh2_hal_callback(...) unhandled hal event occurred, eventId = %d", event->eventId);
  }
}

void BNO085_HAL::sh2_sensor_callback(sh2_SensorEvent_t * event)
{
  internal_sh2_sensor_callback(event);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool BNO085_HAL::waitForIrqLow(std::chrono::milliseconds const timeout)
{
  auto const isIrqLow = [this]()
  {
    unsigned int nirq_value = 1;

    if (auto const rc = _nirq->gpio_get_value(nirq_value); rc != 0)
      throw BNO085_Exception("BNO085_HAL::waitForIrqLow(...) nirq->gpio_get_value(...) failed with error code %d", rc);

    if (nirq_value == 0)
      return true;
    else
      return false;
  };

  if (isIrqLow())
    return true;

  auto const gpio_nirq_fd = _nirq->gpio_fd_open();
  auto const rc_poll = _nirq->gpio_poll(gpio_nirq_fd, timeout.count());
  _nirq->gpio_fd_close(gpio_nirq_fd);

  if      (rc_poll < 0)
    throw BNO085_Exception("BNO085_HAL::waitForIrqLow(...) poll(gpio_nirq) failed with error code %d", rc_poll);
  else if (rc_poll == 0) /* Timeout, but we still check nIRQ one more time. */
    return isIrqLow();
  else /* A IO event occurred, let's check nIRQ- */
    return isIrqLow();
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
  BNO085_HAL * this_ptr = reinterpret_cast<BNO085_HAL *>(self->user_reference);
  return this_ptr->sh2_hal_read(pBuffer, len, t_us);
}

int priv_sh2_hal_write(sh2_Hal_t * self, uint8_t * pBuffer, unsigned len)
{
  BNO085_HAL * this_ptr = reinterpret_cast<BNO085_HAL *>(self->user_reference);
  return this_ptr->sh2_hal_write(pBuffer, len);
}

uint32_t priv_sh2_hal_getTimeUs(sh2_Hal_t * self)
{
  BNO085_HAL * this_ptr = reinterpret_cast<BNO085_HAL *>(self->user_reference);
  return this_ptr->sh2_hal_getTimeUs();
}

void priv_sh2_hal_callback (void * cookie, sh2_AsyncEvent_t * event)
{
  BNO085_HAL * this_ptr = reinterpret_cast<BNO085_HAL *>(cookie);
  this_ptr->sh2_hal_callback(event);
}

void priv_sh2_sensor_callback(void * cookie, sh2_SensorEvent_t * event)
{
  BNO085_HAL * this_ptr = reinterpret_cast<BNO085_HAL *>(cookie);
  this_ptr->sh2_sensor_callback(event);
}
