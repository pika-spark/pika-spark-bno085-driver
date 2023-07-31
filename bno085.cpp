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

BNO085::BNO085(std::shared_ptr<SPI> const spi,
               std::shared_ptr<SysGPIO> const nirq,
               StabilizedRotationVectorWAccuracyCallbackFunc const sensor_func)
: _spi{spi}
, _nirq{nirq}
, _sensor_func{sensor_func}
, _start{std::chrono::steady_clock::now()}
{
  _sh2_hal.user_reference = reinterpret_cast<void *>(this);

  _sh2_hal.open      = priv_sh2_hal_open;
  _sh2_hal.close     = priv_sh2_hal_close;
  _sh2_hal.read      = priv_sh2_hal_read;
  _sh2_hal.write     = priv_sh2_hal_write;
  _sh2_hal.getTimeUs = priv_sh2_hal_getTimeUs;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

int BNO085::begin()
{
  if (auto const rc = sh2_open(&_sh2_hal, priv_sh2_hal_callback, reinterpret_cast<void *>(this)); rc != SH2_OK)
    return rc;

  if (auto const rc = sh2_setSensorCallback(priv_sh2_sensor_callback, reinterpret_cast<void *>(this)); rc != SH2_OK)
    return rc;

  return SH2_OK;
}

int BNO085::readProductIds(sh2_ProductIds_t * prod_ids)
{
  memset(prod_ids, 0, sizeof(sh2_ProductIds_t));
  return sh2_getProdIds(prod_ids);
}

int BNO085::config()
{
  sh2_SensorConfig_t const bno085_config =
    {
      /* .changeSensitivityEnabled  = */ false,
      /* .changeSensitivityRelative = */ false,
      /* .wakeupEnabled             = */ false,
      /* .alwaysOnEnabled           = */ false,
      /* .sniffEnabled              = */ false,
      /* .changeSensitivity         = */ 0,
      /* .reportInterval_us         = */ 5000, /* 200 Hz */
      /* .batchInterval_us          = */ 0,
      /* .sensorSpecific            = */ 0
    };

  return sh2_setSensorConfig(SH2_ARVR_STABILIZED_RV, &bno085_config);
}

void BNO085::spinOnce()
{
  sh2_service();
}

int BNO085::sh2_hal_read(uint8_t * pBuffer, unsigned len, uint32_t * /* t_us */)
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
  memset(pBuffer, 0, packet_size);

  if (!waitForIrqLow())
    return 0;

  /* Transmit zero's and read from the device. */
  if (!_spi->transfer(pBuffer, pBuffer, packet_size))
    return 0;

  return packet_size;
}

int BNO085::sh2_hal_write(uint8_t * pBuffer, unsigned len)
{
  if (!waitForIrqLow())
    return 0;

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

void BNO085::sh2_hal_callback(sh2_AsyncEvent_t * event)
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

  if (event->eventId == SH2_RESET)
    std::cerr << "BNO085::sh2_hal_callback(...) reset has occurred" << std::endl;
  else if (event->eventId == SH2_SHTP_EVENT)
    std::cerr << "BNO085::sh2_hal_callback(...) SHTP error has occurred: \"" << SHTPEventToErrStr(event->shtpEvent) << "\"" << std::endl;
  else if (event->eventId == SH2_GET_FEATURE_RESP)
    std::cerr << "BNO085::sh2_hal_callback(...) get feature response received" << std::endl;
  else
    throw BNO085_Exception("BNO085::sh2_hal_callback(...) unhandled hal event occurred, eventId = %d", event->eventId);
}

void BNO085::sh2_sensor_callback(sh2_SensorEvent_t * event)
{
  sh2_SensorValue_t sensor_value;

  if (auto const rc = sh2_decodeSensorEvent(&sensor_value, event); rc != SH2_OK)
    throw BNO085_Exception("error decoding sensor event, rc = %d", rc);

  if (sensor_value.sensorId == SH2_ARVR_STABILIZED_RV) {
    if (_sensor_func)
      _sensor_func(sensor_value.un.arvrStabilizedRV);
  }
  else
    throw BNO085_Exception("unhandled sensor event decoded, sensorId = %d", sensor_value.sensorId);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool BNO085::waitForIrqLow(std::chrono::milliseconds const timeout)
{
  for (auto const start = std::chrono::steady_clock::now();
       std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start) < timeout;
       )
  {
    unsigned int nirq_value = 1;

    if (auto const rc = _nirq->gpio_get_value(nirq_value); rc != 0)
      throw BNO085_Exception("BNO085::waitForIrqLow(...) nirq->gpio_get_value(...) failed with error code %d", rc);

    if (nirq_value == 0)
      return true;
  }

  /* A timeout has occured, nIRQ has not been set to LOW within
   * "timeout" milliseconds since the first call to this function.
   */
  return false;
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

void priv_sh2_hal_callback (void * cookie, sh2_AsyncEvent_t * event)
{
  BNO085 * this_ptr = reinterpret_cast<BNO085 *>(cookie);
  this_ptr->sh2_hal_callback(event);
}

void priv_sh2_sensor_callback(void * cookie, sh2_SensorEvent_t * event)
{
  BNO085 * this_ptr = reinterpret_cast<BNO085 *>(cookie);
  this_ptr->sh2_sensor_callback(event);
}
