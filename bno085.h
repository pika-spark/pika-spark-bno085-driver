/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/pika-spark-bno085-driver/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <chrono>
#include <memory>

#include <sh2.h>
#include <sh2_err.h>

#include "spi.h"
#include "gpio-sysfs.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class BNO085
{
public:
  BNO085(std::shared_ptr<SPI> const spi,
         std::shared_ptr<SysGPIO> const nirq);


  int begin();


  int readProductIds(sh2_ProductIds_t * prod_ids);


  /* Do not publicly use those functions.
   * They are used for the sensor hub HAL.
   */
  int      sh2_hal_read     (uint8_t * pBuffer, unsigned len, uint32_t * t_us);
  int      sh2_hal_write    (uint8_t * pBuffer, unsigned len);
  uint32_t sh2_hal_getTimeUs();

private:
  std::shared_ptr<SPI> const _spi;
  std::shared_ptr<SysGPIO> const _nirq;
  std::chrono::steady_clock::time_point const _start;
  sh2_Hal_t _sh2_hal;

  void init();
  int  open();
  bool waitForIrqLow(const std::chrono::milliseconds timeout = std::chrono::milliseconds(125));
};
