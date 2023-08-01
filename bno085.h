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

#include "bno085_hal.h"

#include <functional>

#include <sh2_SensorValue.h>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class BNO085 : private BNO085_HAL
{
public:
  typedef std::function<void(sh2_RotationVectorWAcc_t const &)> StabilizedRotationVectorWAccuracyCallbackFunc;

  BNO085(std::shared_ptr<SPI> const spi,
         std::shared_ptr<SysGPIO> const nirq,
         StabilizedRotationVectorWAccuracyCallbackFunc const sensor_func);
  virtual ~BNO085() { }


  int  config();
  void spinOnce();


protected:
  virtual void internal_sh2_sensor_callback(sh2_SensorEvent_t * event) override;


private:
  StabilizedRotationVectorWAccuracyCallbackFunc const _sensor_func;
};
