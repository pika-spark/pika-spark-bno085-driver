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
  typedef std::function<void(sh2_Accelerometer_t const &)> LinearAccelerationCallbackFunc;
  typedef std::function<void(sh2_Gyroscope_t const &)> CalibratedGyroscopeCallbackFunc;
  typedef std::function<void(sh2_RotationVectorWAcc_t const &)> StabilizedRotationVectorWAccuracyCallbackFunc;

  BNO085(std::shared_ptr<SPI> const spi,
         std::shared_ptr<SysGPIO> const nirq,
         LinearAccelerationCallbackFunc const acc_callback,
         CalibratedGyroscopeCallbackFunc const gyro_callback,
         StabilizedRotationVectorWAccuracyCallbackFunc const attitude_callback);
  virtual ~BNO085() { }


  int  enableAccelerometer();
  int  enableGyroscope();
  int  enableAttitude();
  void spinOnce();


protected:
  virtual void internal_sh2_sensor_callback(sh2_SensorEvent_t * event) override;


private:
  LinearAccelerationCallbackFunc const _acc_callback;
  CalibratedGyroscopeCallbackFunc const _gyro_callback;
  StabilizedRotationVectorWAccuracyCallbackFunc const _attitude_callback;
};
