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

#include "bno085_exception.h"

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

BNO085::BNO085(std::shared_ptr<SPI> const spi,
               std::shared_ptr<SysGPIO> const nirq,
               CalibratedGyroscopeCallbackFunc const gyro_callback,
               StabilizedRotationVectorWAccuracyCallbackFunc const attitude_callback)
: BNO085_HAL{spi, nirq}
, _gyro_callback{gyro_callback}
, _attitude_callback{attitude_callback}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

int BNO085::enableGyroscope()
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

  return sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &bno085_config);
}

int BNO085::enableAttitude()
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

/**************************************************************************************
 * PROTECTED MEMBER FUNCTIONS
 **************************************************************************************/

void BNO085::internal_sh2_sensor_callback(sh2_SensorEvent_t * event)
{
  sh2_SensorValue_t sensor_value;

  if (auto const rc = sh2_decodeSensorEvent(&sensor_value, event); rc != SH2_OK)
    throw BNO085_Exception("error decoding sensor event, rc = %d", rc);

  if (sensor_value.sensorId == SH2_GYROSCOPE_CALIBRATED)
    _gyro_callback(sensor_value.un.gyroscope);
  else if (sensor_value.sensorId == SH2_ARVR_STABILIZED_RV)
   _attitude_callback(sensor_value.un.arvrStabilizedRV);
  else
    throw BNO085_Exception("unhandled sensor event decoded, sensorId = %d", sensor_value.sensorId);
}
