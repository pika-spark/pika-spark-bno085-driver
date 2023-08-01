/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/pika-spark-bno085-driver/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdlib>

#include <memory>
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>

#include "spi.h"
#include "gpio-sysfs.h"

#include "bno085.h"
#include "bno085_exception.h"

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static int constexpr nRST_PIN  =  86;
static int constexpr nBOOT_PIN =  87;
static int constexpr nIRQ_PIN  = 131;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int /* argc */, char ** /* argv */) try
{
  auto const gpio_nboot = std::make_shared<SysGPIO>(nBOOT_PIN);
  gpio_nboot->gpio_set_dir(true);
  gpio_nboot->gpio_set_value(1); /* Note: setting it to '0' activates bootloader mode. */
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); /* Ensure that the value of the pin is settled before its sampled during reset. */

  auto const gpio_nrst = std::make_shared<SysGPIO>(nRST_PIN);
  gpio_nrst->gpio_set_dir(true);
  gpio_nrst->gpio_set_value(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gpio_nrst->gpio_set_value(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto const gpio_nirq = std::make_shared<SysGPIO>(nIRQ_PIN);
  gpio_nirq->gpio_set_dir(false);
  gpio_nirq->gpio_set_edge("falling");

  auto arvrStabilizedRV_callback_last = std::chrono::steady_clock::now();
  auto const arvrStabilizedRV_callback = [&arvrStabilizedRV_callback_last](sh2_RotationVectorWAcc_t const & data)
  {
    auto const now = std::chrono::steady_clock::now();
    auto const diff_time = (now - arvrStabilizedRV_callback_last);
    auto const diff_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(diff_time).count();
    arvrStabilizedRV_callback_last = now;

    char msg[128] = {0};
    snprintf(msg,
             sizeof(msg),
             "[%010ld] [i, j, k, real, accuracy] = [%0.3f, %0.3f, %0.3f, %0.3f, %0.3f]",
             diff_time_ms,
             data.i,
             data.j,
             data.k,
             data.real,
             data.accuracy);
    std::cout << msg << std::endl;
  };

  auto spi = std::make_shared<SPI>("/dev/spidev0.0", SPI_MODE_3, 8, 1*1000*1000UL);
  auto bno085 = std::make_shared<BNO085>(spi, gpio_nirq, arvrStabilizedRV_callback);

  /* Configure sensor for obtaining current orientation
   * as a quaternion with accuracy estimation.
   */
  if (auto const rc = bno085->config(); rc != SH2_OK) {
    std::cerr << "config failed with error code " << rc << std::endl;
    return EXIT_FAILURE;
  }

  /* Run until killed by Ctrl+C to service
   * the sensor hub.
   */
  for (;;)
  {
    auto const gpio_nirq_fd = gpio_nirq->gpio_fd_open();
    auto const rc_poll = gpio_nirq->gpio_poll(gpio_nirq_fd, 1000);
    gpio_nirq->gpio_fd_close(gpio_nirq_fd);

    if      (rc_poll < 0)
      throw BNO085_Exception("poll(gpio_nirq) failed with error code %d", rc_poll);
    else if (rc_poll == 0)
      std::cout << "poll(gpio_nirq) timed out." << std::endl;
    else
      bno085->spinOnce();
  }

  return EXIT_SUCCESS;
}
catch (BNO085_Exception const & err)
{
  std::cerr << "BNO085 exception caught: " << err.what() << std::endl;
  std::cerr << "Terminating ..." << std::endl;
  return EXIT_FAILURE;
}
catch (std::runtime_error const & err)
{
  std::cerr << "Exception (std::runtime_error) caught: " << err.what() << std::endl;
  std::cerr << "Terminating ..." << std::endl;
  return EXIT_FAILURE;
}
catch (...)
{
  std::cerr << "Unhandled exception caught." << std::endl;
  std::cerr << "Terminating ..." << std::endl;
  return EXIT_FAILURE;
}
