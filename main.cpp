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
#include "bno085.h"
#include "gpio-sysfs.h"

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

  auto const gpio_nrst = std::make_shared<SysGPIO>(nRST_PIN);
  gpio_nrst->gpio_set_dir(true);
  gpio_nrst->gpio_set_value(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gpio_nrst->gpio_set_value(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto const gpio_nirq = std::make_shared<SysGPIO>(nIRQ_PIN);
  gpio_nirq->gpio_set_dir(false);

  auto spi = std::make_shared<SPI>("/dev/spidev1.0", SPI_MODE_3, 8, 1*1000*1000UL);
  auto bno085 = std::make_shared<BNO085>(spi);

  if (auto const rc = bno085->begin(); rc != SH2_OK) {
    std::cerr << "begin failed with error code " << rc << std::endl;
    return EXIT_FAILURE;
  }

  sh2_ProductIds_t prod_ids;
  if (auto const rc = bno085->readProductIds(&prod_ids); rc != SH2_OK) {
    std::cerr << "readProductIds failed with error code " << rc << std::endl;
    return EXIT_FAILURE;
  }
  for (size_t i = 0; i < prod_ids.numEntries; i++)
  {
    std::cout << "Entry " << i << " of " << prod_ids.numEntries << std::endl;
    std::cout << "\tswVersionMajor = " << prod_ids.entry[i].swVersionMajor << std::endl
              << "\tswVersionMinor = " << prod_ids.entry[i].swVersionMinor << std::endl
              << "\tswPartNumber   = " << prod_ids.entry[i].swPartNumber   << std::endl
              << "\tswBuildNumber  = " << prod_ids.entry[i].swBuildNumber  << std::endl
              << "\tswVersionPatch = " << prod_ids.entry[i].swVersionPatch << std::endl;
  }

  return EXIT_SUCCESS;
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
