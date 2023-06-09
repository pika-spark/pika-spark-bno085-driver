/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/pika-spark-bno085-driver/graphs/contributors.
 */

#include <cstdlib>

#include <memory>
#include <iostream>
#include <stdexcept>

#include "spi.h"
#include "bno085.h"

int main(int /* argc */, char ** /* argv */) try
{
  auto spi = std::make_shared<SPI>("/dev/spidev1.0", SPI_MODE_3, 8, 1*1000*1000UL);
  auto bno085 = std::make_shared<BNO085>(spi);
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
