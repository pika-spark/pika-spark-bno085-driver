/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/pika-spark-bno085-driver/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <cstdio>
#include <cstdarg>

#include <stdexcept>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class BNO085_Exception : public std::exception
{
private:
  std::string _err_msg;

public:
  BNO085_Exception(char const * fmt, ...)
  {
    va_list args;
    va_start(args, fmt);
    char msg_buf[256] = {0};
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    va_end(args);
    _err_msg = std::string(msg_buf);
  }

  const char * what() const noexcept override
  {
    return _err_msg.c_str();
  }
};
