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

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

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

int main(int argc, char ** argv) try
{
  rclcpp::init(argc, argv);

  auto const node = rclcpp::Node::make_shared("pika_spark_bno085_driver_node");

  rclcpp::QoS imu_qos_profile(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  node->declare_parameter("imu_topic", "joy");
  auto const imu_topic = node->get_parameter("imu_topic").as_string();
  auto const imu_topic_deadline = std::chrono::milliseconds(100);
  auto const imu_topic_liveliness_lease_duration = std::chrono::milliseconds(1000);

  imu_qos_profile.deadline(imu_topic_deadline);
  imu_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  imu_qos_profile.liveliness_lease_duration(imu_topic_liveliness_lease_duration);

  sensor_msgs::msg::Imu imu_msg;
  auto const imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, imu_qos_profile);

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

  auto const acc_callback = [node, &imu_msg](sh2_Accelerometer_t const & data)
  {
    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000UL,
                          "Acceleration [x, y, z,] = [%0.3f, %0.3f, %0.3f] m/s²",
                          data.x, data.y, data.z);

    imu_msg.linear_acceleration.x = data.x;
    imu_msg.linear_acceleration.y = data.y;
    imu_msg.linear_acceleration.z = data.z;
  };

  auto const gyro_callback = [node, &imu_msg](sh2_Gyroscope_t const & data)
  {
    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000UL,
                          "Gyroscope    [x, y, z,] = [%0.3f, %0.3f, %0.3f] rad/s",
                          data.x, data.y, data.z);

    imu_msg.angular_velocity.x = data.x;
    imu_msg.angular_velocity.y = data.y;
    imu_msg.angular_velocity.z = data.z;
  };

  auto const attitude_callback = [node, imu_pub, &imu_msg](sh2_RotationVectorWAcc_t const & data)
  {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 250UL,
                "Attitude     [i, j, k, real, accuracy] = [%0.3f, %0.3f, %0.3f, %0.3f, %0.3f]",
                data.i, data.j, data.k, data.real, data.accuracy);

    imu_msg.header.stamp = node->now();
    imu_msg.header.frame_id = std::string("imu");

    imu_msg.orientation.x = data.i;
    imu_msg.orientation.y = data.j;
    imu_msg.orientation.z = data.k;
    imu_msg.orientation.w = data.real;

    imu_pub->publish(imu_msg);
  };

  auto spi = std::make_shared<SPI>("/dev/spidev0.0", SPI_MODE_3, 8, 3*1000*1000UL);
  auto bno085 = std::make_shared<BNO085>(spi, gpio_nirq, acc_callback, gyro_callback, attitude_callback);

  /* Configure sensor for obtaining current orientation
   * as a quaternion with accuracy estimation.
   */
  if (auto const rc = bno085->enableAccelerometer(); rc != SH2_OK) {
    std::cerr << "\"enableAccelerometer()\" failed with error code " << rc << std::endl;
    return EXIT_FAILURE;
  }
  if (auto const rc = bno085->enableGyroscope(); rc != SH2_OK) {
    std::cerr << "\"enableGyroscope()\" failed with error code " << rc << std::endl;
    return EXIT_FAILURE;
  }
  if (auto const rc = bno085->enableAttitude(); rc != SH2_OK) {
    std::cerr << "\"enableAttitude()\" failed with error code " << rc << std::endl;
    return EXIT_FAILURE;
  }

  /* Run until killed by Ctrl+C to service the sensor hub.
   */
  RCLCPP_INFO(node->get_logger(), "%s init complete.", node->get_name());

  while (rclcpp::ok())
  {
    /* Let ROS do its things. */
    rclcpp::spin_some(node);

    /* Check if an event has been signalled via the
     * BNO085 nIRQ pin.
     */
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

  RCLCPP_INFO(node->get_logger(), "%s shut down.", node->get_name());

  rclcpp::shutdown();
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
