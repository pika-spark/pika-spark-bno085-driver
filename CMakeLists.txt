##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
project("pika_spark_bno085_driver")
set(PIKA_SPARK_BNO085_TARGET ${PROJECT_NAME}_node)
##########################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
##########################################################################
include_directories(${PIKA_SPARK_BNO085_TARGET} include)
##########################################################################
add_subdirectory(sh2)
##########################################################################
add_executable(${PIKA_SPARK_BNO085_TARGET}
        src/main.cpp
        src/bno085.cpp
        src/bno085_hal.cpp
        src/spi.cpp
        src/gpio-sysfs.cpp
)
##########################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PIKA_SPARK_BNO085_TARGET} PRIVATE -Wall -Werror -Wextra -Wpedantic)
endif()
##########################################################################
target_link_libraries(${PIKA_SPARK_BNO085_TARGET} pika-spark-sh2)
##########################################################################
target_compile_features(${PIKA_SPARK_BNO085_TARGET} PUBLIC cxx_std_17)
##########################################################################
ament_target_dependencies(${PIKA_SPARK_BNO085_TARGET} rclcpp sensor_msgs)
##########################################################################
install(TARGETS ${PIKA_SPARK_BNO085_TARGET} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
##########################################################################
ament_package()
##########################################################################
