##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
project("pika-spark-bno085-driver")
##########################################################################
add_executable(${PROJECT_NAME} 
  main.cpp
  spi.cpp
)
##########################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror -Wextra -Wpedantic)
endif()
##########################################################################
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
##########################################################################
