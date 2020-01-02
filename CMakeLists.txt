cmake_minimum_required (VERSION 2.8) 
project (self-balancing-robot) 
find_package(OpenCV REQUIRED)

include_directories (include src) 
add_executable (controller src/controller.cpp src/imu.cpp src/uart.cpp src/vision.cpp include/imu.hpp include/uart.hpp include/vision.hpp include/util.hpp) 
find_library(WIRINGPI_LIBRARIES NAMES wiringPi) 
target_link_libraries(controller ${WIRINGPI_LIBRARIES} ${OpenCV_LIBS}) 

add_executable (tuning src/tuning.cpp) 
target_link_libraries(tuning ${OpenCV_LIBS}) 