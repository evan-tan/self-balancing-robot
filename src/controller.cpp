#include "imu.hpp"
// #include "uart.hpp"
#include "vision.hpp"

// Global variables
int fd_IMU = NULL;
int fd_UART = NULL;


int main(int argc, char argv[])
{
    wiringPiSetup();
    UART uart_handler("/dev/ttyS0");
    uart_handler.initUART();

    Vision visionHandler(0);
    visionHandler.initVision(0.25, 0, uart_handler); // Enters infinite while loop

    IMU imu_handler("/dev/i2c-1");
    imu_handler.initI2C();
    imu_handler.selectSensors(0);
    imu_handler.initSensors();
    imu_handler.readSensors(); // Enters infinite while loop
    
    return 0;
}