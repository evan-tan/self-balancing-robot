#pragma once

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <wiringPi.h>

// Global variables
#define PI 3.1415926

class IMU
{
    private:
        char* m_device_name = "/dev/i2c-1"; // I2C port enabled on Raspberry Pi
        int m_fd_IMU = NULL; // "file descriptor" of IMU
        const int m_accel_gyro_address = 0x6B;
        const int m_mag_address = 0x1E;
        const int m_buffer_address = 0x22;
    public:
        IMU();
        IMU(char*);
        bool initI2C();
        void selectSensors(int);
        void initSensors();
        void readSensors();
};