#pragma once

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <wiringPi.h>
#include <wiringSerial.h>

class UART
{
    private:
        char* m_device_name = "/dev/ttyS0"; // Serial port enabled on Raspberry Pi, check using "ls -l /dev/"
        int m_baud_rate = 115200;
        int m_fd_UART = NULL;

    public:
        UART();
        UART(char*);
        void initUART();
        void sendSingleChar(char, int);
        void sendMultiChar(char*, int, char*, char*);
        void receiveData(char*);
        const int getFd();
};