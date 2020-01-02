#include "uart.hpp"

// Constructors
//TODO Fix default constructor
UART::UART()
{
    // Open the serial bus at ttyS0(check your RPi3 tho using "ls -l /dev" in Terminal and seeing which is linked to serial0)
    m_device_name = "/dev/ttyS0";
}
UART::UART(char* device)
{
    m_device_name = device;
}

// Methods
void UART::initUART()
{
    m_fd_UART = serialOpen(m_device_name, m_baud_rate);
    if (m_fd_UART == -1)
    {
        std::cout << "Failed to open Serial bus on " << m_device_name << std::endl;
    } else {
        std::cout << "Successfully opened Serial bus on " << m_device_name << std::endl;
    }
}

/* Send single char (8bits) via serial bus to specified device with delay in milliseconds */
void UART::sendSingleChar(char data, int delayTime)
{
    if (delayTime == 0)
    {
        serialPutchar(m_fd_UART, data);
    } else {
        delay(delayTime);
        serialPutchar(m_fd_UART, data);
    }
}

/* Send multiple char via serial bus to specified device with delay in milliseconds */
void UART::sendMultiChar(char* string, int delayTime, char* startKey, char* endKey)
{
    // Send the end key first as it's pushed further into the buffer
    serialPutchar(m_fd_UART, endKey[0]);
    for (int i=strlen(string)-1; i>=0; i--)
    {
        serialPutchar(m_fd_UART, *(string+i));
        if (delayTime != 0)
        {
            delay(delayTime);
        }
    }
    // Send the start key last, so it's closer to the start of the buffer
    serialPutchar(m_fd_UART, startKey[0]);
}

/* Receive data from serial bus */
void UART::receiveData(char* buffer)
{
    int numCharAvail = serialDataAvail(m_fd_UART);
    char* dataReceived = "";
    if (numCharAvail > 0)
    {
        for (int i=0; i<numCharAvail; i++)
        {
            *(dataReceived+i) = serialGetchar(m_fd_UART);
        }
    } else if (serialDataAvail(m_fd_UART) == -1)
    {
        std::cout << "No data available" << std::endl;
    }
}

const int UART::getFd()
{
    return m_fd_UART;
}