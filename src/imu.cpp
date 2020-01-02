#include "imu.hpp"

// Constructors
//TODO Fix default constructor
IMU::IMU()
{
    m_device_name = "/dev/i2c-1";
}
IMU::IMU(char* device_str)
{
    m_device_name = device_str;
}

// Methods
// Initializes I2C on specified bus
bool IMU::initI2C()
{
    if (m_device_name != NULL)
    {
        m_fd_IMU = open(m_device_name, O_RDWR); // Open device in ReaD and WRite mode
        if (m_fd_IMU < 0)
        {
            std::cout << "Failed to open I2C bus on " << m_device_name << std::endl;
            return false;
        } else {
            std::cout << "Successfully opened I2C bus on " << m_device_name << std::endl;
            return true;
        }
    }
}

// Selects sensors to read values from
// Check that IMU is alive using "i2cdetect -y 1" in Terminal
void IMU::selectSensors(int option)
{
    int sensor_address = 0x0;
    switch (option)
    {
        case 0:
        {
            sensor_address = m_accel_gyro_address;
            if (ioctl(m_fd_IMU, I2C_SLAVE, sensor_address) < 0)
            {
                std::cout << "Failed to access LSM6DS33 (Accelerometer/Gyroscope)" << std::endl;
            } else {
                std::cout << "Successfully accessing LSM6DS33 (Accelerometer/Gyroscope)" << std::endl;
            }
            break;
        }
        case 1:
        {
            sensor_address = m_mag_address;
            if (ioctl(m_fd_IMU, I2C_SLAVE, sensor_address) < 0)
            {
                std::cout << "Failed to access LIS3MDL (Magnetometer)" << std::endl;
            } else {
                std::cout << "Successfully accessing LIS3MDL (Magnetometer)" << std::endl;
            }
            break;
        }
        default:
        {
            sensor_address = m_accel_gyro_address;
            if (ioctl(m_fd_IMU, I2C_SLAVE, sensor_address) < 0)
            {
                std::cout << "Failed to access LSM6DS33 (Accelerometer/Gyroscope)" << std::endl;
            } else {
                std::cout << "Successfully accessing LSM6DS33 (Accelerometer/Gyroscope)" << std::endl;
            }
            break;
        }
    }
}

// Initialize sensors by writing to CTRL1_XL and CTRL2_G addresses
void IMU::initSensors()
{
    unsigned char initBuffer[2] = "";   // Buffer for Register Address, Value
    
    // Enable Accelerometer
    initBuffer[0] = 0x10;   // CTRL1_XL register address, Page 23 of LSM6DS33.pdf
    initBuffer[1] = 0x84;   // Enable 1.66kHz mode, FS_XL set to +- 16g
    if (write(m_fd_IMU, initBuffer, 2) != 2)
    {
        std::cout << "Failed to write to I2C bus (Accelerometer)" << std::endl;
    } else {
        std::cout << "Accelerometer enabled at 1.66KHz mode" << std::endl;
    }

    // Enable Gyroscope
    initBuffer[0] = 0x11; // CTRL2_G address, Page 23 of LSM6DS33.pdf
    initBuffer[1] = 0x8C; // Enable 1.66kHz mode, FS_G set to +- 2000dps
    if (write(m_fd_IMU, initBuffer, 2) != 2)
    {
        std::cout << "Failed to write to I2C bus (Gyroscope)" << std::endl;
    } else {
        std::cout << "Gyro enabled at 1.66KHz mode" << std::endl;
    }
}

/* Read data from selected sensors */
void IMU::readSensors()
{
    unsigned char dataBuffer[12] = ""; // Buffer for accelerometer/gyroscope
    //TODO Create arrays for all data, instead of many variables
    short AxRaw = 0, AyRaw = 0, AzRaw = 0, GxRaw = 0, GyRaw = 0, GzRaw = 0, MxRaw = 0, MyRaw = 0, MzRaw = 0;
    float GxOffset = 0, GyOffset = 0, GzOffset = 0, AxOffset = 0, AyOffset = 0, AzOffset = 0;
    float Ax = 0, Ay = 0, Az = 0, Gx = 0, Gy = 0, Gz, Mx, My, Mz;
    float GxRate = 0, GyRate = 0, GzRate = 0;
    float AyAngle = 0;
    float CFyAngle = 0;         // Balancing angle
    const float K_CF = 0.95;    // Complementary filter constant
    const float A_SENS = 0.488; // Accelerometer sensitivity
    const float G_SENS = 0.07;  // Gyroscope sensitivity
    float dt_target = 0.02;     // Target interval for each iteration in s
    float dt = 0.02;            // 0.02s = 20milliseconds
    struct timeval start, end;  // Containers for storing time
    
    int calibrationIter = 10000;
    // Calibrate accelerometer/gyroscope
    for (int i=0; i<calibrationIter; i++)
    {
        // m_buffer_address VS &m_buffer_address
        if (write(m_fd_IMU, &m_buffer_address, 1) != 1)
        {
            std::cout << "Failed to change address pointer to" << m_buffer_address << std::endl;
            return;
        }
        if (read(m_fd_IMU, dataBuffer, 12) != 12){
            std::cout << "Something's wrong, received data length is not 12" << std::endl;
            return;
        } else {
            // << for Shift Left Logical operation
            // Combining Most Significant Bits (MSB) and Least Significant Bits (LSB)
            GxRaw = (short)(dataBuffer[1] << 8 | dataBuffer[0]);
            GyRaw = (short)(dataBuffer[3] << 8 | dataBuffer[2]);
            GzRaw = (short)(dataBuffer[5] << 8 | dataBuffer[4]);

            AxRaw = (short)(dataBuffer[7] << 8 | dataBuffer[6]);
            AyRaw = (short)(dataBuffer[9] << 8 | dataBuffer[8]);
            AzRaw = (short)(dataBuffer[11]<< 8 | dataBuffer[10]);
        }
        // Accumulate the offset in Gyro readings
        GxOffset += GxRaw;
        GyOffset += GyRaw;
        GzOffset += GzRaw;
    }
    GxOffset = GxOffset/calibrationIter;
    GyOffset = GyOffset/calibrationIter;
    GzOffset = GzOffset/calibrationIter;

    // Reset raw values to zero 
    GxRaw = 0;
    GyRaw = 0;
    GzRaw = 0;
    while (true)
    {
        // m_buffer_address VS &m_buffer_address
        if (write(m_fd_IMU,&m_buffer_address,1) != 1)
        {
           // Error Handling: I2C transaction failed
            std::cout << "Failed to change address pointer to" << m_buffer_address << std::endl;
            return;
        }
        if (read(m_fd_IMU,dataBuffer,12) != 12)
        {
            std::cout << "Something's wrong, received data length is not 12" << std::endl;
            return;
        } else {
            // Get current time, store into start
            gettimeofday(&start,NULL);
            // << for Shift Left Logical operation
            // Combining Most Significant Bits (MSB) and Least Significant Bits (LSB)
            GxRaw = (short)(dataBuffer[1] << 8 | dataBuffer[0]);
            GyRaw = (short)(dataBuffer[3] << 8 | dataBuffer[2]);
            GzRaw = (short)(dataBuffer[5] << 8 | dataBuffer[4]);

            AxRaw = (short)(dataBuffer[7] << 8 | dataBuffer[6]);
            AyRaw = (short)(dataBuffer[9] << 8 | dataBuffer[8]);
            AzRaw = (short)(dataBuffer[11]<< 8 | dataBuffer[10]);
        }

        // Converting data types
        Gx = (float)GxRaw - GxOffset;
        Gy = (float)GyRaw - GyOffset;
        Gz = (float)GzRaw - GzOffset;
        Ax = (float)AxRaw;
        Ay = (float)AyRaw;
        Az = (float)AzRaw;
        // Rates for gyroscope, in degrees per second
        GxRate = Gx*G_SENS;
        GyRate = Gy*G_SENS;
        GzRate = Gz*G_SENS;

        // Convert raw data to Euler angles
        AyAngle = atan2(Ay,sqrt(pow(Ax,2) + pow(Az,2))) * 180/PI;

        // Apply Complementary Filter to data
        CFyAngle = (K_CF*(CFyAngle+GyRate*dt) + (1-K_CF)*AyAngle);
        std::cout << "Balancing angle: " << CFyAngle << std::endl;      
        // Computing dt, Units: milliseconds
        gettimeofday(&end,NULL);
        dt = (float) (end.tv_sec - start.tv_sec) * 1000 + (float) (end.tv_usec - start.tv_usec) / 1000;
        if (dt<dt_target && dt>0)
        {
            delay((dt_target - dt )*1000);
        }
        // TODO: Filter data (Kalman/Extended Kalman)
    }
}