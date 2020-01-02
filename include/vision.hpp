#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <uart.hpp>
#include <util.hpp>

class Vision
{
    private:
        int m_device_name = 0;
        const int m_width = 640; // Full width of camera
        const int m_height = 480; // Full height of camera
    public:
        Vision();
        Vision(int);
        void initVision(float, char, UART&);
};