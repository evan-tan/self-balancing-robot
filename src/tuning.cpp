#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

// Global variables
const String window_capture_name = "Original";
const String window_detection_name = "Thresholding";
const String window_erosion_name = "Erosion";
const String window_dilation_name = "Dilation";
const String window_blob = "Blob";

// Width and height of frame
const int TARGET_WIDTH = 160;
const int TARGET_HEIGHT = 120;

// Create image containers
Mat frame, frame_HSV, frame_threshold;
// Erosion and dilation variables
Mat erosion_dst, dilation_dst;
int erosion_type;
int erosion_elem = 0;
int erosion_size = 2;
int dilation_type;
int dilation_elem = 0;
int dilation_size = 2;
int const max_elem = 2;
int const max_kernel_size = 21;

const int max_value = 255;
const int max_value_H = 360/2;

// Tuned values so far
int low_H = 40;
int low_S = 100;
int low_V = 100;
int high_H = 100;
int high_S = 225;
int high_V = 225;

// int low_H = 30;
// int low_S = 105;
// int low_V = 80;
// int high_H = 80;
// int high_S = 225;
// int high_V = 225;

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

/* @function Erosion  */
void Erosion( int, void* )
{

  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
}

/* @function Dilation */
void Dilation( int, void* )
{

  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
}

int main(int argc, char* argv[])
{
    // Open camera
    int deviceID = 0;
    VideoCapture camera(deviceID); /* Open the default camera */
    if (!camera.isOpened()) {return -1;} /* Check if camera opened successfully */
    // Reducing resolution of camera
    // Also increases RPi3's GPU memory from 128 to 256, almost no lag now
    camera.set(CV_CAP_PROP_FRAME_WIDTH, TARGET_WIDTH);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT);

    // Create windows to display video
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    namedWindow(window_erosion_name);
    namedWindow(window_dilation_name);
    namedWindow(window_blob);

    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    createTrackbar("Kernel", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    
    // Create Erosion Trackbar
    createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_erosion_name, &erosion_elem, max_elem, Erosion);
    createTrackbar("Kernel size:\n 2n +1", window_erosion_name, &erosion_size, max_kernel_size, Erosion);
    /// Create Dilation Trackbar
    createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_dilation_name, &dilation_elem, max_elem, Dilation);
    createTrackbar( "Kernel size:\n 2n +1", window_dilation_name, &dilation_size, max_kernel_size, Dilation);

    // Erosion setup
    Mat element_erosion, element_dilation;
    Mat image_inverted, image_blob;

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;
    // Change thresholds
    params.minThreshold = 10; // oh shit
    params.maxThreshold = 200;
    params.filterByArea = false;          // Filter by Area.
    params.minArea = 500;
    params.filterByCircularity = true;  // Filter by Circularity
    params.minCircularity = 0.1;
    params.filterByConvexity = false;   // Filter by Convexity
    params.minConvexity = 0.87;
    params.filterByInertia = false;     // Filter by Inertia
    params.minInertiaRatio = 0.01;
    params.blobColor = 255;

    // Storage for blobs
    vector<KeyPoint> keypoints;
    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    // cv::SimpleBlobDetector detector(params);

    while (true) {
        // Grab next frame from camera
        camera >> frame;
        if(frame.empty()) { break; }

        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        // Format: inRange(inputFrame, ..., ..., destinationFrame)
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        
        element_erosion = getStructuringElement( erosion_type, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size));
        element_dilation = getStructuringElement( dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size));
        // Apply the erosion operation
        erode( frame_threshold, erosion_dst, element_erosion);
        // Apply the dilation operation
        dilate( erosion_dst, dilation_dst, element_dilation);

        // Detect blobs after dilating image
        detector->detect(dilation_dst, keypoints);
        // drawKeypoints(image_inverted, keypoints, image_blob, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        drawKeypoints(dilation_dst, keypoints, image_blob, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT); // Seems to show centre
        std::cout << keypoints.size() << std::endl;
        for (int i=0; i<keypoints.size(); i++){
            cv::KeyPoint currKP = keypoints[i];
            std::cout << currKP.pt.x << "\t" << currKP.pt.y << std::endl;
        }
        // Show the frames
        imshow(window_capture_name, frame);
        imshow(window_detection_name, frame_threshold);
        // Error checking to prevent runtime errors
        if (!(erosion_dst.empty()))
        {
            imshow(window_erosion_name, erosion_dst);
        }
        if (!(dilation_dst.empty()))
        {
            imshow(window_dilation_name, dilation_dst);
        }
        imshow(window_blob,image_blob);

        // TODO: Get the location of image_blob

        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            // key == 27 for ESC
            break;
        }
    }
    // Camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}