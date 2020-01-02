#include <vision.hpp>

// Constructors
// TODO: Fix default constructor
Vision::Vision()
{
    m_device_name = 0;
}
Vision::Vision(int device)
{
    m_device_name = device;
}

// Methods
void Vision::initVision(float scale, char mode, UART& uart_object)
{
    
    using namespace cv;

    // Create image containers
    Mat frame, frame_HSV, frame_threshold;
    Mat erosion_dst, dilation_dst;
    Mat element_erosion, element_dilation;
    Mat frame_final;

    const int scaled_width = scale*m_width;
    const int scaled_height = scale*m_height;
    int blob_thresh = 2500;
    int num_laps = 0;
    State robot_state = Homing;
    // Erosion and dilation parameters
    int erosion_type;
    int erosion_elem = 2;
    int erosion_size = 2;
    int dilation_type;
    int dilation_elem = 2;
    int dilation_size = 2;
    int const max_elem = 2;
    int const max_kernel_size = 21;
    const int max_value = 255;
    const int max_value_H = 360/2;
    // HSV Values for thresholding (inRange)
    int low_H = 40;
    int low_S = 100;
    int low_V = 100;
    int high_H = 100;
    int high_S = 225;
    int high_V = 225;
    // Determine percentages of frame for visual homing
    float hard_val = 0.85;
    float med_val = 0.75;
    float soft_val = 0.6;
    float tolerance = 0.045; 

    // Initialize camera
    VideoCapture camera(m_device_name);
    if (!camera.isOpened())
    { 
        return;
    } else {
        std::cout << "Camera successfully opened" << std::endl;
    }
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, scaled_height);
    camera.set(CV_CAP_PROP_FRAME_WIDTH, scaled_width);

    // Blob Detection
    // Setup SimpleBlobDetector parameters
    cv::SimpleBlobDetector::Params sbd_params; 
    // Change thresholds
    sbd_params.minThreshold = 10; // Minimum value cutoff for pixels to be set to zero
    sbd_params.maxThreshold = 200; // Maximum value cutoff for pixels to be set to zero
    sbd_params.filterByArea = false;        // Filter by Area.
    sbd_params.minArea = 500;
    sbd_params.filterByCircularity = true;  // Filter by Circularity
    sbd_params.minCircularity = 0.1;
    sbd_params.filterByConvexity = false;   // Filter by Convexity
    sbd_params.minConvexity = 0.87;
    sbd_params.filterByInertia = false;     // Filter by Inertia
    sbd_params.minInertiaRatio = 0.01;
    sbd_params.blobColor = 255;             // Blob white bits, for black use 0

    // Storage of keypoints for blob
    std::vector<KeyPoint> keypoints;
    // Set up detector with sbd_params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(sbd_params);

    while (true)
    {
        camera >> frame;   // Grab next frame from camera
        int numPixels = 0; 
        if (frame.empty()) { return; }
    
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        // Format: inRange(inputFrame, ..., ..., destinationFrame)
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        element_erosion = getStructuringElement(erosion_type, Size(2*erosion_size+1, 2*erosion_size+1), Point(erosion_size, erosion_size));
        element_dilation = getStructuringElement(dilation_type, Size(2*dilation_size+1, 2*dilation_size+1), Point(dilation_size, dilation_size));
        // Apply the erosion operation
        erode(frame_threshold, erosion_dst, element_erosion); 
        // Apply the dilation operation  
        dilate(erosion_dst, dilation_dst, element_dilation);    
         // Detect blobs after dilating image
        detector->detect(dilation_dst, keypoints);             
        cv::drawKeypoints(dilation_dst, keypoints, frame_final, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);

        // State: Going straight
        if (keypoints.size() > 0 &&  robot_state == Homing)
        {
            //TODO Handle multiple keypoints
            for (int i=0; i<1; i++)
            {
                cv::KeyPoint greenKeyPoint = keypoints[i];
                float x = greenKeyPoint.pt.x;
                float y = greenKeyPoint.pt.y;
                switch (mode)
                {
                    default:
                    {
                        // Override percentages of frame for visual homing
                        float hard_val_ovr= 0.9;
                        float med_val_ovr = 0.7;
                        float soft_val_ovr = 0.6;
                        float tolerance = 0.03;

                        /* Legend
                        A = hard left, a = soft left
                        D = hard right, d = soft right
                        q = slight left, e = slight right
                        W = move forward
                        T = turn
                        k = killswitch
                        */

                        //TODO Control motors directly from RPi 
                        if (x>=hard_val_ovr*scaled_width)
                        {
                            uart_object.sendSingleChar('A', 0);
                        } else if (x>=med_val_ovr*scaled_width && x<hard_val_ovr*scaled_width) {
                            uart_object.sendSingleChar('a', 0);
                        } else if (x>=soft_val_ovr*scaled_width && x<med_val_ovr*scaled_width) {
                            uart_object.sendSingleChar('q' ,0);
                        } else if (x>(0.5-tolerance)*scaled_width && x<(0.5+tolerance)*scaled_width) {
                            uart_object.sendSingleChar('W', 0);
                        } else if (x<=(1-soft_val_ovr)*scaled_width && x>(1-med_val_ovr)*scaled_width) {
                            uart_object.sendSingleChar('e', 0);
                        } else if (x<=(1-med_val_ovr)*scaled_width && x>(1-hard_val_ovr)*scaled_width) {
                            uart_object.sendSingleChar('d', 0);
                        } else if (x<=(1-hard_val_ovr)*scaled_width){
                            uart_object.sendSingleChar('D', 0);
                        }
                    }
                }
            }

            // Iterate through Mat object
            for (int j=0; j<frame_final.rows; j++)
            {
                for (int k=0; k<frame_final.cols; k++)
                {
                    // Count number of nonzero pixels
                    // <float> because frame_final.type() is 16 i.e. CV_32F
                    if (frame_final.at<float>(j,k) != 0)
                    { 
                        numPixels++;
                    }
                }
            }

            // Check if we should turn or not
            if (numPixels > blob_thresh){
                robot_state = Turning;
            } else if (numPixels <= blob_thresh) {
                robot_state = Homing;
            }

            // This doesn't do anything atm, make an empty frame to use this functionality
            char key = (char) waitKey(10);
            if (key == 27 || key == 'q'){
                uart_object.sendSingleChar('k',0);
                serialFlush(uart_object.getFd());
                serialClose(uart_object.getFd());
                return;
            }
        } else if (robot_state == Turning) {
            uart_object.sendSingleChar('T',0);

            // Iterate through Mat object
            for (int j = 0; j < frame_final.rows; j++){
                for (int k = 0; k < frame_final.cols; k++){
                    // Count number of nonzero pixels
                    // <float> because frame_final.type() is 16 i.e. CV_32F
                    if (frame_final.at<float>(j,k) != 0){ 
                        numPixels++;
                    }
                }
            }
            if (numPixels > blob_thresh)
            {
                robot_state = Turning;
            }else if (numPixels <= blob_thresh){
                robot_state = Homing;
            }
        }
    }
}