#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <iomanip>

#include <fstream>
using namespace std;
using namespace cv;


int main( int argc, char** argv )
{
    Mat colorImage_dst, colorImage_src = imread("/home/eee/Pictures/depth_img2.png");

    // Open the file in write mode.
    ofstream outputFile;
    outputFile.open("/home/eee/catkin_ws/src/SMART_ZED/src/depth_img_2_mat.txt");

    resize(colorImage_src, colorImage_dst, Size(), 0.1, 0.1, 0);

    // Iterate through pixels.
    for (int r = 0; r < colorImage_dst.rows; r++)
    {
        for (int c = 0; c < colorImage_dst.cols; c++)
        {
            int pixel = colorImage_dst.at<uchar>(r,c);

            outputFile << pixel << '\t';
        }
        outputFile << endl;
    }

    // Close the file.
    outputFile.close();
    return 0;
}