#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;

Mat src, erosion_dst, dilation_dst;

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;


static const char WINDOW_NAME[] = "Depth View";
double min_range_;
double max_range_;

void depthCb( const sensor_msgs::ImageConstPtr& image )
{
    // convert to cv image
    cv_bridge::CvImagePtr bridge;
    try
    {
        bridge = cv_bridge::toCvCopy(image, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to transform depth image.");
        return;
    }

    // convert to something visible
    Mat img(bridge->image.rows, bridge->image.cols, CV_8UC1);
    for(int i = 0; i < bridge->image.rows; i++)
    {
        float* Di = bridge->image.ptr<float>(i);//get ith row of original image, index Di
        char* Ii = img.ptr<char>(i);//get ith row of img, index Ii
        for(int j = 0; j < bridge->image.cols; j++)
        {   
            Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_))); //normalize and copy Di to Ii
            Ii[j] = (char) (255 - Ii[j]);
        }   
    }

    // display

    // cv::imshow(WINDOW_NAME, img);
    // cv::waitKey(1);

    // cout << "img = "<< endl << " "  << img << endl << endl;
    // cout << "========================================" << endl << endl;
    Mat img_blur;
    GaussianBlur(img, img_blur, Size(5,5), 0, 0);
    imshow(WINDOW_NAME, img_blur);
    waitKey(1);

}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "depth_viewer_node" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param("min_range", min_range_, 0.0);
    nh.param("max_range", max_range_, 20.0);

    cv::namedWindow(WINDOW_NAME);
    
    ros::Subscriber sub = n.subscribe("/camera/depth/image_rect_color", 3, &depthCb);
    ros::spin();
}

