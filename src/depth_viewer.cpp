#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;

class Depth_viewer
{ 
 public:                                                      //public functions
	Depth_viewer()                                         //declare a constructor, same name as the class name.

		{
		 min_range_ = 0.5;
		 max_range_ = 10.5; 
		}
	Mat img_filter(Mat);                              //member function declaration
	void depthCb(const sensor_msgs::ImageConstPtr&);
	void image_show(Mat);
	Mat img_erosion;
	Mat img_dilation;
	Mat img_blur;
    Mat filt_img;
    double min_range_;
    double max_range_; 
};




Mat Depth_viewer::img_filter(Mat img_origin)                                               //member function definition
{
	Mat element = getStructuringElement (1, Size(5,5), Point(2,2));  // how to assign this value in constructor???
	GaussianBlur(img_origin, img_blur, Size(5,5), 0, 0);
	erode(img_blur, img_erosion, element);
	dilate(img_erosion, img_dilation, element);

	return(img_erosion);
}




void Depth_viewer::depthCb(const sensor_msgs::ImageConstPtr& image)         //?????
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
        cout<<"error in subscribing image"<<endl;
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
    //display
    filt_img = Depth_viewer::img_filter(img);
    Depth_viewer::image_show(filt_img);
    // return (img);
}

void Depth_viewer::image_show(Mat img_show)
{
	cv::imshow("image_show", img_show);
	cv::waitKey(1);
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "depth_viewer_node" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    Depth_viewer depth_viewer;
    nh.param("min_range", depth_viewer.min_range_, 0.5);
    nh.param("max_range", depth_viewer.max_range_, 10.5);
    
    ros::Subscriber sub = n.subscribe("/camera/depth/image_rect_color", 3, &Depth_viewer::depthCb, &depth_viewer);


    ros::spin();

}