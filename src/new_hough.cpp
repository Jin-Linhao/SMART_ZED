/**
 * @file new_hough.cpp
 * @author: Jin Linhao
 * @for barrier detection, SMART FM IRG
 */
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

/// Global variables

/** General variables */

class Line_detection
{
  public:
    Mat src, edges;
    Mat src_gray;
    Mat probabilistic_hough;
    int min_threshold = 50;

    const char* standard_name = "Standard Hough Lines Demo";
    const char* probabilistic_name = "Probabilistic Hough Lines Demo";


    /// Function Headers
    void hough_line_callback( const sensor_msgs::ImageConstPtr& image );
    void help();
    void Probabilistic_Hough( int, void* );
    void draw_line( Mat, Point, Point );
};
    /**
     * @function main
     */
void Line_detection::hough_line_callback( const sensor_msgs::ImageConstPtr& image )
{

    // convert to cv image
    cv_bridge::CvImagePtr bridge;
    try
    {
        bridge = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to transform rgb image.");
        cout<<"error in subscribing image"<<endl;
        return;
    }


   if( bridge->image.empty() )
     { help();
       return;
     }

   /// Pass the image to gray
   cvtColor( bridge->image, src_gray, COLOR_RGB2GRAY );

   /// Apply Canny edge detector
   Canny( src_gray, edges, 50, 200, 3 );

   /// Create Trackbars for Thresholds
   char thresh_label[50];
   sprintf( thresh_label, "Thres: %d + input", min_threshold );

   namedWindow( probabilistic_name, WINDOW_AUTOSIZE );
   // createTrackbar( thresh_label, probabilistic_name, &p_trackbar, max_trackbar, Probabilistic_Hough);

   /// Initialize
   Probabilistic_Hough(0, 0);
   waitKey(1);
   return;
}

/**
 * @function help
 * @brief Indications of how to run this program and why is it for
 */
void Line_detection::help()
{
  printf("\t Hough Transform to detect lines \n ");
  printf("\t---------------------------------\n ");
  printf(" Usage: ./HoughLines_Demo <image_name> \n");
}


/**
 * @function Probabilistic_Hough
 */
void Line_detection::Probabilistic_Hough( int, void* ) // more efficient, gives the extremes of detected lines(x0,y0 and x1,y1), slightly better for implementation
{
  vector<Vec4i> p_lines;
  cvtColor( edges, probabilistic_hough, CV_GRAY2BGR );

  /// 2. Use Probabilistic Hough Transform
  HoughLinesP( edges, p_lines, 1, CV_PI/180, 155, 100, 50 );

  /// Show the result
  for( size_t i = 0; i < p_lines.size(); i++ )
     {
       Vec4i l = p_lines[i];
       double gradient;
       if (l[2] - l[0] == 0.0)
	{
	 gradient = 1000.0;
	}
       else
	{
	 gradient = (l[3] - l[1])/(l[2] - l[0]);
	}
       cout<<"l[0]="<<l[0]<<" l[1]="<<l[1]<<" l[2]="<<l[2]<<" l[3]="<<l[3]<<" gradient="<<gradient <<endl;
       line( probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
       rectangle( probabilistic_hough, Point(l[0]+10, l[1]+30), Point(l[2]-10, l[3]-30), Scalar(255, 255, 0), 1, 1);

     }

   imshow( probabilistic_name, probabilistic_hough );
   waitKey(1);
}



void Line_detection::draw_line(Mat img, Point start, Point end)
{
  int thickness = 2;
  int lineType = 8;
  line( img,
        start,
        end,
        Scalar(255,0,0),
        thickness,
        lineType);
}



int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "new_hough_node" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    Line_detection line_detection;
    
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_rect_color", 3, &Line_detection::hough_line_callback, &line_detection);

    ros::spin();
}
