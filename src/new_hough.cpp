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
#include <std_msgs/Int32MultiArray.h>
#include <cmath>

using namespace cv;
using namespace std;

/// Global variables

/** General variables */

class Line_detection
{
  public:

    const char* probabilistic_name = "Probabilistic Hough Lines Demo";

    //function header
    void hough_line_callback( const sensor_msgs::ImageConstPtr& image );
    void help();
    void draw_line( Mat, Point, Point );
    void publish_vector();
    void image_show();
    double cal_Euclidean(int, int, int, int);


    //ros header
    ros::Publisher  pub;
    ros::Subscriber sub; 

    Mat probabilistic_hough;
    Mat src, edges;
    Mat src_gray, edges_rgb;
    vector<Vec4i> p_lines;
    //Vec4i line_vector = p_lines[i];
};





void Line_detection::hough_line_callback( const sensor_msgs::ImageConstPtr& image )
{

    //Mat probabilistic_hough;

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

   imshow("rgb", bridge->image);
   waitKey(1);
   /// Pass the image to gray
   cvtColor( bridge->image, probabilistic_hough, COLOR_RGB2GRAY );

   /// Apply Canny edge detector
   Canny( probabilistic_hough, edges, 50, 200, 3 );

   // vector<Vec4i> p_lines;
   HoughLinesP( edges, p_lines, 1, CV_PI/180, 155, 150, 10 );
   //cout << edges.size() <<endl;
   Line_detection::publish_vector();

   return;
}




void Line_detection::help()
{
  printf("\t Hough Transform to detect lines \n ");
  printf("\t---------------------------------\n ");
  printf("\t Usage: roslaunch zed_smart hough_line.launch\n");
  printf(" rostopic echo /hough_line\n");
}



double Line_detection::cal_Euclidean(int x1, int y1, int x2, int y2)
{
  double dis_Euclidean;
  dis_Euclidean = sqrt(pow((x2 - x1),2.0)+pow((y2 - y1), 2.0));
  return dis_Euclidean;
}



void Line_detection::publish_vector()
{
  /// Show the result
  double max_dis;
  Vec4i max_line_vector;
  for( size_t i = 0; i < p_lines.size(); i++ )
     {
  	// cout<<"p_lines size"<<p_lines.size()<<endl;	
        Vec4i line_vector = p_lines[i];

        double line_dis = Line_detection::cal_Euclidean(line_vector[0], line_vector[1], line_vector[2], line_vector[3]);
        if (line_dis >= max_dis)
        {
          max_dis = line_dis;
          max_line_vector = line_vector;
        }
      }

	
 //       double gradient;
 //       if (l[2] - l[0] == 0.0)
	// {
	//  gradient = 1000.0;
	// }
 //       else
	// {
	//  gradient = (l[3] - l[1])/(l[2] - l[0]);
	// }
   // cout<<"l[0]="<<line_vector[0]<<" l[1]="<<line_vector[1]<<" l[2]="<<line_vector[2]<<" l[3]="<<line_vector[3]<<endl;
     cvtColor( edges, edges_rgb, COLOR_GRAY2RGB );
     line(edges_rgb, Point(max_line_vector[0], max_line_vector[1]), Point(max_line_vector[2], max_line_vector[3]), Scalar(0,0,255), 3, CV_AA);
     // rectangle(edges_rgb, Point(max_line_vector[0], max_line_vector[1]), Point(max_line_vector[2], max_line_vector[3]), Scalar(255, 255, 0), 1, 1);
     //good rectangle
     rectangle(edges_rgb, Point(max_line_vector[0]-30, max_line_vector[1]+30), Point(max_line_vector[2]+30, max_line_vector[3]-30), Scalar(255, 255, 0), 1, 1);
     
     // cout << edges_rgb.size() << endl;
     imshow( probabilistic_name, edges_rgb );
     waitKey(1);
      if (max_dis != 0)
      {

           std_msgs::Int32MultiArray p_lines_array;
           p_lines_array.data.push_back(max_line_vector[0]);
           p_lines_array.data.push_back(max_line_vector[1]);
           p_lines_array.data.push_back(max_line_vector[2]);
           p_lines_array.data.push_back(max_line_vector[3]);

           pub.publish(p_lines_array);
     }
}


void Line_detection::image_show()
{
   // line( probabilistic_hough, Point(line_vector[0], line_vector[1]), Point(line_vector[2], line_vector[3]), Scalar(0,0,255), 3, CV_AA);
    //rectangle( probabilistic_hough, Point(line_vector[0]+10, line_vector[1]+30), Point(line_vector[2]-10, line_vector[3]-30), Scalar(255, 255, 0), 1, 1);

    //waitKey(1);
}




int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "new_hough_node" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    Line_detection line_detection;

    line_detection.pub = n.advertise<std_msgs::Int32MultiArray>("hough_line",100);
    line_detection.sub = n.subscribe("/camera/rgb/image_rect_color", 100, &Line_detection::hough_line_callback, &line_detection);

    ros::spin();
}



/**
 * @function Probabilistic_Hough
 */

// Vec4i Line_detection::Probabilistic_Hough( Mat edges ) // more efficient, gives the extremes of detected lines(x0,y0 and x1,y1), slightly better for implementation
// {
//   vector<Vec4i> p_lines;
  // cvtColor( edges, probabilistic_hough, CV_GRAY2BGR );

  /// 2. Use Probabilistic Hough Transform
  // HoughLinesP( edges, p_lines, 1, CV_PI/180, 155, 150, 10 ); //HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 ); 
                                                              // dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
                                                              // lines: A vector that will store the parameters (x_{start}, y_{start}, x_{end}, y_{end}) of the detected lines
                                                              // rho : The resolution of the parameter r in pixels. We use 1 pixel.
                                                              // theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
                                                              // threshold: The minimum number of intersections to “detect” a line
                                                              // minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
                                                              // maxLineGap: The maximum gap between two points to be considered in the same line.
// }
//   return p_lines;



//void Line_detection::draw_line(Mat img, Point start, Point end)
//{
//  int thickness = 2;
//  int lineType = 8;
//  line( img,
//        start,
//        end,
//        Scalar(255,0,0),
//        thickness,
//        lineType);
//}

