/**
 * @file new_hough.cpp
 * @author: Jin Linhao
 * @for barrier detection, SMART FM IRG
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

/// Global variables

/** General variables */
Mat src, edges;
Mat src_gray;
Mat standard_hough, probabilistic_hough;
int min_threshold = 50;
int max_trackbar = 150;

const char* standard_name = "Standard Hough Lines Demo";
const char* probabilistic_name = "Probabilistic Hough Lines Demo";

int s_trackbar = max_trackbar;
int p_trackbar = max_trackbar;

/// Function Headers
void help();
void Probabilistic_Hough( int, void* );
void draw_line( Mat, Point, Point);

/**
 * @function main
 */
int main( int, char** )
{
   /// Read the image
   src = imread( "/home/eee/catkin_ws/src/SMART_ZED/data/pic/utown_barrier.jpeg", 1 );

   if( src.empty() )
     { help();
       return -1;
     }

   /// Pass the image to gray
   cvtColor( src, src_gray, COLOR_RGB2GRAY );

   /// Apply Canny edge detector
   Canny( src_gray, edges, 50, 200, 3 );

   /// Create Trackbars for Thresholds
   char thresh_label[50];
   sprintf( thresh_label, "Thres: %d + input", min_threshold );

   namedWindow( probabilistic_name, WINDOW_AUTOSIZE );
   // createTrackbar( thresh_label, probabilistic_name, &p_trackbar, max_trackbar, Probabilistic_Hough);

   /// Initialize
   Probabilistic_Hough(0, 0);
   waitKey(0);
   return 0;
}

/**
 * @function help
 * @brief Indications of how to run this program and why is it for
 */
void help()
{
  printf("\t Hough Transform to detect lines \n ");
  printf("\t---------------------------------\n ");
  printf(" Usage: ./HoughLines_Demo <image_name> \n");
}


/**
 * @function Probabilistic_Hough
 */
void Probabilistic_Hough( int, void* ) // more efficient, gives the extremes of detected lines(x0,y0 and x1,y1), slightly better for implementation
{
  vector<Vec4i> p_lines;
  cvtColor( edges, probabilistic_hough, CV_GRAY2BGR );

  /// 2. Use Probabilistic Hough Transform
  HoughLinesP( edges, p_lines, 1, CV_PI/180, 150, 30, 10 );

  /// Show the result
  for( size_t i = 0; i < p_lines.size(); i++ )
     {
       Vec4i l = p_lines[i];
       line( probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
       rectangle( probabilistic_hough, Point(l[0], l[1]+10), Point(l[2], l[3]-10), Scalar(255, 255, 0), 1, 1);
     }

   imshow( probabilistic_name, probabilistic_hough );
}



void draw_line(Mat img, Point start, Point end)
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


