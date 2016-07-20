#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <std_msgs/Int32MultiArray.h>


using namespace std;
using namespace cv;

class Depth_viewer
{ 
 public:                                                      //public functions
	Depth_viewer()                                         //declare a constructor, same name as the class name.

		{
		 min_range_ = 0.5;
		 max_range_ = 20.5; 

         line_vector[0] = 230;
         line_vector[1] = 50;
         line_vector[2] = 330;
         line_vector[3] = 450;
         thresholding_value = 130;
		}

    //member function declaration
	Mat  img_filter(Mat);
	void depth_callback(const sensor_msgs::ImageConstPtr&);
    void hough_line_callback(const std_msgs::Int32MultiArray&);
	void image_show(Mat, Mat, Mat, Mat, Mat);
    Mat  horizontal_line_detection(Mat);
    Mat  vertical_line_detection(Mat);
    void rot90(Mat &, int);
    Mat  getROI(Mat);
    void identify(Mat, Mat);
    int  check_ROI();
    // void detection();
    // void segmentation(Mat, Mat);
    // void laplacian(Mat);

    //global variables
    Mat   img_threshold,  img_blur,     filt_img,       img_fast;
    Mat   horizontal_img, vertical_img, horizontal_seg, vertical_seg;
    float min_range_,     max_range_,   gradient,       mean_dis; 
    Vec4i line_vector;
    int   x1, y1, x2, y2, thresholding_value;;
    // Mat depth_img;
};



Mat Depth_viewer::img_filter(Mat img_origin)                                               //member function definition
{
    Mat thresholded_image;
	// Mat element_large = getStructuringElement(1, Size(11,11), Point(6,6)),
 //        element_medium = getStructuringElement(1, Size(9,9), Point(4,4)),
 //        element_small = getStructuringElement(1, Size(5,5), Point(2,2)),
 //        element_rect = getStructuringElement(0, Size(5,19), Point(-1, -1));

    threshold(img_origin, thresholded_image, thresholding_value, 255, 0);
	return(thresholded_image);
}



Mat Depth_viewer::horizontal_line_detection(Mat bw)
{
    Mat edges, smooth, kernel = Mat::ones(2, 2, CV_8UC1);
    Mat horizontal = bw.clone();
    int horizontalsize = horizontal.cols / 40;
    Mat horizontalStructure = getStructuringElement(MORPH_RECT, Size(horizontalsize,2));

    erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
    dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));
    adaptiveThreshold(horizontal, edges, 255, CV_ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 3, -2);
    dilate(edges, edges, kernel);

    horizontal.copyTo(smooth);

    blur(smooth, smooth, Size(2, 2));
    smooth.copyTo(horizontal, edges);
    // imshow("smooth", vertical);

    waitKey(1);
    return horizontal;
}



Mat Depth_viewer::vertical_line_detection(Mat bw)
{

    Mat edges, smooth, kernel = Mat::ones(2, 2, CV_8UC1);
    Mat vertical = bw.clone();
    int verticalsize = vertical.rows / 40;
    Mat verticalStructure = getStructuringElement(MORPH_RECT, Size( 2,verticalsize));

    erode(vertical, vertical, verticalStructure, Point(-1, -1));
    dilate(vertical, vertical, verticalStructure, Point(-1, -1));
    adaptiveThreshold(vertical, edges, 255, CV_ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 3, -2);
    dilate(edges, edges, kernel);

    vertical.copyTo(smooth);

    blur(smooth, smooth, Size(2, 2));
    smooth.copyTo(vertical, edges);

    waitKey(1);
    return vertical;
}



void Depth_viewer::rot90(Mat &matImage, int rotflag)
{
  //1=CW, 2=CCW, 3=180
  if (rotflag == 1){
    transpose(matImage, matImage);  
    flip(matImage, matImage,1);     //transpose+flip(1)=CW
  } 
  else if (rotflag == 2) 
  {
    transpose(matImage, matImage);  
    flip(matImage, matImage,0);     //transpose+flip(0)=CCW     
  } 
  else if (rotflag ==3)
  {
    flip(matImage, matImage,-1);    //flip(-1)=180          
  } 
  else if (rotflag != 0)
  {                                 //if not 0,1,2,3:
    cout  << "Unknown rotation flag(" << rotflag << ")" << endl;
  }
  //to keep original image, using cv::Mat matRotated = matImage.clone();
}



Mat Depth_viewer::getROI(Mat src)
{
    Mat ROI;

    if (line_vector[0] <= line_vector[2] && line_vector[1] <= line_vector[3])
	{
	        x1 = line_vector[0] - 5, y1 = line_vector[1] - 5,  //x-start, y-start
		    x2 = line_vector[2] + 5, y2 = line_vector[3] + 5;  //x-end, y-end
    }
    else
    {
            x1 = line_vector[0] - 5, y1 = line_vector[3] - 5, //x-start, y-start
            x2 = line_vector[2] + 5, y2 = line_vector[1] + 5;
    }               
    x1, y1, x2, y2 = check_ROI();
    int width = x2 - x1,  
	    height = y2 - y1;

	    ROI = src(Rect(x1, y1, width, height));

    return ROI;
}



int Depth_viewer::check_ROI()
{
    if (x1 < 0)
    {
        x1 = 0;
    }
    else if (y1 < 0)
    {
        y1 = 0;
    }
    else if (x1 < 0 && y1 < 0)
    {
        x1 = y1 = 0;
    }
    else if (x2 > 640)
    {
        x2 > 640;
    }
    else if (y2 > 480)
    {
        y2 = 480;
    }
    else if (x2 > 640 && y2 > 480)
    {
        x2 = 640, y2 = 480;
    }

    return (x1, y1, x2, y2);
}



void Depth_viewer::identify(Mat horizontal, Mat vertical)
{
    float horizontal_num = countNonZero(horizontal), 
          vertical_num = countNonZero(vertical),
          ratio = horizontal_num/(horizontal_num + vertical_num +1);
    int   area  = (x2 - x1)*(y2 - y1)*0.0025;
    // cout <<"area: " << area <<endl;

    if (vertical_num >= area)
    {
        if (ratio >= 0.9)
        {
            cout << "Barrier is down"<< endl;
            cout << "gradient is "<< gradient << endl;
        }
        else if (ratio <= 0.3)
        {
            cout <<  "Barrier is upright" << endl;
            cout << "gradient is "<< gradient << endl;
        }
        else
        {
            cout << "The Barrier is moving" << endl;
            cout << "gradient is "<< gradient << endl;
        }
    }
    else
    {
        cout << "No barrier detected" << endl;
    }
}



void Depth_viewer::hough_line_callback(const std_msgs::Int32MultiArray& p_lines_array)
{
    for( size_t i = 0; i < p_lines_array.data.size(); i++ )
    {

        line_vector[i] = p_lines_array.data[i];//should not be removed
    }
//     float gradient;
    if (x1 - x2 == 0.0)
    {
     gradient = 1000000.0;
    }
       else
    {
     gradient = (double)(y2 - y1)/(x2 - x1);
    }
    cout << "x1 = " << x1 << " y1 = " << y1 << " x2 = " << x2 << " y2 = " << y2 << endl;
    cout << "gradient is "<< gradient << endl;
}
 

void Depth_viewer::depth_callback(const sensor_msgs::ImageConstPtr& image)         //?????
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

    Mat     depth_img_msg(bridge->image.rows, bridge->image.cols, CV_8UC1),
            depth_img_msg2(bridge->image.rows, bridge->image.cols, CV_32FC1);
    float   barrier_height = 1.1, camera_height = 1.5; //110cm and 150cm
    float   total_dis = 0;
    int     total_dis_num = 0;

    for(int i = 0; i < bridge->image.rows; i++)
    {
        float*          Di = bridge->image.ptr<float>(i);//get ith row of original image, index Di
        unsigned char*  Ii = depth_img_msg.ptr<unsigned char>(i);

        for(int j = 0; j < bridge->image.cols; j++)
        {   
            if(Di[j]<min_range_)
            {
                Di[j] = max_range_ ;
            }
           
            Ii[j] = (unsigned char)(255*((Di[j] - min_range_)/(max_range_-min_range_))); 
            Ii[j] = (unsigned char)(255-Ii[j]);
        }   
    }

    for (int i = y1; i < (y2); i++)
    {
        float* Di = bridge->image.ptr<float>(i);//get ith row of original image, index Di
        float* Ii = depth_img_msg2.ptr<float>(i);
        for(int j = x1; j < (x2); j++)
        {
            if ( Di[j] > 0 && Di[j] < 10.0) 
            {
                total_dis = total_dis + Di[j];
                total_dis_num  = total_dis_num + 1;
            }
        }
    }
 
    mean_dis = total_dis/(float)total_dis_num;
    float mean_horizontal_dis = sqrt(pow(mean_dis, 2) - pow((camera_height - barrier_height), 2));
    
    if (mean_horizontal_dis <= 8)
    {
        cout << "The average distance is "<< mean_horizontal_dis << endl; 
        cout << "The total distance number is "<< total_dis_num << endl;        
    }
     
    else
    {
        cout << "The barrier is either too far or NaN" << endl;
    }     


        filt_img = Depth_viewer::img_filter(depth_img_msg);
        //Depth_viewer::rot90(filt_img, 2);
        horizontal_img = Depth_viewer::horizontal_line_detection(filt_img);
        vertical_img = Depth_viewer::vertical_line_detection(filt_img);
        // Depth_viewer::rot90(img, 2);
        // img  = Depth_viewer::getROI(img);
        // Depth_viewer::rot90(vertical_img, 2);
        // Depth_viewer::rot90(horizontal_img, 2);   
        vertical_seg  = Depth_viewer::getROI(vertical_img);
        horizontal_seg = Depth_viewer::getROI(horizontal_img);
        Depth_viewer::image_show(filt_img, vertical_seg, horizontal_seg, horizontal_img, vertical_img);
        Depth_viewer::identify(vertical_seg, horizontal_seg);
}
    // Depth_viewer::segmentation(filt_img, img);
    // Depth_viewer::laplacian(filt_img);
    // return (img);



void Depth_viewer::image_show(Mat img_show, Mat horizontal_seg, Mat vertical_seg, Mat horizontal, Mat vertical)
{
    if (!img_show.empty() && !horizontal_seg.empty() && !vertical_seg.empty())
	{
    	line( img_show, Point(line_vector[0], line_vector[1]), Point(line_vector[2], line_vector[3]), Scalar(0,0,255), 3, CV_AA);
    	rectangle( img_show, Point(x1, y1), Point(x2, y2), Scalar(255, 255, 0), 1, 1);

    	imshow("image_proc", img_show);
    	imshow("horizontal_seg", horizontal_seg);
    	imshow("vertical_seg", vertical_seg);
    	// imshow("horizontal", horizontal);    
    	// imshow("vertical", vertical);
	}
	waitKey(1);
}




int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "depth_viewer_node" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    Depth_viewer depth_viewer;
    // Hough_line hough_line;

    ros::Subscriber depth_sub = n.subscribe("/camera/depth/image_rect_color", 3, &Depth_viewer::depth_callback, &depth_viewer);
    ros::Subscriber hough_line_sub = n.subscribe("/hough_line", 3, &Depth_viewer::hough_line_callback, &depth_viewer);

    //depth_viewer.detection();
    ros::spin();
}







// void Depth_viewer::laplacian(Mat img)
// {
//     Mat kernel = (Mat_<float>(3,3) << 
//                     9,  9,  9,
//                     9, -72,  9,
//                     9,  9,  9);
//     Mat imgLaplacian;
//     Mat sharp = img;

//     filter2D(sharp, imgLaplacian, CV_32F, kernel);
//     img.convertTo(sharp, CV_32F);
//     Mat imgResult = sharp - imgLaplacian;
//     // convert back to 8bits gray scale
//     imgResult.convertTo(imgResult, CV_8UC3);
//     imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
//     // imshow( "Laplace Filtered Image", imgLaplacian );
//     imshow( "New Sharped Image", imgResult);
//     return;
// }

// void Depth_viewer::segmentation(Mat bw, Mat src)
// {
//     Mat dist;
//     distanceTransform(bw, dist, CV_DIST_L2, 3);
//     // Normalize the distance image for range = {0.0, 1.0}
//     // so we can visualize and threshold it
//     normalize(dist, dist, 0, 1., NORM_MINMAX);
//     imshow("Distance Transform Image", dist);
//     // Threshold to obtain the peaks
//     // This will be the markers for the foreground objects
//     threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
//     // Dilate a bit the dist image
//     Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
//     dilate(dist, dist, kernel1);
//     imshow("Peaks", dist);
//     // Create the CV_8U version of the distance image
//     // It is needed for findContours()
//     Mat dist_8u;
//     dist.convertTo(dist_8u, CV_8U);
//     // Find total markers
//     vector<vector<Point> > contours;
//     findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//     // Create the marker image for the watershed algorithm
//     Mat markers = Mat::zeros(dist.size(), CV_32SC1);
//     // Draw the foreground markers
//     for (size_t i = 0; i < contours.size(); i++)
//         drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);
//     // Draw the background marker
//     circle(markers, Point(5,5), 3, CV_RGB(255,255,255), -1);
//     imshow("Markers", markers*10000);
//     // Perform the watershed algorithm
//     watershed(src, markers);
//     Mat mark = Mat::zeros(markers.size(), CV_8UC1);
//     markers.convertTo(mark, CV_8UC1);
//     bitwise_not(mark, mark);
// //    imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
//                                   // image looks like at that point
//     // Generate random colors
//     vector<Vec3b> colors;
//     for (size_t i = 0; i < contours.size(); i++)
//     {
//         int b = theRNG().uniform(0, 255);
//         int g = theRNG().uniform(0, 255);
//         int r = theRNG().uniform(0, 255);
//         colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
//     }
//     // Create the result image
//     Mat dst = Mat::zeros(markers.size(), CV_8UC3);
//     // Fill labeled objects with random colors
//     for (int i = 0; i < markers.rows; i++)
//     {
//         for (int j = 0; j < markers.cols; j++)
//         {
//             int index = markers.at<int>(i,j);
//             if (index > 0 && index <= static_cast<int>(contours.size()))
//                 dst.at<Vec3b>(i,j) = colors[index-1];
//             else
//                 dst.at<Vec3b>(i,j) = Vec3b(0,0,0);
//         }
//     }
//     // Visualize the final image
//     imshow("Final Result", dst);
//     waitKey(0);
//     return;
// }
