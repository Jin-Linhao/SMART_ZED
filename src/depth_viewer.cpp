#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <iomanip>
#include <deque>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace cv;

class Depth_viewer
{ 
 public:                                                      //public functions
	Depth_viewer()                                       //declare a constructor, same name as the class name.

		{
		 min_range_ = 0.5;
		 max_range_ = 20.5; 

         line_vector[0] = 400;
         line_vector[1] = 50;
         line_vector[2] = 480;
         line_vector[3] = 450;

         x1 = line_vector[0] ;
         y1 = line_vector[1];
         x2 = line_vector[2];
         y2 = line_vector[3];

         thresholding_value = 130;
         mean_horizontal_dis = 0.0;
		}

    //member function declaration
	Mat  img_filter(Mat);
	void depth_callback(const sensor_msgs::ImageConstPtr&);
    void hough_line_callback(const std_msgs::Int32MultiArray&);
	void image_show(Mat, Mat, Mat);
    Mat  horizontal_line_detection(Mat);
    Mat  vertical_line_detection(Mat);
    void rot90(Mat &, int);
    Mat  getROI(Mat);
    void identify(Mat, Mat);
    int  check_ROI();
    void Pub_flag();
    void Pub_dis();
    // void detection();
    // void segmentation(Mat, Mat);
    // void laplacian(Mat);

    //global variables
    Mat   img_threshold,  img_blur,     filt_img,       img_fast;
    Mat   horizontal_img, vertical_img, horizontal_seg, vertical_seg;
    float min_range_,     max_range_,   gradient,       mean_dis,       mean_horizontal_dis; 
    Vec4i line_vector;
    int   x1, y1, x2, y2, thresholding_value;;
    // Mat depth_img;

    std::deque<int> flag_status;

    //ros functions
    ros::Subscriber depth_sub;
    ros::Subscriber hough_line_sub;
    ros::Publisher  pub_flag;
    ros::Publisher  pub_dis;


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

    if (line_vector[0] <= line_vector[2] )
    {
            x1 = line_vector[0] - 5,  x2 = line_vector[2] + 5;          
    }
    else
    {
            x1 = line_vector[2] - 5,  x2 = line_vector[0] + 5;
    }
    if (line_vector[1] <= line_vector[3] )
    {
            y1 = line_vector[1] - 5,  y2 = line_vector[3] + 5;
    }
    else
    {
            y1 = line_vector[3] - 5,  y2 = line_vector[1] + 5;
    }
    // cout << "x1:" << x1 << " y1P:" << y1 << " x2:" << x2 << " y2:" << y2 << endl;               
    Depth_viewer::check_ROI();
    int width = x2 - x1,  
        height = y2 - y1;

    ROI = src(Rect(x1, y1, width, height));

    return ROI;
}



int Depth_viewer::check_ROI()
{
    x1 = x1 < 0 ? 0 : x1;          //tri-arguement operator
    x2 = x2 > 639 ? 639 : x2;

    y1 = y1 < 0 ? 0 : y1;
    y2 = y2 > 479 ? 479 : y2;
}


void Depth_viewer::identify(Mat horizontal, Mat vertical)
{
    float horizontal_num = countNonZero(horizontal), 
          vertical_num = countNonZero(vertical),
          ratio = horizontal_num/(horizontal_num + vertical_num);
    int   area  = (x2 - x1)*(y2 - y1)*0.1;
    // cout <<"area: " << area <<endl;
    // cout <<"horizontal_num: " << horizontal_num <<endl;

    if (horizontal_num >= area)
    {
        if (ratio >= 0.9 && gradient > 5)
        {
            cout << "Barrier is down"<< endl;
            flag_status.push_back(-1);
            // cout << "gradient is "<< gradient << endl;
        }
    }
    else if (ratio < 0.9 && ratio > 0.3 && gradient > 0.2 && gradient < 5)
    {
        cout <<  "Barrier is moving" << endl;
        flag_status.push_back(0);
        // cout << "gradient is "<< gradient << endl;
    }
    else if (ratio < 0.3 && gradient < 0.2)
    {
        cout << "The Barrier is up" << endl;
        flag_status.push_back(1);
        // cout << "gradient is "<< gradient << endl;
    }
    else
    {
        cout << "undetermined situation" << endl;
        flag_status.push_back(0);
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
    // cout << "x1 = " << x1 << " y1 = " << y1 << " x2 = " << x2 << " y2 = " << y2 << endl;
    // cout << "gradient is "<< gradient << endl;
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
           
            Ii[j] = (unsigned char)(255*((Di[j] - min_range_)/(max_range_ - min_range_))); 
            Ii[j] = (unsigned char)(255 - Ii[j]);
        }   
    }

    // cout<<"rows:"<<bridge->image.rows<<" cols:"<<bridge->image.cols<<endl;
    // cout<<"y1: "<<y1<<" y2:"<<y2<<" x1: "<<x1<<" x2: "<<x2<<endl;
    for (int i = y1; i < (y2); i++)
    {
        float* Di = bridge -> image.ptr<float>(i);//get ith row of original image, index Di
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
    mean_horizontal_dis = sqrt(pow(mean_dis, 2) - pow((camera_height - barrier_height), 2));
    std_msgs::Float64 g_dis;

    if (mean_horizontal_dis <= 8)
    {
        cout << "The average distance is "<< mean_horizontal_dis << endl;
        g_dis.data = mean_horizontal_dis;
        pub_dis.publish(g_dis);
        // cout << "The gradient is " << gradient <<endl;
        cout << endl;
        // cout << "The total distance number is "<< total_dis_num << endl;        
    } 
    else
    {
        cout << "The barrier is either too far or NaN" << endl;
        g_dis.data = 20.0;
        pub_dis.publish(g_dis);
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
        Depth_viewer::image_show(filt_img, vertical_seg, horizontal_seg);
        Depth_viewer::identify(vertical_seg, horizontal_seg);
        Depth_viewer::Pub_flag();
}
    // Depth_viewer::segmentation(filt_img, img);
    // Depth_viewer::laplacian(filt_img);
    // return (img);



void Depth_viewer::image_show(Mat img_show, Mat horizontal_seg, Mat vertical_seg)
{
    if (!img_show.empty() && !horizontal_seg.empty() && !vertical_seg.empty())
	{
    	line( img_show, Point(line_vector[0], line_vector[1]), Point(line_vector[2], line_vector[3]), Scalar(0,0,255), 3, CV_AA);
    	rectangle( img_show, Point(x1, y1), Point(x2, y2), Scalar(255, 255, 0), 1, 1);

    	imshow("image_proc", img_show);
    	imshow("horizontal_seg", horizontal_seg);
    	imshow("vertical_seg", vertical_seg);

	}
	waitKey(1);
}



void Depth_viewer::Pub_flag()
{
    std_msgs::Int64 g_flag;
    g_flag.data = -1;
    float total_status = 0;
    // pub_flag.publish(g_flag);


    cout << "size of the array is " << flag_status.size() << endl;
    if (flag_status.size() > 29)
    {
        flag_status.pop_front();
        // cout << flag_status.at(28) << endl;
        for (int i=0; i<flag_status.size(); i++)
        {
            cout << flag_status.at(i) << " "; 
            total_status = total_status + flag_status.at(i);

        }
        cout << endl;
        cout << "the total_status" << total_status << endl;
        if (total_status <= 5)
        {
            g_flag.data = 0;
            pub_flag.publish(g_flag);
        }
        else if (total_status > 5)
        {
            g_flag.data = 1;
            pub_flag.publish(g_flag);
        }
        else
        {
            cout << "cannot determine action" << endl;
            g_flag.data = 0;
            pub_flag.publish(g_flag);
        }
    }
}



void Depth_viewer::Pub_dis()
{
    // std_msgs::Float64 g_dis;
    // g_dis.data = mean_horizontal_dis;
    // pub_dis.publish(g_dis);
}



int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "depth_viewer_node" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    Depth_viewer depth_viewer;
    // Hough_line hough_line;

    depth_viewer.depth_sub = n.subscribe("/camera/depth/image_rect_color", 3, &Depth_viewer::depth_callback, &depth_viewer);
    depth_viewer.hough_line_sub = n.subscribe("/hough_line", 3, &Depth_viewer::hough_line_callback, &depth_viewer);
    depth_viewer.pub_flag = n.advertise<std_msgs::Int64>("gantry_flag",100);
    depth_viewer.pub_dis = n.advertise<std_msgs::Float64>("gantry_dis",100);

    //depth_viewer.detection();

    ros::spin();
}
