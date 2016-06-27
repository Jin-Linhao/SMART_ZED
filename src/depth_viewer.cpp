#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <iomanip>

using namespace std;

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
    cv::Mat img(bridge->image.rows, bridge->image.cols, CV_8UC1);
    for(int i = 0; i < bridge->image.rows; i++)
    {
        float* Di = bridge->image.ptr<float>(i);
        char* Ii = img.ptr<char>(i);
        for(int j = 0; j < bridge->image.cols; j++)
        {   
            Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
        }   
    }

    // display

    cv::imshow(WINDOW_NAME, img);
    cv::waitKey(1);

    // cout << "img = "<< endl << " "  << img << endl << endl;
    // cout << "========================================" << endl << endl;
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "depth_viewer_node" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param("min_range", min_range_, 0.5);
    nh.param("max_range", max_range_, 5.5);

    cv::namedWindow(WINDOW_NAME);
    
    ros::Subscriber sub = n.subscribe("/camera/depth/image_rect_color", 3, &depthCb);
    ros::spin();
}


// source ~/git-completion.bash
// # colors!
// green="\[\033[0;32m\]"
// blue="\[\033[0;34m\]"
// purple="\[\033[0;35m\]"
// reset="\[\033[0m\]"
// # Change command prompt
// source ~/git-prompt.sh
// export GIT_PS1_SHOWDIRTYSTATE=1
// # '\u' adds the name of the current user to the prompt
// # '\$(__git_ps1)' adds git-related stuff
// # '\W' adds the name of the current directory
// export PS1="$purple\u$green\$(__git_ps1)$blue \W $ $reset"
// alias ls="ls --color=auto"
// alias ll="ls -al --color=auto"
// alias ck="catkin_make"
// alias gst="git status"
// alias ros="source ~/myros.sh"
// alias rosr="source ~/myrosr.sh"
// alias ssh148="ssh ubuntu@192.168.1.148"
// alias ssh133="ssh ubuntu@192.168.1.133"
// alias freshtime="sudo ntpdate time time.nsit.gov"
// export EDITOR='nano'

// source /opt/ros/indigo/setup.bash
// #source ~/myrosr.sh
// source ~/catkin_ws/devel/setup.bash