#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
bool bCaptrueOneFrame = true;
 
void callbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
    if(bCaptrueOneFrame == true)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imwrite("/home/dxs/image.jpg",cv_ptr->image);
        ROS_WARN("captrue image");
        bCaptrueOneFrame = false;
    } 
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "image_node");
    ROS_WARN("image_node start");
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/camera/image_raw",1,callbackRGB);
    ros::spin();
    return 0;
}

