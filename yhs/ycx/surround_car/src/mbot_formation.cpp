//在目标小车mbot4下创建固定坐标系carrot1,2
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
using namespace std;

int main(int argc,char** argv)
{
    //初始化节点
    ros::init(argc,argv,"mbot_formation");
    ros::NodeHandle n;

    tf2_ros::TransformBroadcaster br1;
    tf2_ros::TransformBroadcaster br2;

  //  tf::Transform transform;//定义一个tf关系
    geometry_msgs::TransformStamped trans1;
    geometry_msgs::TransformStamped trans2;
        geometry_msgs::TransformStamped trans3;

    trans1.header.frame_id="laser_link";
    //trans1.child_frame_id="laser_link";
    trans1.child_frame_id="carrot1";
    trans1.transform.translation.x=1;/*-0.866*///相对与laser_link的坐标
    trans1.transform.translation.y=-1;/*0.5*/
    trans1.transform.translation.z=0;
    tf2::Quaternion q1;//定义四元数
      q1.setRPY(0,0,0);//用rpy初始化四元数0.25
    trans1.transform.rotation.x=q1.x();//设置旋转变换
    trans1.transform.rotation.y=q1.y();
    trans1.transform.rotation.z=q1.z();
    trans1.transform.rotation.w=q1.w();

    trans2.header.frame_id="laser_link";
    trans2.child_frame_id="carrot2";
    trans2.transform.translation.x=1;/*-0.866*/
    trans2.transform.translation.y=1;/*-0.5*/
    trans2.transform.translation.z=0;
    tf2::Quaternion q2;//定义四元数
      q2.setRPY(0,0,0);//用rpy初始化四元数-0.52
    trans2.transform.rotation.x=q2.x();//设置旋转变换
    trans2.transform.rotation.y=q2.y();
    trans2.transform.rotation.z=q2.z();
    trans2.transform.rotation.w=q2.w();
    
    ros::Rate rate(10);
    while (ros::ok())
    {
      trans1.header.stamp=ros::Time::now();
      br1.sendTransform(trans1);
      trans2.header.stamp=ros::Time::now();
      br2.sendTransform(trans2);
      rate.sleep();
    }

    return 0;
}
