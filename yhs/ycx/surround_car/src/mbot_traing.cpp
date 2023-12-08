//给两个追捕小车速度
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"mbot_traing");
  ros::NodeHandle n;

  //发布类型位速度类型的速度发布器
    ros::Publisher mbot2_vel=n.advertise<geometry_msgs::Twist>("mbot2/cmd_vel",10);
    ros::Publisher mbot3_vel=n.advertise<geometry_msgs::Twist>("mbot3/cmd_vel",10);

  tf::TransformListener listener2;//创建tf监听器/
  tf::TransformListener listener3;
  ros::Rate rate(10.0);
  while (n.ok())
  {
    tf::StampedTransform transform2;
    tf::StampedTransform transform3;
    try
    {
      //查找mbot2与carrot2的坐标转换/****************************/
      listener2.waitForTransform("/mbot2_base_footprint","/carrot1",ros::Time(0),ros::Duration(3.0));
      listener2.lookupTransform("/mbot2_base_footprint","/carrot1",ros::Time(0),transform2);
      //查找mbot3与carrot2的坐标转换/****************************/
      listener3.waitForTransform("/mbot3_base_footprint","/carrot2",ros::Time(0),ros::Duration(3.0));
      listener3.lookupTransform("/mbot3_base_footprint","/carrot2",ros::Time(0),transform3);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    geometry_msgs::Twist vel_msg2;//速度消息
    geometry_msgs::Twist vel_msg3;

    vel_msg2.angular.z=-2*atan2(transform2.getOrigin().y(),transform2.getOrigin().x());
    vel_msg2.linear.x=0.8*sqrt(pow(transform2.getOrigin().x(),2)+pow(transform2.getOrigin().y(),2));

    vel_msg3.angular.z=-2*atan2(transform3.getOrigin().y(),transform3.getOrigin().x());
    vel_msg3.linear.x=0.8*sqrt(pow(transform3.getOrigin().x(),2)+pow(transform3.getOrigin().y(),2));
    
    mbot2_vel.publish(vel_msg2);//发布速度消息
    mbot3_vel.publish(vel_msg3);
    rate.sleep();
  }
  return 0;
}
