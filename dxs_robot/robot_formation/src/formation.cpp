#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
using namespace std;

int main(int argc,char** argv)
{
    //初始化节点
    ros::init(argc,argv,"formation");
    ros::NodeHandle n;

    //创建一个tf发布对象
    static tf::TransformBroadcaster broadcaster;

    //构造TransformStamped消息,TransformStamped用于表示坐标系之间的刚体变换
	  geometry_msgs::TransformStamped trans;
    trans.header.frame_id = "robot1/imu_link";
    trans.child_frame_id = "carrot";
    trans.transform.translation.x = -2;
    trans.transform.translation.y = 0;
    trans.transform.translation.z = 0;
    tf::Quaternion q;//创建四元数对象q,tf::Quaternion(double x, double y, double z, double w)
    q.setRPY(0,0,0);//setRPY函数可以将一个旋转角度向量转换为四元数
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      trans.header.stamp = ros::Time::now();
      broadcaster.sendTransform(trans);//发布TransformStamped消息
      //ROS_INFO("x = %f , y = %f , z = %f",trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z);
      loop_rate.sleep();
    }
    return 0;
}
