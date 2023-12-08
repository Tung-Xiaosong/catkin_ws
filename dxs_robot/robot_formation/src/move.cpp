#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_node");
	ros::NodeHandle nh;

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("robot3/cmd_vel", 10);

	// 创建tf的监听器
	static tf::TransformListener listener;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		tf::StampedTransform transform;
		try
		{
			//tf::TransformListener类的waitForTransform函数，等待发布的"robot3/base_link"到"carrot"的tf变换关系。
			listener.waitForTransform("robot3/base_link","carrot",ros::Time(0),ros::Duration(3.0));
			//请求发布的"robot3/base_link"到"carrot"的tf变换关系，并将其存储为一个tf2_ros::StampedTransform类型的变量transform
			listener.lookupTransform("robot3/base_link","carrot",ros::Time(0),transform);
		}
		catch(const std::exception& e)
		{
			ROS_WARN("Error!");
			ROS_ERROR("%s",e.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		
		geometry_msgs::Twist vel_msg;

		vel_msg.linear.x = 0.8*sqrt(pow(transform.getOrigin().x(),2) + pow(transform.getOrigin().y(),2));
		vel_msg.angular.z = -2*atan2(transform.getOrigin().x(),transform.getOrigin().y());
		
		ROS_INFO("linear.x = %f , angular.z = %f ",vel_msg.linear.x,vel_msg.angular.z);
		vel_pub.publish(vel_msg);
		loop_rate.sleep();
	}
	return 0;
}

