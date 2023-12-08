#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "showpath");
    
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory", 1, true);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    nav_msgs::Path path;
    path.header.stamp = current_time;
    path.header.frame_id = "odom";

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double vx = 0.1;
    double vy = 0.0;
    double vth = 0.1;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        current_time = ros::Time::now();
        double dt = 0.1;
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) - vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::PoseStamped this_pose_stamped;//用于表示在三维空间中的位置和方向。
        this_pose_stamped.pose.position.x = x;
        this_pose_stamped.pose.position.y = y;
        
        tf::Quaternion tf_quat = tf::createQuaternionFromYaw(th);
        geometry_msgs::Quaternion goal_quat;
        tf::quaternionTFToMsg(tf_quat, goal_quat);
        //geometry_msgs::Quaternion goal_quat = tf::createQuaternionFromYaw(th);
        //geometry_msgs::Quaternion用于表示三维空间中的旋转信息
        //tf::createQuaternionFromYaw(th)将一个欧拉角th转换为一个四元数，该四元数可以用于描述三维空间中的旋转，th为弧度。
        
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        //使用 path.poses.push_back() 操作来构建一个路径消息，该路径消息包含了物体在不同时间点上的位置和姿态信息。
        path_pub.publish(path);
        ros::spinOnce();

        last_time = current_time;
        loop_rate.sleep();   
    }
    
    return 0;

}