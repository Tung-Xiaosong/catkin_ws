#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>

int main(int argc ,char **argv)
{
    ros::init(argc ,argv ,"pub_path_node");
    ros::NodeHandle nh;
    
    ros::Publisher pub_path;
    pub_path = nh.advertise<nav_msgs::Path>("Path_", 10);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        nav_msgs::Path path_;
        path_.header.stamp = ros::Time::now();
        path_.header.frame_id = "map";

        for(int i = 0; i < 5; i++)
        {
            geometry_msgs::PoseStamped pose_;
            pose_.header.stamp = ros::Time::now();
            pose_.header.frame_id = "map";
            pose_.pose.position.x = -1.0;
            pose_.pose.position.y = i;
            pose_.pose.orientation.w = 1.0;

            path_.poses.push_back(pose_);
        }
        pub_path.publish(path_);
        ROS_INFO("Path have published !!!");

        loop_rate.sleep();
    }
    return 0;
}