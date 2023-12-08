#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>

void pathCallBack(const nav_msgs::Path::ConstPtr& path_msgs)
{
    ROS_INFO("Received path with %zu pose!!!", path_msgs->poses.size());
    //%zu 用于输出 size_t 类型的变量。size_t 是一种用于表示对象大小或元素数量的无符号整数类型。通常用于数组、容器、字符串等的大小或数量。
}
int main(int argc ,char **argv)
{
    ros::init(argc ,argv ,"sub_path_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_path;
    sub_path = nh.subscribe("Path_", 5, pathCallBack);

    ros::spin();

    return 0;
}