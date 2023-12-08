//使雷达小车跟随目标mbot4
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
ros::Publisher velpub;

void subCallback(const sensor_msgs::LaserScan msg)//sensor_msgs::LaserScan
{
   geometry_msgs::Twist vel_msg;
   int index = 0;
   float obs_angle = 0;//雷达机器人和目标机器人之间的角度
   float obs_min_dist = msg.range_max;//雷达扫描到的最远距离
   
   for (size_t i = 0; i < msg.ranges.size(); i++)
    {
        if(msg.ranges[i]<obs_min_dist){
            obs_min_dist = msg.ranges[i];
            obs_angle = msg.angle_min+(float)i*msg.angle_increment;
        }
    }
   vel_msg.linear.x = obs_min_dist*0.3;//将雷达能扫描到的最近的激光点的距离赋给x方向速度(误差作为控制变量)
   if(vel_msg.linear.x > 2&&vel_msg.linear.x < 3) vel_msg.linear.x = 1.0;//雷达距离目标2~3时速度设为1
   vel_msg.angular.z =-obs_angle*2;//旋转速度
   if((obs_angle < 0.03)&&(obs_angle > -0.03)) vel_msg.angular.z = 0;//当雷达机器人和目标机器人的角度在0.03范围内,则旋转速度为0
   std::cout<<"angle is:"<<obs_angle<<std::endl;
   if((obs_min_dist >= msg.range_max)||(obs_min_dist<2)) vel_msg.linear.x = 0.0;//目标超出雷达范围或目标距离雷达小于2时,停下
   velpub.publish(vel_msg);//给雷达机器人发布速度命令
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"mbot_control");
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("/scan", 50, subCallback);/*scan*///订阅雷达消息scan,进入回调函数
  velpub = nh.advertise<geometry_msgs::Twist>("/mbot1/cmd_vel", 10);//给雷达机器人发布速度命令

  ros::spin();
  return 0;
}
