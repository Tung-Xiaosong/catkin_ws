#ifndef ROBOT_RECHARGE_H
#define ROBOT_RECHARGE_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>//tf2::Quaternion()
#include <move_base_msgs/MoveBaseGoal.h>//move_base_msgs::MoveBaseGoal
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>//geometry_msgs::Twist ctrl_cmd;
#include <tf/transform_listener.h>//tf::TransformListener tf_listener_;
#include "robot_recharge/Recharge.h"


namespace RobotRecharge_ns{

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    class RobotRecharge{

        public:
            RobotRecharge();
            ~RobotRecharge();

        private:
            ros::NodeHandle nh_;

            tf::TransformListener tf_listener_;

            //充电点
            double charge_point_x;
            double charge_point_y;
            int times;
            //回充导航点
            geometry_msgs::PoseStamped recharge_goal;

            //倒车速度发布
            ros::Publisher back_vel_pub_;

            //回充服务器对象声明
            ros::ServiceServer recharge_service_;

            //发布回充目标点函数
            bool publishRechargeGoal();

            //回充服务回调函数声明
            bool rechargeCB(robot_recharge::Recharge::Request &req, robot_recharge::Recharge::Response &res);
 
    };
}

#endif  //ROBOT_RECHARGE_H
