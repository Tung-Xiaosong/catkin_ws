#include "robot_recharge/robot_recharge.h"

namespace RobotRecharge_ns{

    RobotRecharge::RobotRecharge() :    //初始化参数
    charge_point_x(0.0),
    charge_point_y(0.0),
    times(1)
    {
        recharge_service_ = nh_.advertiseService("recharge", &RobotRecharge::rechargeCB, this);//回充服务
        back_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }
    RobotRecharge::~RobotRecharge() 
    {

    }

    bool RobotRecharge::publishRechargeGoal()
    {
        MoveBaseClient ac("move_base", true);

        while(!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("waiting for move_base server up ...\n");
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();//时间戳和frame_id一定要有
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position.x = recharge_goal.pose.position.x;
        goal.target_pose.pose.position.y = recharge_goal.pose.position.y;
        goal.target_pose.pose.orientation = recharge_goal.pose.orientation;

        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("goal succeeded !!!\n");
            return true;
        }
        else
        {
            ROS_INFO("goal failed !!!\n");
            return false;
        }
        ROS_ERROR("error happened !!!\n");
        return false;
    }

    //回充服务回调函数定义
    bool RobotRecharge::rechargeCB(robot_recharge::Recharge::Request &req, robot_recharge::Recharge::Response &res)
    {
        charge_point_x = req.charge_point.pose.position.x;
        charge_point_y = req.charge_point.pose.position.y;
        // //创建一个四元数对象q
        // tf2::Quaternion q(
        //     req.charge_point.pose.orientation.x,
        //     req.charge_point.pose.orientation.y,
        //     req.charge_point.pose.orientation.z,
        //     req.charge_point.pose.orientation.w
        // );

        recharge_goal.pose.position.x = charge_point_x + 0.4;
        recharge_goal.pose.position.y = charge_point_y;
        recharge_goal.pose.orientation = req.charge_point.pose.orientation;

        //发布回充目标点
        bool pub_goal = publishRechargeGoal();
        ROS_INFO("recharging !!!\n"); 

        while(pub_goal)
        {
            // 获取机器人当前坐标
            tf::StampedTransform transform;
            try {
                tf_listener_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10));
                tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return false;
            }
            ROS_INFO("base_footprint x = %f , y = %f \n", transform.getOrigin().x(), transform.getOrigin().y());
            
            double dx = fabs(charge_point_x - transform.getOrigin().x());
            
            ROS_INFO("dx = %f \n", dx);

            geometry_msgs::Twist ctrl_cmd;
            ctrl_cmd.linear.x = -0.05;

            back_vel_pub_.publish(ctrl_cmd);
            if (dx <= 0.01)
            {
                ctrl_cmd.linear.x = 0.0;
                back_vel_pub_.publish(ctrl_cmd);
                ROS_WARN("charge done !!!\n");
                return true;
            }
        }
        return false;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_recharge_func");
    RobotRecharge_ns::RobotRecharge bot_recharge;//类的实例化，进入类的构造函数
    ROS_WARN("node initial success !!!\n");
    ros::spin();
    return 0;
}