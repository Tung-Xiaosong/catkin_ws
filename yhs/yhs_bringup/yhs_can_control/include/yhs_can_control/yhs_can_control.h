#ifndef __CANCONTROL_NODE_H__
#define __CANCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_msgs/ctrl_cmd.h"
#include "yhs_msgs/io_cmd.h"
#include "yhs_msgs/ctrl_fb.h"
#include "yhs_msgs/l_wheel_fb.h"
#include "yhs_msgs/r_wheel_fb.h"
#include "yhs_msgs/io_fb.h"
#include "yhs_msgs/free_ctrl_cmd.h"
#include "yhs_msgs/bms_fb.h"
#include "yhs_msgs/bms_flag_fb.h"
#include "yhs_msgs/ChassisInfoFb.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>


namespace yhs_tool {
class CanControl
{
public:
	CanControl();
	~CanControl();
	
	void run();
private:
	ros::NodeHandle nh_;

	ros::Publisher ctrl_fb_pub_;
	ros::Publisher l_wheel_fb_pub_;
	ros::Publisher r_wheel_fb_pub_;
	ros::Publisher io_fb_pub_;
	ros::Publisher bms_fb_pub_;
	ros::Publisher bms_flag_fb_pub_;
	ros::Publisher chassis_info_fb_pub_;
	ros::Publisher odom_pub_;

	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber io_cmd_sub_;
	ros::Subscriber free_ctrl_cmd_sub_;

  ros::Subscriber cmd_sub_;

	boost::mutex cmd_mutex_;

	std::string odomFrame_, baseFrame_;
	bool tfUsed_;

	int dev_handler_;
	can_frame recv_frames_;

	

	void io_cmdCallBack(const yhs_msgs::io_cmd msg);
	void ctrl_cmdCallBack(const yhs_msgs::ctrl_cmd msg);
  void cmdCallBack(const geometry_msgs::Twist msg);
	void free_ctrl_cmdCallBack(const yhs_msgs::free_ctrl_cmd msg);

	void odomPub(const float linear,const float angular);


	void recvData();
	void sendData();

};

}


#endif

