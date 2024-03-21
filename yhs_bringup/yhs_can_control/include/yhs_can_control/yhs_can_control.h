#ifndef __PIDCONTROL_NODE_H__
#define __PIDCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/ctrl_cmd_init_data.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/odo_fb.h"
#include "yhs_can_msgs/bms_Infor.h"
#include "yhs_can_msgs/bms_flag_Infor.h"
#include "yhs_can_msgs/Drive_MCUEcoder_fb.h"
#include "yhs_can_msgs/Veh_Diag_fb.h"
#include "yhs_can_msgs/ultrasonic.h"//dxs
#include "sensor_msgs/LaserScan.h"//dxs

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
	ros::Publisher cmd_pub_;
	ros::Publisher lr_wheel_fb_pub_;
	ros::Publisher rr_wheel_fb_pub_;
	ros::Publisher io_fb_pub_;
	ros::Publisher odo_fb_pub_;
	ros::Publisher bms_Infor_pub_;
	ros::Publisher bms_flag_Infor_pub_;
	ros::Publisher Drive_MCUEcoder_fb_pub_;
	ros::Publisher Veh_Diag_fb_pub_;
	ros::Publisher odom_pub_;
	ros::Publisher ultrasonic_pub_;
	ros::Publisher scan_pub_;

	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber io_cmd_sub_;

	ros::Subscriber cmd_sub_;
	ros::Subscriber imu_sub_;

	std::string odomFrame_, baseFrame_;
	bool tfUsed_;

	boost::mutex cmd_mutex_;

	int dev_handler_;
	can_frame send_frames_[2];
	can_frame recv_frames_[1];

	double imu_roll_,imu_pitch_,imu_yaw_;

	void io_cmdCallBack(const yhs_can_msgs::io_cmd msg);
	void imu_cmdCallBack(const sensor_msgs::Imu msg);
	void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg);
	void cmdCallBack(const geometry_msgs::Twist msg);

	void recvData();

	void odomPub(float velocity,float steering);

};

}


#endif

