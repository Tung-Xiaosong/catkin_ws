#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "yhs_can_control.h"

#include <fstream>


namespace yhs_tool {


CanControl::CanControl()
{
	ros::NodeHandle private_node("~");

	private_node.param("odom_frame", odomFrame_, std::string("odom"));
  private_node.param("base_link_frame", baseFrame_, std::string("base_link"));

	
	private_node.param("tfUsed", tfUsed_, false);
	
}


CanControl::~CanControl()
{

}


void CanControl::io_cmdCallBack(const yhs_msgs::io_cmd msg)
{
	static unsigned char count = 0;

	unsigned char sendDataTemp[8] = {0};

	cmd_mutex_.lock();

	sendDataTemp[0] = 0xff;
	if(msg.io_cmd_lamp_ctrl)
		sendDataTemp[0] &= 0xff;
	else sendDataTemp[0] &= 0xfe;
	if(msg.io_cmd_unlock)
		sendDataTemp[0] &= 0xff;
	else sendDataTemp[0] &= 0xfd;

	sendDataTemp[1] = 0xff;
	if(msg.io_cmd_lower_beam_headlamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xfe;
	if(msg.io_cmd_upper_beam_headlamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendDataTemp[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendDataTemp[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendDataTemp[1] &= 0xfb;

	if(msg.io_cmd_braking_lamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xef;
	if(msg.io_cmd_clearance_lamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xdf;
	if(msg.io_cmd_fog_lamp)
		sendDataTemp[1] &= 0xff;
	else sendDataTemp[1] &= 0xbf;

	sendDataTemp[2] = msg.io_cmd_speaker;

	count ++;
	if(count == 16)	count = 0;

	sendDataTemp[6] =  count << 4;

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D7D0;
    send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
  if (ret <= 0) 
  {
    ROS_ERROR("send message failed, error code: %d",ret);
  }
	
	cmd_mutex_.unlock();
}


void CanControl::ctrl_cmdCallBack(const yhs_msgs::ctrl_cmd msg)
{
	const short linear = msg.ctrl_cmd_linear * 1000;
	const short angular = msg.ctrl_cmd_angular * 100;
	const int gear = msg.ctrl_cmd_gear;

	static unsigned char count = 0;
	unsigned char sendDataTemp[8] = {0};

	cmd_mutex_.lock();

	sendDataTemp[0] = sendDataTemp[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendDataTemp[1] = (linear >> 4) & 0xff;

	sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linear >> 12));


	sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendDataTemp[3] = (angular >> 4) & 0xff;

	sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

	count ++;

	if(count == 16)	count = 0;

	sendDataTemp[6] =  count << 4;
	

	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D1D0;
    send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
  if (ret <= 0) 
  {
    ROS_ERROR("send message failed, error code: %d",ret);
  }

	cmd_mutex_.unlock();
}


void CanControl::cmdCallBack(const geometry_msgs::Twist msg)
{
	const short linear = msg.linear.x * 1000;
	const short angular = msg.angular.z / 3.14 * 180 * 100;
	const int gear = 3;
	static unsigned char count = 0;

	unsigned char sendDataTemp[8] = {0};

	cmd_mutex_.lock();

	sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);
	
	sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendDataTemp[1] = (linear >> 4) & 0xff;

	sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linear >> 12));

	sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendDataTemp[3] = (angular >> 4) & 0xff;

	sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

	count ++;

	if(count == 16)	count = 0;

	sendDataTemp[6] =  count << 4;
	
	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D1D0;
  send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
  if (ret <= 0) 
  {
    ROS_ERROR("send message failed, error code: %d",ret);
  }

	cmd_mutex_.unlock();
}


void CanControl::free_ctrl_cmdCallBack(const yhs_msgs::free_ctrl_cmd msg)
{
	const short linearl = msg.free_ctrl_cmd_velocity_l * 1000;
	const short linearr = msg.free_ctrl_cmd_velocity_r * 1000;
	static unsigned char count = 0;

	unsigned char sendDataTemp[8] = {0};

	cmd_mutex_.lock();

	sendDataTemp[0] = sendDataTemp[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linearl & 0x0f) << 4));

	sendDataTemp[1] = (linearl >> 4) & 0xff;

	sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linearl >> 12));

	sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((linearr & 0x0f) << 4));

	sendDataTemp[3] = (linearr >> 4) & 0xff;

	sendDataTemp[4] = sendDataTemp[4] | (0x0f & (linearr >> 12));

	count ++;

	if(count == 16)	count = 0;

	sendDataTemp[6] =  count << 4;
	
	sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

	can_frame send_frame;

	send_frame.can_id = 0x98C4D2D0;
  send_frame.can_dlc = 8;

	memcpy(send_frame.data, sendDataTemp, 8);

	int ret = write(dev_handler_, &send_frame, sizeof(send_frame));
  if (ret <= 0) 
	{
    ROS_ERROR("send message failed, error code: %d",ret);
  }

	cmd_mutex_.unlock();
}


void CanControl::recvData()
{
	ros::Rate loop(100);

	static yhs_msgs::ChassisInfoFb chassis_info_msg;
	while(ros::ok())
	{
		if(read(dev_handler_, &recv_frames_, sizeof(recv_frames_)) >= 0)
		{
			switch (recv_frames_.can_id)
			{
				case 0x98C4D1EF:
				{
					yhs_msgs::ctrl_fb msg;
					
					msg.ctrl_fb_target_gear = recv_frames_.data[0] & 0x0f;
					msg.ctrl_fb_linear = static_cast<float>(static_cast<short>((recv_frames_.data[2] & 0x0f) << 12) | (recv_frames_.data[1] << 4) | ((recv_frames_.data[0] & 0xf0) >> 4)) / 1000.0;
					msg.ctrl_fb_angular = static_cast<float>(static_cast<short>((recv_frames_.data[4] & 0x0f) << 12) | (recv_frames_.data[3] << 4) | ((recv_frames_.data[2] & 0xf0) >> 4)) / 100.0;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
					
						chassis_info_msg.header.stamp=ros::Time::now();
						chassis_info_msg.ctrl_fb = msg;
						chassis_info_fb_pub_.publish(chassis_info_msg);
						ctrl_fb_pub_.publish(msg);

						odomPub(msg.ctrl_fb_linear * 1.018, msg.ctrl_fb_angular/180*3.14);
					}

					break;
				}

				case 0x98C4D7EF:
				{
					yhs_msgs::l_wheel_fb msg;

					msg.l_wheel_fb_velocity = static_cast<float>((recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 1000.0f;
					msg.l_wheel_fb_pulse = static_cast<int32_t>((recv_frames_.data[5] << 24) | (recv_frames_.data[4] << 16) | (recv_frames_.data[3] << 8) | recv_frames_.data[2]);

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						chassis_info_msg.l_wheel_fb = msg;	
						l_wheel_fb_pub_.publish(msg);
					}

					break;
				}

				//
				case 0x98C4D8EF:
				{
					yhs_msgs::r_wheel_fb msg;

					msg.r_wheel_fb_velocity = static_cast<float>((recv_frames_.data[1] << 8 | recv_frames_.data[0])) / 1000.0f;
					msg.r_wheel_fb_pulse = static_cast<int32_t>((recv_frames_.data[5] << 24) | (recv_frames_.data[4] << 16) | (recv_frames_.data[3] << 8) | recv_frames_.data[2]);

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{	
						chassis_info_msg.r_wheel_fb = msg;
						r_wheel_fb_pub_.publish(msg);
					}

					break;
				}

				//io反馈
				case 0x98C4DAEF:
				{
					yhs_msgs::io_fb msg;

					msg.io_fb_lamp_ctrl = (recv_frames_.data[0] & 0x01) != 0;
					msg.io_fb_unlock = (recv_frames_.data[1] & 0x02) != 0;
					msg.io_fb_lower_beam_headlamp = (recv_frames_.data[1] & 0x01) != 0;
					msg.io_fb_upper_beam_headlamp = (recv_frames_.data[1] & 0x02) != 0;
					msg.io_fb_turn_lamp = (recv_frames_.data[1] & 0xc0) >> 2;
					msg.io_fb_braking_lamp = (recv_frames_.data[1] & 0x10) != 0;
					msg.io_fb_clearance_lamp = (recv_frames_.data[1] & 0x20) != 0;
					msg.io_fb_fog_lamp = (recv_frames_.data[1] & 0x40) != 0;
					msg.io_fb_speaker = (recv_frames_.data[2] & 0x01) != 0;
					msg.io_fb_fl_impact_sensor = (recv_frames_.data[3] & 0x01) != 0;
					msg.io_fb_fm_impact_sensor = (recv_frames_.data[3] & 0x02) != 0;
					msg.io_fb_fr_impact_sensor = (recv_frames_.data[3] & 0x04) != 0;
					msg.io_fb_rl_impact_sensor = (recv_frames_.data[3] & 0x08) != 0;
					msg.io_fb_rm_impact_sensor = (recv_frames_.data[3] & 0x10) != 0;
					msg.io_fb_rr_impact_sensor = (recv_frames_.data[3] & 0x20) != 0;
					msg.io_fb_fl_drop_sensor = (recv_frames_.data[4] & 0x01) != 0;
					msg.io_fb_fm_drop_sensor = (recv_frames_.data[4] & 0x02) != 0;
					msg.io_fb_fr_drop_sensor = (recv_frames_.data[4] & 0x04) != 0;
					msg.io_fb_rl_drop_sensor = (recv_frames_.data[4] & 0x08) != 0;
					msg.io_fb_rm_drop_sensor = (recv_frames_.data[4] & 0x10) != 0;
					msg.io_fb_rr_drop_sensor = (recv_frames_.data[4] & 0x20) != 0;
					msg.io_fb_estop = (recv_frames_.data[5] & 0x01) != 0;
					msg.io_fb_joypad_ctrl = (recv_frames_.data[5] & 0x02) != 0;
					msg.io_fb_charge_state = (recv_frames_.data[5] & 0x04) != 0;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						chassis_info_msg.io_fb = msg;
						io_fb_pub_.publish(msg);
					}

					break;
				}

				
				//bms反馈
				case 0x98C4E1EF:
				{
					yhs_msgs::bms_fb msg;

					msg.bms_fb_voltage = static_cast<float>(reinterpret_cast<unsigned short&>(recv_frames_.data[0])) / 100;
					msg.bms_fb_current = static_cast<float>(reinterpret_cast<short&>(recv_frames_.data[2])) / 100;
					msg.bms_fb_remaining_capacity = static_cast<float>(reinterpret_cast<unsigned short&>(recv_frames_.data[4])) / 100;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						chassis_info_msg.bms_fb = msg;
						bms_fb_pub_.publish(msg);
					}

					break;
				}

				//bms_flag反馈
				case 0x98C4E2EF:
				{
					yhs_msgs::bms_flag_fb msg;

					msg.bms_flag_fb_soc = recv_frames_.data[0];
					msg.bms_flag_fb_single_ov = (0x01 & recv_frames_.data[1]) != 0;
					msg.bms_flag_fb_single_uv = (0x02 & recv_frames_.data[1]) != 0;
					msg.bms_flag_fb_ov = (0x04 & recv_frames_.data[1]) != 0;
					msg.bms_flag_fb_uv = (0x08 & recv_frames_.data[1]) != 0;
					msg.bms_flag_fb_charge_ot = (0x10 & recv_frames_.data[1]) != 0;
					msg.bms_flag_fb_charge_ut = (0x20 & recv_frames_.data[1]) != 0;
					msg.bms_flag_fb_discharge_ot = (0x40 & recv_frames_.data[1]) != 0;
					msg.bms_flag_fb_discharge_ut = (0x80 & recv_frames_.data[1]) != 0;
					msg.bms_flag_fb_charge_oc = (0x01 & recv_frames_.data[2]) != 0;
					msg.bms_flag_fb_discharge_oc = (0x02 & recv_frames_.data[2]) != 0;
					msg.bms_flag_fb_short = (0x04 & recv_frames_.data[2]) != 0;
					msg.bms_flag_fb_ic_error = (0x08 & recv_frames_.data[2]) != 0;
					msg.bms_flag_fb_lock_mos = (0x10 & recv_frames_.data[2]) != 0;
					msg.bms_flag_fb_charge_flag = (0x20 & recv_frames_.data[2]) != 0;

					const float kTemperatureConversionFactor = 0.1;
					msg.bms_flag_fb_hight_temperature = static_cast<float>((static_cast<short>(recv_frames_.data[4] << 4 | recv_frames_.data[3] >> 4))) * kTemperatureConversionFactor;
					msg.bms_flag_fb_low_temperature = static_cast<float>((static_cast<short>((recv_frames_.data[6] & 0x0f) << 8 | recv_frames_.data[5]))) * kTemperatureConversionFactor;

					unsigned char crc = recv_frames_.data[0] ^ recv_frames_.data[1] ^ recv_frames_.data[2] ^ recv_frames_.data[3] ^ recv_frames_.data[4] ^ recv_frames_.data[5] ^ recv_frames_.data[6];

					if(crc == recv_frames_.data[7])
					{
						chassis_info_msg.bms_flag_fb = msg;
						bms_flag_fb_pub_.publish(msg);
					}

					break;
				}

				default:
					break;
			}			
		}
	}
}

void CanControl::odomPub(const float linear,const float angular)
{
	static double x = 0.0;
	static double y = 0.0;
	static double th = 0.0;

	static double lastYaw = 0;

	static tf::TransformBroadcaster odom_broadcaster;

	static ros::Time last_time = ros::Time::now();
	ros::Time current_time;


	double vx = linear;
	double vth = angular;

	current_time = ros::Time::now();

	double dt = (current_time - last_time).toSec();

	double delta_x = (vx * cos(th)) * dt;
	double delta_y = (vx * sin(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = odomFrame_;
	odom_trans.child_frame_id = baseFrame_;

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	if(tfUsed_)
		odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = odomFrame_;

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom.child_frame_id = baseFrame_;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vth;

	odom.pose.covariance[0]  = 0.1;   	
	odom.pose.covariance[7]  = 0.1;		
	odom.pose.covariance[35] = 0.2;   

	odom.pose.covariance[14] = 1e10; 	
	odom.pose.covariance[21] = 1e10; 	
	odom.pose.covariance[28] = 1e10; 	
	odom_pub_.publish(odom);

	last_time = current_time;

}


void CanControl::sendData()
{
	ros::Rate loop(100);


	while(ros::ok())
	{

		loop.sleep();
	}
}


void CanControl::run()
{

	ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);
	free_ctrl_cmd_sub_ = nh_.subscribe<yhs_msgs::free_ctrl_cmd>("rear_free_ctrl_cmd", 5, &CanControl::free_ctrl_cmdCallBack, this);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("smoother_cmd_vel", 5, &CanControl::cmdCallBack, this);

	ctrl_fb_pub_ = nh_.advertise<yhs_msgs::ctrl_fb>("ctrl_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_msgs::io_fb>("io_fb",5);
	r_wheel_fb_pub_ = nh_.advertise<yhs_msgs::r_wheel_fb>("r_wheel_fb",5);
	l_wheel_fb_pub_ = nh_.advertise<yhs_msgs::l_wheel_fb>("l_wheel_fb",5);
	bms_fb_pub_ = nh_.advertise<yhs_msgs::bms_fb>("bms_fb",5);
	bms_flag_fb_pub_ = nh_.advertise<yhs_msgs::bms_flag_fb>("bms_flag_fb",5);
	chassis_info_fb_pub_ = nh_.advertise<yhs_msgs::ChassisInfoFb>("chassis_info_fb",5);

	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);

	//打开设备
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dev_handler_ < 0) 
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}
    else
	{
		ROS_INFO(">>open can deivce success!");
	}


	struct ifreq ifr;
	
	std::string can_name("can0");

	strcpy(ifr.ifr_name,can_name.c_str());

	ioctl(dev_handler_,SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
	if (ret < 0) 
	{
		ROS_ERROR(">>bind dev_handler error!\r\n");
		return;
	}
	
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
//	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

	ros::spin();

	if(recvdata_thread.joinable()) recvdata_thread.join();

	close(dev_handler_);
}

}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "yhs_can_control_node");

	yhs_tool::CanControl cancontrol;
	cancontrol.run();

	return 0;
}
