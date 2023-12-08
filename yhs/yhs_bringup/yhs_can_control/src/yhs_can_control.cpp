#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "yhs_can_control.h"


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

//io控制回调函数
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;
	unsigned char sendData_u_io[8];

	cmd_mutex_.lock();

	memset(sendData_u_io,0,8);

	sendData_u_io[0] = msg.io_cmd_enable;

	sendData_u_io[1] = 0xff;
	if(!msg.io_cmd_lower_beam_headlamp)
		sendData_u_io[1] &= 0xfe;

	if(!msg.io_cmd_upper_beam_headlamp)
		sendData_u_io[1] &= 0xfd;

    sendData_u_io[1] |= msg.io_cmd_turn_lamp << 2;

	if(!msg.io_cmd_braking_lamp)
		sendData_u_io[1] &= 0xef;

	if(!msg.io_cmd_clearance_lamp)
		sendData_u_io[1] &= 0xdf;

	if(!msg.io_cmd_fog_lamp)
		sendData_u_io[1] &= 0xcf;

	sendData_u_io[2] = msg.io_cmd_speaker;

	sendData_u_io[3] = 0;
	sendData_u_io[4] = 0;
	sendData_u_io[5] = msg.io_cmd_disCharge;

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io[6] =  count_1 << 4;

	sendData_u_io[7] = sendData_u_io[0] ^ sendData_u_io[1] ^ sendData_u_io[2] ^ sendData_u_io[3] ^ sendData_u_io[4] ^ sendData_u_io[5] ^ sendData_u_io[6];

	send_frames_[0].can_id = 0x98C4D7D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_io, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}

//速度控制回调函数
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	unsigned short vel = 0;
	short angular = msg.ctrl_cmd_steering * 100;
	static unsigned char count = 0;
	unsigned char sendData_u_vel[8];

	cmd_mutex_.lock();

	memset(sendData_u_vel,0,8);

	if(msg.ctrl_cmd_velocity < 0) vel = 0;
	else
	{
		vel = msg.ctrl_cmd_velocity * 1000;
	}

	sendData_u_vel[0] = sendData_u_vel[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_vel[0] = sendData_u_vel[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendData_u_vel[1] = (vel >> 4) & 0xff;

	sendData_u_vel[2] = sendData_u_vel[2] | (0x0f & (vel >> 12));

	sendData_u_vel[2] = sendData_u_vel[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel[3] = (angular >> 4) & 0xff;

	sendData_u_vel[4] = sendData_u_vel[4] | (0xf0 & ((msg.ctrl_cmd_Brake & 0x0f) << 4));

	sendData_u_vel[4] = sendData_u_vel[4] | (0x0f & (angular >> 12));

	sendData_u_vel[5] = 0;

	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel[6] =  count << 4;
	

	sendData_u_vel[7] = sendData_u_vel[0] ^ sendData_u_vel[1] ^ sendData_u_vel[2] ^ sendData_u_vel[3] ^ sendData_u_vel[4] ^ sendData_u_vel[5] ^ sendData_u_vel[6];

	send_frames_[0].can_id = 0x98C4D2D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}


int CanControl::char2bits(char ch)
{
    int bits = 0;
    if (ch >= 'a' && ch <= 'z') 
    {
        bits = ch - 'a' + 10;
    } 
    else if (ch >= 'A' && ch <= 'Z') 
    {
        bits = ch - 'A' + 10;
    } 
    else if (ch >= '0' && ch <= '9') 
    {
        bits = ch - '0';
    } 
    else
    {
        bits = -1;
    }
    return bits;
}
 
int CanControl::hex2bytes(const char *hex, uint8_t *bytes, int size) 
{
    int len = strlen(hex);
    int nbytes = (len + 1) / 3;
    if (nbytes > size) 
    {
        return -1;
    } 
 
    int n;
    for (n = 0; n != nbytes; ++ n) 
    {
        int lndx = n * 3;
        int rndx = lndx + 1;
        int lbits = char2bits(hex[lndx]);
        int rbits = char2bits(hex[rndx]);
        if (lbits == -1 || rbits == -1)
        {
            return -1;
        }
        bytes[n] = (lbits << 4) | rbits;
    }
 
    return nbytes;
}

//CAN原始数据接收函数
void CanControl::ctrl_init_data_cmdCallBack(const yhs_can_msgs::ctrl_cmd_init_data msg)
{
	const char* temp_data;
	uint8_t get_data[8];

	static unsigned char count = 0;

	cmd_mutex_.lock();

	temp_data = msg.data.c_str();

    int nbytes = hex2bytes(temp_data, get_data, 8);
	if (nbytes == 8) 
    {
/*      int i = 0;
        for ( ; i < nbytes; ++ i) 
        {
            printf("%02X ", get_data[i]);
        }
		printf("\r\n");*/
    }
	else
	{
		ROS_ERROR("input can data error!");
		cmd_mutex_.unlock();
		return;
	}

	if(msg.can_id > 100000)
	{
	
		count ++;

		if(count == 16)	count = 0;

		get_data[6] =  count << 4;
	
		get_data[7] = get_data[0] ^ get_data[1] ^ get_data[2] ^ get_data[3] ^ get_data[4] ^ get_data[5] ^ get_data[6];

	}
	else
	{
		
	}

	send_frames_[0].can_id = msg.can_id;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, get_data, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}


//继电器接收函数
void CanControl::ctrl_relay_cmdCallBack(const std_msgs::UInt8 msg)
{
	const char* temp_data;
	uint8_t get_data[8] = {0};

	

	cmd_mutex_.lock();

	
	
	get_data[0] = msg.data;


	send_frames_[0].can_id = 0x101;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, get_data, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}

//数据接收解析线程
void CanControl::recvData()
{

	while(ros::ok())//ros::ok()是ROS提供的一个函数，用于检查ROS节点是否已经被关闭。当ROS节点被关闭时，ros::ok()会返回false，循环会停止执行。
	{

		if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
		//recv_frames_数组用于存储从CAN总线上接收到的数据帧
		//read函数从dev_handler_所表示的设备文件中读取数据，并将读取到的数据存储到recv_frames_[0]所表示的内存地址中。
		//sizeof(recv_frames_[0])表示要读取的数据的大小，即一个CAN数据帧的大小。
		{
			for(int j=0;j<1;j++)
			{
				
				switch (recv_frames_[0].can_id)
				{
					//速度控制反馈
					case 0x98C4D2EF:
					{
						yhs_can_msgs::ctrl_fb msg;
						msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
						
						msg.ctrl_fb_velocity = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
						
						msg.ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

						msg.ctrl_fb_Brake = (recv_frames_[0].data[4] & 0x30) >> 4;
						
						msg.ctrl_fb_mode = (recv_frames_[0].data[4] & 0xc0) >> 6;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];
                                                
						geometry_msgs::Twist cmd_fb;
						cmd_fb.linear.x = msg.ctrl_fb_velocity;
						cmd_fb.angular.z = msg.ctrl_fb_steering/180*3.14;
						if(crc == recv_frames_[0].data[7])
						{
							cmd_pub_.publish(cmd_fb);	
							ctrl_fb_pub_.publish(msg);
							if(msg.ctrl_fb_gear == 2) msg.ctrl_fb_velocity = -msg.ctrl_fb_velocity;
							odomPub(msg.ctrl_fb_velocity,msg.ctrl_fb_steering/180*3.14);
						}

						break;
					}

					//左轮反馈
					case 0x98C4D7EF:
					{
						yhs_can_msgs::lr_wheel_fb msg;
						msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							lr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//右轮反馈
					case 0x98C4D8EF:
					{
						yhs_can_msgs::rr_wheel_fb msg;
						msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							rr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//io反馈
					case 0x98C4DAEF:
					{
						yhs_can_msgs::io_fb msg;
						if(0x01 & recv_frames_[0].data[0]) msg.io_fb_enable = true;	else msg.io_fb_enable = false;

						msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

						if(0x10 & recv_frames_[0].data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

						if(0x02 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						if(0x10 & recv_frames_[0].data[3]) msg.io_fb_rm_impact_sensor = true;	else msg.io_fb_rm_impact_sensor = false;

						if(0x01 & recv_frames_[0].data[5]) msg.io_fb_disCharge = true;	else msg.io_fb_disCharge = false;

						if(0x02 & recv_frames_[0].data[5]) msg.io_fb_chargeEn = true;	else msg.io_fb_chargeEn = false;

						if(0x10 & recv_frames_[0].data[5]) msg.io_fb_ScramSt = true;	else msg.io_fb_ScramSt = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					//里程计反馈
					case 0x98C4DEEF:
					{
						yhs_can_msgs::odo_fb msg;
						msg.odo_fb_accumulative_mileage = (float)((int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

						msg.odo_fb_accumulative_angular = (float)((int)(recv_frames_[0].data[7] << 24 | recv_frames_[0].data[6] << 16 | recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 1000;

						odo_fb_pub_.publish(msg);

						break;
					}

					//bms_Infor反馈
					case 0x98C4E1EF:
					{
						yhs_can_msgs::bms_Infor msg;
						msg.bms_Infor_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.bms_Infor_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						msg.bms_Infor_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_Infor_pub_.publish(msg);
						}

						break;
					}

					//bms_flag_Infor反馈
					case 0x98C4E2EF:
					{
						yhs_can_msgs::bms_flag_Infor msg;
						msg.bms_flag_Infor_soc = recv_frames_[0].data[0];

						if(0x01 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_ov = true;	else msg.bms_flag_Infor_single_ov = false;

						if(0x02 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_uv = true;	else msg.bms_flag_Infor_single_uv = false;

						if(0x04 & recv_frames_[0].data[1]) msg.bms_flag_Infor_ov = true;	else msg.bms_flag_Infor_ov = false;

						if(0x08 & recv_frames_[0].data[1]) msg.bms_flag_Infor_uv = true;	else msg.bms_flag_Infor_uv = false;

						if(0x10 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ot = true;	else msg.bms_flag_Infor_charge_ot = false;

						if(0x20 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ut = true;	else msg.bms_flag_Infor_charge_ut = false;

						if(0x40 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ot = true;	else msg.bms_flag_Infor_discharge_ot = false;

						if(0x80 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ut = true;	else msg.bms_flag_Infor_discharge_ut = false;

						if(0x01 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_oc = true;	else msg.bms_flag_Infor_charge_oc = false;

						if(0x02 & recv_frames_[0].data[2]) msg.bms_flag_Infor_discharge_oc = true;	else msg.bms_flag_Infor_discharge_oc = false;

						if(0x04 & recv_frames_[0].data[2]) msg.bms_flag_Infor_short = true;	else msg.bms_flag_Infor_short = false;

						if(0x08 & recv_frames_[0].data[2]) msg.bms_flag_Infor_ic_error = true;	else msg.bms_flag_Infor_ic_error = false;

						if(0x10 & recv_frames_[0].data[2]) msg.bms_flag_Infor_lock_mos = true;	else msg.bms_flag_Infor_lock_mos = false;

						if(0x20 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_flag = true;	else msg.bms_flag_Infor_charge_flag = false;

						if(0x40 & recv_frames_[0].data[2]) msg.bms_flag_Infor_SOCWarning = true;	else msg.bms_flag_Infor_SOCWarning = false;

						if(0x80 & recv_frames_[0].data[2]) msg.bms_flag_Infor_SOCLowProtection = true;	else msg.bms_flag_Infor_SOCLowProtection = false;

						msg.bms_flag_Infor_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

						msg.bms_flag_Infor_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_flag_Infor_pub_.publish(msg);
						}

						break;
					}

					//Drive_fb_MCUEcoder反馈
					case 0x98C4DCEF:
					{
						yhs_can_msgs::Drive_MCUEcoder_fb msg;
						msg.Drive_fb_MCUEcoder = (int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]); 

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Drive_MCUEcoder_fb_pub_.publish(msg);
						}

						break;
					}

					//Veh_fb_Diag反馈
					case 0x98C4EAEF:
					{
						yhs_can_msgs::Veh_Diag_fb msg;
						msg.Veh_fb_FaultLevel = 0x0f & recv_frames_[0].data[0];

						if(0x10 & recv_frames_[0].data[0]) msg.Veh_fb_AutoCANCtrlCmd = true;	else msg.Veh_fb_AutoCANCtrlCmd = false;

						if(0x20 & recv_frames_[0].data[0]) msg.Veh_fb_AutoIOCANCmd = true;	else msg.Veh_fb_AutoIOCANCmd = false;

						if(0x01 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisOnline = true;	else msg.Veh_fb_EPSDisOnline = false;

						if(0x02 & recv_frames_[0].data[1]) msg.Veh_fb_EPSfault = true;	else msg.Veh_fb_EPSfault = false;

						if(0x04 & recv_frames_[0].data[1]) msg.Veh_fb_EPSMosfetOT = true;	else msg.Veh_fb_EPSMosfetOT = false;

						if(0x08 & recv_frames_[0].data[1]) msg.Veh_fb_EPSWarning = true;	else msg.Veh_fb_EPSWarning = false;

						if(0x10 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisWork = true;	else msg.Veh_fb_EPSDisWork = false;

						if(0x20 & recv_frames_[0].data[1]) msg.Veh_fb_EPSOverCurrent = true;	else msg.Veh_fb_EPSOverCurrent = false;

						
						
						if(0x10 & recv_frames_[0].data[2]) msg.Veh_fb_EHBecuFault = true;	else msg.Veh_fb_EHBecuFault = false;

						if(0x20 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisOnline = true;	else msg.Veh_fb_EHBDisOnline = false;

						if(0x40 & recv_frames_[0].data[2]) msg.Veh_fb_EHBWorkModelFault = true;	else msg.Veh_fb_EHBWorkModelFault = false;

						if(0x80 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisEn = true;	else msg.Veh_fb_EHBDisEn = false;


						if(0x01 & recv_frames_[0].data[3]) msg.Veh_fb_EHBAnguleFault = true;	else msg.Veh_fb_EHBAnguleFault = false;

						if(0x02 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOT = true;	else msg.Veh_fb_EHBOT = false;

						if(0x04 & recv_frames_[0].data[3]) msg.Veh_fb_EHBPowerFault = true;	else msg.Veh_fb_EHBPowerFault = false;

						if(0x08 & recv_frames_[0].data[3]) msg.Veh_fb_EHBsensorAbnomal = true;	else msg.Veh_fb_EHBsensorAbnomal = false;

						if(0x10 & recv_frames_[0].data[3]) msg.Veh_fb_EHBMotorFault = true;	else msg.Veh_fb_EHBMotorFault = false;

						if(0x20 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilPressSensorFault = true;	else msg.Veh_fb_EHBOilPressSensorFault = false;

						if(0x40 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilFault = true;	else msg.Veh_fb_EHBOilFault = false;


						msg.Veh_fb_LDrvMCUFault = 0x3f & recv_frames_[0].data[4];
						msg.Veh_fb_RDrvMCUFault = (recv_frames_[0].data[5] & 0x0f << 2) | (recv_frames_[0].data[4] >> 6);


						if(0x10 & recv_frames_[0].data[5]) msg.Veh_fb_AUXBMSDisOnline = true;	else msg.Veh_fb_AUXBMSDisOnline = false;

						if(0x20 & recv_frames_[0].data[5]) msg.Veh_fb_AuxScram = true;	else msg.Veh_fb_AuxScram = false;

						if(0x40 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteClose = true;	else msg.Veh_fb_AuxRemoteClose = false;

						if(0x80 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteDisOnline = true;	else msg.Veh_fb_AuxRemoteDisOnline = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Veh_Diag_fb_pub_.publish(msg);
						}


						break;
					}

					default:
						break;
				}

			}

					
		}
	}
}

void CanControl::imu_cmdCallBack(const sensor_msgs::Imu msg)
{

	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg.orientation,quat);
	tf::Matrix3x3(quat).getRPY(imu_roll_,imu_pitch_,imu_yaw_);

}

void CanControl::odomPub(float velocity,float steering)
{
	static double x = 0.0;
	static double y = 0.0;
	static double th = 0.0;

	th = imu_yaw_;

	double x_mid = 0.0;
	double y_mid = 0.0;

	static tf::TransformBroadcaster odom_broadcaster;

	static ros::Time last_time = ros::Time::now();
	ros::Time current_time;


	double vx = velocity;
	double vth = vx * tan(steering) / 0.63;

	current_time = ros::Time::now();

	//compute odometry in a typical way given the velocities of the robot
	double dt = (current_time - last_time).toSec();


	double delta_x = (vx * cos(th)) * dt;
	double delta_y = (vx * sin(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
//	th += delta_th;


	x_mid = x + 0.31 * cos(th);
	y_mid = y + 0.31 * sin(th);

	//转换为四元素
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = odomFrame_;
	odom_trans.child_frame_id = baseFrame_;

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//是否发布tf转换
	if(tfUsed_)
		odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = odomFrame_;

	//set the position
	odom.pose.pose.position.x = x_mid;
	odom.pose.pose.position.y = y_mid;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = baseFrame_;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vth;

	odom.pose.covariance[0]  = 0.1;   	// x的协方差 
	odom.pose.covariance[7]  = 0.1;		// y的协方差
	odom.pose.covariance[35] = 0.2;   	//theta的协方差

	odom.pose.covariance[14] = 1e10; 	// set a non-zero covariance on unused    theta x axis
	odom.pose.covariance[21] = 1e10; 	// dimensions (z, pitch and roll); this   theta y  axis
	odom.pose.covariance[28] = 1e10; 	// is a requirement of robot_pose_ekf     theta z axis

	//publish the message
	odom_pub_.publish(odom);

	last_time = current_time;

}


void CanControl::cmdCallBack(const geometry_msgs::Twist msg)
{
	unsigned short vel = 0;
	unsigned char ctrl_cmd_gear = 0;
	short angular = msg.angular.z / 3.14 * 180 * 100;
	static unsigned char count = 0;
	unsigned char sendData_u_vel[8];

	cmd_mutex_.lock();

	memset(sendData_u_vel,0,8);

	vel = abs(msg.linear.x * 1000);

	if(msg.linear.x < 0)
	{
		ctrl_cmd_gear = 2;
	}
	else
	{
		ctrl_cmd_gear = 4;
	}

	sendData_u_vel[0] = sendData_u_vel[0] | (0x0f & ctrl_cmd_gear);
	
	sendData_u_vel[0] = sendData_u_vel[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendData_u_vel[1] = (vel >> 4) & 0xff;

	sendData_u_vel[2] = sendData_u_vel[2] | (0x0f & (vel >> 12));

	sendData_u_vel[2] = sendData_u_vel[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel[3] = (angular >> 4) & 0xff;

	sendData_u_vel[4] = sendData_u_vel[4] | (0xf0 & ((0 & 0x0f) << 4));

	sendData_u_vel[4] = sendData_u_vel[4] | (0x0f & (angular >> 12));

	sendData_u_vel[5] = 0;

	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel[6] =  count << 4;
	

	sendData_u_vel[7] = sendData_u_vel[0] ^ sendData_u_vel[1] ^ sendData_u_vel[2] ^ sendData_u_vel[3] ^ sendData_u_vel[4] ^ sendData_u_vel[5] ^ sendData_u_vel[6];

	send_frames_[0].can_id = 0x98C4D2D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}

void CanControl::run()
{

	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);

	cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("smoother_cmd_vel", 5, &CanControl::cmdCallBack, this);
	imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_data", 5, &CanControl::imu_cmdCallBack, this);

	cmd_init_data_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd_init_data>("can_data", 5, &CanControl::ctrl_init_data_cmdCallBack, this);
	cmd_relay_sub_ = nh_.subscribe<std_msgs::UInt8>("can/relay", 5, &CanControl::ctrl_relay_cmdCallBack, this);

	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb",5);
	lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb",5);
	rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb",5);
	odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb",5);
	bms_Infor_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor>("bms_Infor",5);
	bms_flag_Infor_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor>("bms_flag_Infor",5);
	Drive_MCUEcoder_fb_pub_ = nh_.advertise<yhs_can_msgs::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb",5);
	Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb",5);

	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
	cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_fb", 5);

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


    // bind socket to network interface
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

	ros::spin();
	
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
