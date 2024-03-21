#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstring>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <yhs_can_msgs/ctrl_cmd.h>
std::mutex mtx; // 互斥锁，用于保护共享资源
ros::Publisher pub;

void sendPositionInfo(int serial_port) {
    while (true) {
        // 机器人位置信息发送逻辑
        std::string position_info = "Position: x=1.0 y=2.0 yaw=0.0\n";
        mtx.lock(); // 锁定互斥锁
        write(serial_port, position_info.c_str(), position_info.length());
        mtx.unlock(); // 解锁互斥锁
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 每隔1秒发送一次
    }
}

void printOne(int serial_port) {
    int buf[256];
                     memset(buf, 0, sizeof(buf));
                 int bytes_read = read(serial_port, buf, sizeof(buf));
        // 打印1的逻辑
        // std::cout << "1" << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(1)); // 每隔1秒打印一次
        //ros::NodeHandle nh;
        
        yhs_can_msgs::ctrl_cmd cmd_msg;

        //ros::Rate loop_rate(100);

        if (bytes_read == -1) {
            std::cerr << "Error reading from serial port" << std::endl;

        }
        while (ros::ok() && buf[0] == 0) {
            // cmd_msg.ctrl_cmd_gear = 1;
            // pub.publish(cmd_msg);
            // loop_rate.sleep();
            yhs_can_msgs::ctrl_cmd vel_msg;
            vel_msg.ctrl_cmd_gear = 3;
            vel_msg.ctrl_cmd_velocity = 0.5;
            vel_msg.ctrl_cmd_Brake = 0;
            vel_msg.ctrl_cmd_steering = 0;
            pub.publish(vel_msg);
std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 每隔1秒发布一次
        }
        if (buf[1] == 1)
        {
            cmd_msg.ctrl_cmd_gear = 3;
            pub.publish(cmd_msg);
            
        }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_server");
    ros::NodeHandle nh;

    // 打开串口设备文件
    int serial_port = open("/dev/ttyUSB1", O_RDWR);
    if (serial_port == -1) {
        ROS_ERROR("Failed to open serial port");
        return -1;
    }

    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port, &tty) != 0) {
        ROS_ERROR("Error from tcgetattr");
        return -1;
    }

    tty.c_cflag &= ~PARENB; // 无奇偶校验位
    tty.c_cflag &= ~CSTOPB; // 1位停止位
    tty.c_cflag &= ~CSIZE;  // 8位数据位
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS; // 关闭硬件流控
    tty.c_cflag |= CREAD | CLOCAL; // 开启读取并忽略控制线
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 设置为原始模式
    tty.c_oflag &= ~OPOST; // 设置为原始模式

    tty.c_cc[VMIN] = 1; // 读取一个字符时等待的最小字节数
    tty.c_cc[VTIME] = 10; // 在读取到字符之前等待的时间（以十分之一秒为单位）

    if (cfsetispeed(&tty, B9600) != 0 || cfsetospeed(&tty, B9600) != 0) {
        ROS_ERROR("Error setting baud rate");
        return -1;
    }

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error from tcsetattr");
        return -1;
    }
// --------------------------------------

    // 读取机器人位置信息并发送到串口
    // char buffer[1024];
    // while (ros::ok()) {
    //     tf::TransformListener listener;
    //     tf::StampedTransform transform;
    //     try
    //     {
    //         listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(3));
    //         listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    //         double position_x_ = transform.getOrigin().x();
    //         double position_y_ = transform.getOrigin().y();
    //         double position_yaw_ = tf::getYaw(transform.getRotation());

    //         std::string position_info = "Position: x=" + std::to_string(position_x_) +
    //                                      " y=" + std::to_string(position_y_) +
    //                                      " yaw=" + std::to_string(position_yaw_) + "\n";
            
            



    //         // 读取数据并输出到终端
    //         int buf[256];

                //  memset(buf, 0, sizeof(buf));
                //  int bytes_read = read(serial_port, buf, sizeof(buf));

    //                                  std::cout<<buf[0]<<std::endl;

    //             // if (bytes_read == -1) {
    //             //     std::cerr << "Error reading from serial port" << std::endl;
    //             //     break;
    //             // }
    //             // if (buf[0] == 0) {
    //             //     //std::cout<<buf[0]<<std::endl;
    //             //     write(serial_port, position_info.c_str(), position_info.length());
    //             // }
    //         switch (buf[0])
    //         {
    //         case 0:
                
                
    //                 write(serial_port, position_info.c_str(), position_info.length());
    //             break;
    //         case 1:
    //             std::cout<<"stop"<<std::endl;
    //             break;
            
    //         default:
    //             break;
    //         }



    //         //sleep(1); // 每隔1秒发送一次位置信息
    //     }
    //     catch (tf::TransformException& ex)
    //     {
    //         ROS_ERROR_STREAM("Failed to get robot pose: " << ex.what());
    //     }
    // }
    pub = nh.advertise<yhs_can_msgs::ctrl_cmd>("/ctrl_cmd", 1);
    // 创建两个线程
    std::thread thread1(sendPositionInfo, serial_port);
    std::thread thread2(printOne, serial_port);

    // 等待两个线程结束
    thread1.join();
    thread2.join();

    // 关闭串口
    close(serial_port);

    return 0;
}
