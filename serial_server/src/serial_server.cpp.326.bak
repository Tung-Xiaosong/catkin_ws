#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

#include <yhs_msgs/ctrl_cmd.h>
#include <serial_server/serial_server.h>

std::mutex mtx; // 互斥锁，用于保护共享资源
ros::Publisher pause_pub; // 暂停导航发布器

//定义一个指针，将数据首地址强转为无符号型，sizeof字节长度发送首地址// 每一帧加入开始和校验位

// 发送机器人位置信息
void sendPositionInfo(int serial_port) {

    while (ros::ok()) {
        tf::TransformListener listener;
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(5));
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            RobotPosition position_msg;
            position_msg.position_x = transform.getOrigin().x();//机器人在map下的坐标
            position_msg.position_y = transform.getOrigin().y();
            position_msg.position_yaw = tf::getYaw(transform.getRotation());//机器人在map下的朝向
            position_msg.position_x = 0.0;
            position_msg.position_y = 1.0;
            position_msg.position_yaw = 2.0;
            // dxs:
            // std::string position_info = "Position: x=" + std::to_string(position_msg.position_x) + " y=" + std::to_string(position_msg.position_y) + " yaw=" + std::to_string(position_msg.position_yaw) + "\n";
            // write(serial_port, position_info.c_str(), position_info.length());
            ROS_INFO("x: %f, y: %f, yaw: %f\n", position_msg.position_x, position_msg.position_y, position_msg.position_yaw);
            // -------------------------

            // 将数据的首地址强转为无符号char型指针
            unsigned char* data_ptr = reinterpret_cast<unsigned char*>(&position_msg);

            // // 发送数据到串口
            // ssize_t bytes_written = write(serial_port, data_ptr, sizeof(position_msg));// 结构体大小24
            // if (bytes_written < 0) {
            //     std::cerr << "Error writing to serial port" << std::endl;
            //     close(serial_port);
            //     return ;
            // }
            // ------------------------------------------
            // 计算校验和
            // uint8_t checksum = 0;
            // for (size_t i = 0; i < sizeof(position_msg); ++i) {
            //     checksum += data_ptr[i];
            // }

            // 构造数据帧
            // std::vector<uint8_t> frame;
            // frame.push_back(0xAA); // 起始位
            // frame.insert(frame.end(), data_ptr, data_ptr + sizeof(position_msg)); // 数据
            // frame.push_back(checksum); // 校验位
            // frame.push_back(0x55); // 结束位
            // -------------------------------------------
            //发送数据帧到串口
            ssize_t bytes_written = write(serial_port, data_ptr, sizeof(position_msg));
            if (bytes_written < 0) {
                std::cerr << "Error writing to serial port" << std::endl;
                close(serial_port);
                return ;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 每隔1秒发送一次
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR_STREAM("Failed to get robot pose: " << ex.what());
        } 
    }
}

void pauseNavigation(int serial_port) {

while (ros::ok())
{

    int buf[256];
    memset(buf, 0, sizeof(buf));
    int bytes_read = read(serial_port, buf, sizeof(buf));

    //ros::Rate loop_rate(100);

    if (bytes_read == -1) {
        std::cerr << "Error reading from serial port" << std::endl;

    }
    while (ros::ok() && buf[0] == 0) { // 当buf[0]为0时，表示暂停导航
        std::lock_guard<std::mutex> lock(mtx); // 使用互斥锁保护临界区
        ROS_INFO("paused !!!");
        yhs_msgs::ctrl_cmd cmd_msg;
        cmd_msg.ctrl_cmd_gear = 01;
        pause_pub.publish(cmd_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 每隔0.01秒发布一次
    }
    while (ros::ok() && buf[0] == 1) { // 当buf[1]为1时，表示继续导航
        std::lock_guard<std::mutex> lock(mtx); // 使用互斥锁保护临界区
        ROS_INFO("navigating !!!");
        yhs_msgs::ctrl_cmd cmd_msg;
        cmd_msg.ctrl_cmd_gear = 03;
        cmd_msg.ctrl_cmd_linear = 0.0;
        cmd_msg.ctrl_cmd_angular = 0.1;
        pause_pub.publish(cmd_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 每隔0.01秒发布一次
    }
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

    if (cfsetispeed(&tty, B115200) != 0 || cfsetospeed(&tty, B115200) != 0) {
        ROS_ERROR("Error setting baud rate");
        return -1;
    }

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error from tcsetattr");
        return -1;
    }
// -----------------------------------------------

    // 暂停导航发布器
    pause_pub = nh.advertise<yhs_msgs::ctrl_cmd>("/ctrl_cmd", 1);

    // 创建两个线程
    std::thread thread1(sendPositionInfo, serial_port);
    //std::thread thread2(pauseNavigation, serial_port);

    // 等待两个线程结束
    thread1.join();
    //thread2.join();

    // 关闭串口
    close(serial_port);

    return 0;
}
