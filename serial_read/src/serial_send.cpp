#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include "/home/dxs/catkin_ws/src/serial_read/include/serial_server.h"

// // 控制数据帧结构体
// struct CtrlDataFrame {
//     unsigned char head_1;
//     unsigned char head_2;
//     unsigned char data;
//     unsigned char checksum;
// };

int main() {
    // 打开串口
    int serial_port = open("/dev/ttyUSB0", O_RDWR);
    if (serial_port == -1) {
        std::cerr << "Error opening serial port" << std::endl;
        return 1;
    }

    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error configuring serial port" << std::endl;
        return 1;
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error configuring serial port" << std::endl;
        return 1;
    }

    // 读取终端输入并发送数据帧
    while (true) {
        std::cout << "Enter data to send (1 to pause, 2 to resume): ";
        std::string input;
        std::getline(std::cin, input);
        if (input == "1" || input == "2") {
            CtrlDataFrame frame;
            frame.head_1 = 0x55;
            frame.head_2 = 0xAA;
            frame.data[0] = std::stoi(input);

            // 将数据的首地址强转为无符号char型指针
            unsigned char* data_ptr = reinterpret_cast<unsigned char*>(&frame);

            // 计算校验和
            unsigned char checksum = 0;
            for (size_t i = 0; i < sizeof(frame); i++) {
                checksum ^= data_ptr[i];
            }
            frame.checksum = checksum;

            //发送数据帧到串口
            ssize_t bytes_written = write(serial_port, data_ptr, sizeof(frame));
            if (bytes_written < 0) {
                std::cerr << "Error writing to serial port" << std::endl;
                close(serial_port);
                //return ;
            }

            std::cout << "Sent data: " << frame.data << std::endl;
        } else {
            std::cout << "Invalid input. Please enter 1 or 2." << std::endl;
        }
    }

    // 关闭串口
    close(serial_port);

    return 0;
}
