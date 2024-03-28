#include <iostream>
#include <chrono>
#include <thread>
#include <string.h>
#include <mutex>
#include <sstream>

#include "CppLinuxSerial/SerialPort.hpp"

#include </home/dxs/catkin_ws/src/serial_read/include/serial_server.h>

using namespace std::chrono_literals;
using namespace mn::CppLinuxSerial;

std::mutex serialMutex;

//加上校验码
int main() 
{
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::OFF, SoftwareFlowControl::OFF);
    serialPort.SetTimeout(200); // Block when reading for 1000ms
    serialPort.Open();

    std::this_thread::sleep_for(100ms);

    std::cout << "RobotPosition size :" << sizeof(RobotPosition) << std::endl;

    std::thread readThread([&](){
        uint8_t data_buf[256];
        uint8_t *p = &data_buf[0];
        uint8_t data_size = 0;

        while (true) {
            std::vector<uint8_t> readData;
            serialPort.ReadBinary(readData);

            std::lock_guard<std::mutex> lock(serialMutex);

            uint8_t *buf = readData.data();
            size_t buf_size = readData.size();

            if(buf_size > 0)
            {
                memcpy(p + data_size,  buf, buf_size);
                data_size += buf_size;
            }

            while( data_size >= sizeof(RobotPosition) )
            {
                RobotPosition* data_ptr = reinterpret_cast<RobotPosition*>(p);

                if(data_ptr->head_1 == 0x55 && data_ptr->head_2 == 0xAA)
                {
                    std::cout << "Received Position X: " << data_ptr->position_x << std::endl;
                    std::cout << "Received Position Y: " << data_ptr->position_y << std::endl;
                    std::cout << "Received Position Yaw: " << data_ptr->position_yaw << std::endl;
                    std::cout << "--------------------------------" << std::endl;
                    data_size -= sizeof(RobotPosition);
                    p += sizeof(RobotPosition);
                }
                else
                {
                    -- data_size;
                    ++ p;
                }
            }

            if(data_size > 0 && p != data_buf)
            {
                memmove(&data_buf[0], p, data_size);
            }

            p = &data_buf[0];
        }
    });

    std::thread writeThread([&](){
        while(true){
            std::string input;
            std::getline(std::cin, input);

            std::lock_guard<std::mutex> lock(serialMutex);

            std::istringstream iss(input);
            int hexValue;
            if (iss >> std::hex >> hexValue) {
                uint8_t data = static_cast<uint8_t>(hexValue);
                std::vector<uint8_t> sendData = {data};
                serialPort.Write(sendData);
            }
        }
    });

    readThread.join();
    writeThread.join();

    serialPort.Close();

    return 0;
}
