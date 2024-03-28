#ifndef SERIAL_SERVER_H
#define SERIAL_SERVER_H

#pragma pack(1)

struct RobotPosition {

    uint8_t head_1;// 帧头
    uint8_t head_2;// 帧尾

    float position_x;
    float position_y;
    float position_yaw;

    unsigned char checksum;// 校验位
};

struct CtrlDataFrame {
    uint8_t head_1;
    uint8_t head_2;

    int data[1];
    unsigned char checksum;
};

#pragma pack()

#endif  // SERIAL_SERVER_H
