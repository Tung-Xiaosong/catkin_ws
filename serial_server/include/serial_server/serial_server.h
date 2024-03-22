#ifndef SERIAL_SERVER_H
#define SERIAL_SERVER_H

#pragma pack(1)

struct RobotPosition {
    double position_x;
    double position_y;
    double position_yaw;
};

#pragma pack()

#endif  // SERIAL_SERVER_H