#ifndef SERIAL_SERVER_H
#define SERIAL_SERVER_H

#pragma pack(1)

struct RobotPosition {
		
		uint8_t head_1;
		uint8_t head_2;
    double position_x;
    double position_y;
    double position_yaw;
};

#pragma pack()

#endif  // SERIAL_SERVER_H
