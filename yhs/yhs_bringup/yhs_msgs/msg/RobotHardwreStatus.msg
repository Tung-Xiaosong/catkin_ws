std_msgs/Header header

bool emergency_stop 			#急停按钮状态 0：没触发 1：触发
bool type 								#底盘类型
uint8 anticollision 			#防撞条状态 0：没触发 1：前防撞条触发 2：后防撞条触发 3：前后防撞条触发
uint16 error 							#故障代码
uint16 io 								#io状态
uint8 battery_percentage  #电量百分比
uint8 charge 							#充电状态
