cmake_minimum_required(VERSION 3.0.2)
project(yhs_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  nav_msgs
  tf
)


 add_message_files(
   FILES
   bms_fb.msg
   bms_flag_fb.msg
   ctrl_cmd.msg
   ctrl_fb.msg
   free_ctrl_cmd.msg
   io_cmd.msg
   io_fb.msg
   l_wheel_fb.msg
   r_wheel_fb.msg
   ChassisInfoFb.msg
   RobotPosition.msg
   RobotHardwreStatus.msg
   Version.msg
   Goal.msg
 )


 add_service_files(
   FILES
   InitialPose.srv
   Pause.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs  nav_msgs
 )


catkin_package(
  CATKIN_DEPENDS geometry_msgs message_generation move_base_msgs nav_msgs pcl_msgs sensor_msgs std_msgs tf
)

