实验五:手柄控制小车围捕目标
运行步骤:
roslaunch surround_car display_multi_bot_gazebo.launch打开gazebo
roslaunch surround_car display_mbot_xacro.launch打开rviz,open->config:mbot_control_3robot.rviz
roslaunch surround_car input_js.launch加入手柄
roslaunch surround_car mbot_triangle.launch形成编队(mbot_formation_node) (mbot_traing_node) (mbot_control_node)
rosrun rqt_tf_tree rqt_tf_tree查看tf树



代码解释:
launch文件:
  1.gazebo:
    display_mbot_gazebo.launch运行gazebo仿真环境,加载单个机器人模型在gazebo中
    display_multi_bot_gazebo.launch运行gazebo仿真环境,加载多个机器人模型在gazebo中,并给出各个机器人在gazebo中的位置坐标

  2.urdf:(rviz用mbot_final_rviz.rviz文件)
    display_mbot_usdf.launch运行rviz界面,单个机器人描述文件
  xacro:
    display_mbot_xacro.launch运行rviz界面,多个机器人描述文件
  
  3.mbot_triangle:
    打开mbot_formation_node节点创建carrot1,2,和laser_link之间的tf关系
    打开mbot_traing_node节点给两个围捕机器人速度,跟随mbot1的固定坐标系
    打开mbot_control_node节点启动雷达机器人mbot1跟随mbot4运动
    
src文件:
  1.mbot_tftree:发布目标机器人mbot4和雷达laser_link之间的tf(不需要)
  2.mbot_formation:在目标机器人mbot1下建立2个固定坐标系,并给出固定坐标系位置坐标(需要)
  3.mbot_traing:查找围捕机器人和固定坐标系的坐标转换,分别给两个追捕机器人发布速度命令跟随固定坐标系(需要)
  4.mbot_control:接收到/scan雷达的信息后,给雷达机器人mbot1发布速度命令,跟随目标机器人(需要)
  
  

思路:
在雷达机器人mbot1建立固定坐标系carrot1和carrot2,使围捕机器人mbot2和mbot3跟随固定坐标系运动
当雷达机器人扫描到目标机器人时,雷达机器人跟随mbot4运动,实现目的
