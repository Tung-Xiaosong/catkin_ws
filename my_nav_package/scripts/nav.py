#!/usr/bin/env python
# coding=utf-8

# todo:进程结束令后要关闭监听端口
import json
import os
import time

import rospy
import yaml
import socket
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

LABEL = "0"

def callback(data):
    global conn
    global LABEL
    # rospy.loginfo(data.status_list[-1])
    LABEL = data.status_list[-1].status
    # rospy.loginfo(status)
    # status == 3 means arrived!
    if LABEL == 3:
        # rospy.sleep(0.5)
        print("Arrived!")
        goal_sub.unregister()
    elif LABEL == 4:
        # rospy.sleep(0.5)
        print("nav error!")
        goal_sub.unregister()
    else:
        print("Have not arrived to the goal yet! ")
    try:
        print('send=', LABEL)
        conn.sendall(str(LABEL).encode())
    except:
        print("Connect error")

HOST = '192.168.1.192'
PORT = 8080

# the python interpreter of ros is 2.7
if __name__ == '__main__':
    # 初始化ROS节点
    serve_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serve_socket.bind((HOST, PORT))
    serve_socket.listen(5)
    rospy.init_node('move_base_client')
    rospy.loginfo("node_init!")
    # 创建MoveBaseClient客户端对象

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
    # 创建MoveBaseAction请求
    goal = PoseStamped()

    rospy.sleep(1)

    while 1:
        print("wait to connect")
        conn, addr = serve_socket.accept()
        print('Connected to' + str(addr))
        while 1:
            print("Wait input")
            data_byte = conn.recv(1024)
            if not data_byte:
                print("Connect break!")
                conn.close()
                break
            # avoid the connect error
            try:
                recv_json = json.loads(data_byte)
                print("get json!")
                # judge if the json is position json or not
                if 'pose' in recv_json:
                    print("get position!")
                    # type of axis
                    goal.header.frame_id = recv_json['header']['frame_id']
                    # destination position
                    goal.pose.position.x = recv_json['pose']['position']['x']
                    goal.pose.position.y = recv_json['pose']['position']['y']
                    goal.pose.position.z = recv_json['pose']['position']['z']
                    # destination orientation
                    goal.pose.orientation.x = recv_json['pose']['orientation']['x']
                    goal.pose.orientation.y = recv_json['pose']['orientation']['y']
                    goal.pose.orientation.z = recv_json['pose']['orientation']['z']
                    goal.pose.orientation.w = recv_json['pose']['orientation']['w']
                    # pub position
                    goal_pub.publish(goal)
                    goal_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, callback)
                    count = 0
                    while LABEL != 3:
                        print("wait to arrive")
                        time.sleep(0.5)
                        count = count + 1
                        if count > 600:
                            break

            except:
                continue
