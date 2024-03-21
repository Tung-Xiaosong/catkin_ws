#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>//inet_addr

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h> 

int main(int argc, char** argv) {
    ros::init(argc, argv, "socket_pause_server");
    ros::NodeHandle nh;

    // 创建一个ROS话题发布者
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
// --------------------------
while(ros::ok())
{
    // 创建一个TCP socket
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        ROS_ERROR("[%s] Failed to create socket", ros::this_node::getName().c_str());
        return -1;
    }

    // 设置服务器地址和端口号
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(8080);//8888
    server_address.sin_addr.s_addr = INADDR_ANY;

    // 绑定端口
    if (bind(server_socket, (struct sockaddr *)&server_address, sizeof(server_address)) == -1) {
        ROS_ERROR("[%s] Failed to bind socket", ros::this_node::getName().c_str());
        return -1;
    }

    // 开始监听传入的连接
    if (listen(server_socket, 1) == -1) {
        ROS_ERROR("[%s] Error listening on socket", ros::this_node::getName().c_str());
        return -1;
    }

    ROS_INFO("[%s] Socket server started, waiting for connection...", ros::this_node::getName().c_str());

    // 等待连接
    int connection;
    struct sockaddr_in client_address;
    socklen_t client_address_size = sizeof(client_address);
    if ((connection = accept(server_socket, (struct sockaddr *)&client_address, &client_address_size)) == -1) {
        ROS_ERROR("[%s] Error accepting connection", ros::this_node::getName().c_str());
        return -1;
    }

    ROS_INFO("[%s] Client connected", ros::this_node::getName().c_str());
// ---------------------------
    char buffer[1024];
    while (true) 
    {
        memset(buffer, 0, sizeof(buffer));
        int bytes_received = recv(connection, buffer, sizeof(buffer), 0);
        if (bytes_received <= 0) {
            ROS_ERROR("[%s] Failed to receive data from client", ros::this_node::getName().c_str());
            break;
        }

        ROS_INFO("[%s] Received data from client: %s", ros::this_node::getName().c_str(), buffer);

        ros::Rate loop_rate(100);
        // 发布ROS消息
        while(strcmp(buffer, "pause") == 0 /*&& ros::ok()*/ && connection)
        {
            ROS_INFO_ONCE("[%s] Paused !", ros::this_node::getName().c_str());
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            pub.publish(vel_msg);
            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    // 关闭连接和socket
    close(connection);
    close(server_socket);
}
    return 0;
}
