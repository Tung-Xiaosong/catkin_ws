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
    ros::init(argc, argv, "socket_position_server");
    ros::NodeHandle nh;

// --------------------------
    // 创建一个TCP socket
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        ROS_ERROR("[%s] Failed to create socket", ros::this_node::getName().c_str());
        return -1;
    }

    // 设置服务器地址和端口号
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(8888);//8888
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
    while (ros::ok()) 
    {
        tf::TransformListener listener;
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(3));

            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            
            double position_x_ = transform.getOrigin().x();//机器人在map下的坐标
            double position_y_ = transform.getOrigin().y();

            double position_yaw_ = tf::getYaw(transform.getRotation());//机器人在map下的朝向

            //ROS_INFO( "position_x_:%f,position_y_:%f,position_yaw_:%f",position_x_,position_y_,position_yaw_);

            std::string position_info = "Position: x=" + std::to_string(position_x_) + " y=" + std::to_string(position_y_) + " yaw=" + std::to_string(position_yaw_);
            send(connection, position_info.c_str(), position_info.length(), 0);

            //sleep(1); // 每隔1秒发送一次位置信息

        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR_STREAM("Failed to get robot pose: " << ex.what());
        }   
    }

    // 关闭连接和socket
    close(connection);
    close(server_socket);

    return 0;
}
