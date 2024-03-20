#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h> 

int main(int argc, char** argv) {
    ros::init(argc, argv, "socket_server");
    ros::NodeHandle nh;

    // 创建一个ROS话题发布者
    ros::Publisher pub = nh.advertise<std_msgs::String>("cmd_vel", 10);
// --------------------------
    // 创建一个TCP socket
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        ROS_ERROR("Failed to create socket");
        return -1;
    }

    // 设置服务器地址和端口号
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(8888);
    server_address.sin_addr.s_addr = INADDR_ANY;

    // 绑定端口
    if (bind(server_socket, (struct sockaddr *)&server_address, sizeof(server_address)) == -1) {
        ROS_ERROR("Failed to bind socket");
        return -1;
    }

    // 开始监听传入的连接
    if (listen(server_socket, 1) == -1) {
        ROS_ERROR("Error listening on socket");
        return -1;
    }

    ROS_INFO("Socket server started, waiting for connection...");

    // 等待连接
    int connection;
    struct sockaddr_in client_address;
    socklen_t client_address_size = sizeof(client_address);
    if ((connection = accept(server_socket, (struct sockaddr *)&client_address, &client_address_size)) == -1) {
        ROS_ERROR("Error accepting connection");
        return -1;
    }

    ROS_INFO("Client connected");
// ---------------------------
    char buffer[1024];
    while (true) {
        memset(buffer, 0, sizeof(buffer));
        int bytes_received = recv(connection, buffer, sizeof(buffer), 0);
        if (bytes_received <= 0) {
            ROS_ERROR("Failed to receive data from client");
            break;
        }

        ROS_INFO("Received data from client: %s", buffer);

//dxs add
        // 获取小车位置信息
        // tf::TransformListener listener;
        // tf::StampedTransform transform;
        // try {
        //     listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        //     double x = transform.getOrigin().x();
        //     double y = transform.getOrigin().y();
        //     double z = transform.getOrigin().z();
        //     double roll, pitch, yaw;
        //     transform.getBasis().getRPY(roll, pitch, yaw);

        //     ROS_INFO("map to baselink: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
        //              x, y, z, roll, pitch, yaw);
        // } catch (tf::TransformException &ex) {
        //     ROS_ERROR("%s", ex.what());
        // }
//
        // 发布ROS消息
        if(buffer == "c")
        {
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        pub.publish(vel_msg);
        }
        ros::spin();
    }

    // 关闭连接和socket
    close(connection);
    close(server_socket);

    return 0;
}
