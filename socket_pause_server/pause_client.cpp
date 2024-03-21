#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>

int main() {
    // 创建socket
    int client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd == -1) {
        std::cerr << "Error creating socket\n";
        return 1;
    }

    // 连接到服务端
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8080);
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) {//
        std::cerr << "Invalid address/ Address not supported\n";
        return 1;
    }

    if (connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed\n";
        return 1;
    }

    // 从终端读取用户输入
    std::string message;
    std::cout << "Enter message to send to server: ";
    std::cin >> message;

    // 发送消息给服务端
    send(client_fd, message.c_str(), message.size(), 0);

    // 关闭连接
    close(client_fd);

    return 0;
}
