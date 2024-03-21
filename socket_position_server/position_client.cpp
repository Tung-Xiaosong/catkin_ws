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
    serv_addr.sin_port = htons(8888);
    if(inet_pton(AF_INET, "192.168.1.102", &serv_addr.sin_addr)<=0) {//
        std::cerr << "Invalid address/ Address not supported\n";
        return 1;
    }

    if (connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed\n";
        return 1;
    }

    while (true)
    {
        char buffer[1024] = {0};
        int valread = read(client_fd, buffer, 1024);
        valread = read(client_fd, buffer, 1024);
        std::cout << buffer << std::endl;
    }

    // 关闭连接
    close(client_fd);

    return 0;
}
