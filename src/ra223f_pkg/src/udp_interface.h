#ifndef UDP_INTERFACE_H
#define UDP_INTERFACE_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <string>
#include <cstring>
#include <iostream>
#include <cerrno>

class UdpInterface {
private:
    int socket_server_fd = -1;
    int socket_client_fd = -1;
    struct sockaddr_in group_addr;
    struct sockaddr_in addr_serv;
    socklen_t addr_len = sizeof(group_addr);
    bool is_multicast_ = false;
    bool is_unicast_ = false;

public:
    UdpInterface() = default;
    ~UdpInterface();

    // 组播服务器初始化
    int initUdpMulticastServer(const std::string& ip, int port);

    // 单播客户端初始化
    int initUdpUnicastClient(const std::string& dest_ip, int dest_port, int local_port);

    // 非阻塞设置
    void setNonBlocking(int socket_type);

    // 带超时的接收
    int receiveWithTimeout(struct sockaddr_in* src_addr, char* buffer, int& recv_len, int timeout_ms = 100);

    // 发送数据
    int sendToRadar(const char* data, int len, int timeout_ms);

    // 关闭所有套接字
    void closeAllSockets();

    int sendHeartbeat(const std::string& heartbeat_data);

    // 状态检查
    bool isServerSocketOpen() const;
    bool isClientSocketOpen() const;
    bool isMulticast() const;
    bool isUnicast() const;

    // 获取信息
    std::string getServerAddress() const;
    uint16_t getServerPort() const;
    std::string getClientAddress() const;
    uint16_t getClientPort() const;
};

#endif // UDP_INTERFACE_H
