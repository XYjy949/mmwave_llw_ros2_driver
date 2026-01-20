#include "udp_interface.h"
#include <cstddef>

UdpInterface::~UdpInterface() {
    closeAllSockets();
}

// 初始化组播服务器（接收端）
int UdpInterface::initUdpMulticastServer(const std::string& ip, int port) {
     // 创建套接字
    socket_server_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_server_fd < 0) {
        std::cerr << "创建套接字失败: " << strerror(errno) << std::endl;
        return -1;
    }

    // === 关键修复：设置地址复用 ===
    int reuse = 1;
    if (setsockopt(socket_server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "设置SO_REUSEADDR失败: " << strerror(errno) << std::endl;
        close(socket_server_fd);
        return -1;
    }

    // 设置组播地址
    memset(&group_addr, 0, sizeof(group_addr));
    group_addr.sin_family = AF_INET;
    group_addr.sin_port = htons(port);
    group_addr.sin_addr.s_addr = inet_addr(ip.c_str());

    // 绑定到本地地址
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(port);

    if (bind(socket_server_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "绑定失败: " << strerror(errno) << std::endl;
        close(socket_server_fd);
        return -1;
    }

    // 加入组播组
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(ip.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(socket_server_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        std::cerr << "加入组播组失败: " << strerror(errno) << std::endl;
        close(socket_server_fd);
        return -1;
    }

    is_multicast_ = true;
    std::cout << "组播服务器初始化成功: " << ip << ":" << port << std::endl;
    return 0;
}

// 初始化单播客户端（发送端）
int UdpInterface::initUdpUnicastClient(const std::string& dest_ip, int dest_port, int local_port = 0) {
    // 创建套接字
    socket_client_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_client_fd < 0) {
        std::cerr << "创建套接字失败: " << strerror(errno) << std::endl;
        return -1;
    }

    // === 关键修复：设置地址复用 ===
    int reuse = 1;
    if (setsockopt(socket_client_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "设置SO_REUSEADDR失败: " << strerror(errno) << std::endl;
        close(socket_client_fd);
        return -1;
    }

    // 绑定到本地端口（可选）
    if (local_port > 0) {
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        local_addr.sin_port = htons(local_port);

        if (bind(socket_client_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            std::cerr << "绑定本地端口失败: " << strerror(errno) << std::endl;
            close(socket_client_fd);
            return -1;
        }
    }

    // 设置目标地址
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(dest_ip.c_str());
    addr_serv.sin_port = htons(dest_port);

    is_unicast_ = true;
    std::cout << "单播客户端初始化成功: " << dest_ip << ":" << dest_port << std::endl;
    return 0;
}

// 设置为非阻塞模式
void UdpInterface::setNonBlocking(int socket_type = 0) {
    int fd = -1;
    if (socket_type == 0) {
        fd = socket_server_fd;
    } else {
        fd = socket_client_fd;
    }

    if (fd >= 0) {
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            std::cout << "套接字设置为非阻塞模式" << std::endl;
        }
    }
}

// 带超时的接收方法（非阻塞）
int UdpInterface::receiveWithTimeout(struct sockaddr_in* src_addr, char* buffer, int& recv_len, int timeout_ms) {
    if (socket_server_fd < 0) {
        recv_len = 0;
        return -1;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_server_fd, &read_fds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(socket_server_fd + 1, &read_fds, NULL, NULL, &tv);
    
    if (ret > 0 && FD_ISSET(socket_server_fd, &read_fds)) {
        socklen_t addr_len = sizeof(struct sockaddr_in);
        recv_len = recvfrom(socket_server_fd, buffer, 40000, 0, 
                            (struct sockaddr*)src_addr, &addr_len);
        
        if (recv_len > 0) {
            return 0; // 成功接收到数据
        } else if (recv_len == 0) {
            return -1; // 连接关闭
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                recv_len = 0;
                return 0; // 非阻塞模式下正常
            }
            return -1; // 其他错误
        }
    } else if (ret == 0) {
        recv_len = 0;
        return 0; // 超时
    } else {
        recv_len = 0;
        return -1; // select错误
    }
}

// 发送数据（非阻塞）
int UdpInterface::sendToRadar(const char* data, int len, int timeout_ms = 100) {
    if (socket_client_fd < 0) {
        return -1;
    }

    // 检查套接字是否可写
    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(socket_client_fd, &write_fds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(socket_client_fd + 1, NULL, &write_fds, NULL, &tv);
    
    if (ret > 0 && FD_ISSET(socket_client_fd, &write_fds)) {
        int n = sendto(socket_client_fd, data, len, 0, 
                        (struct sockaddr*)&addr_serv, sizeof(addr_serv));
        return n;
    } else {
        return -1; // 超时或错误
    }
}

// 发送心跳包
int UdpInterface::sendHeartbeat(const std::string& heartbeat_data = "HEARTBEAT") {
    return sendToRadar(heartbeat_data.c_str(), heartbeat_data.length(), 100);
}

// 关闭所有套接字
void UdpInterface::closeAllSockets() {
    if (socket_server_fd >= 0) {
        if (is_multicast_) {
            // 离开组播组
            struct ip_mreq mreq;
            mreq.imr_multiaddr.s_addr = group_addr.sin_addr.s_addr;
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);
            setsockopt(socket_server_fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
        }
        close(socket_server_fd);
        socket_server_fd = -1;
    }

    if (socket_client_fd >= 0) {
        close(socket_client_fd);
        socket_client_fd = -1;
    }

    is_multicast_ = false;
    is_unicast_ = false;
    std::cout << "所有UDP套接字已关闭" << std::endl;
}



// 获取服务器信息
std::string UdpInterface::getServerAddress() const {
    return inet_ntoa(group_addr.sin_addr);
}
uint16_t UdpInterface::getServerPort() const {
    return ntohs(group_addr.sin_port);
}

// 获取客户端信息
std::string UdpInterface::getClientAddress() const {
    return inet_ntoa(addr_serv.sin_addr);
}

uint16_t UdpInterface::getClientPort() const {
    return ntohs(addr_serv.sin_port);
}

