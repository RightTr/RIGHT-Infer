#include <cstdio>
#include <unistd.h>
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define MAX_LISTEN 5
#define CHECK_RET(q) if((q) == false){return -1;}

using namespace std;


class TcpSocket
{
    public:
        TcpSocket() : _sockfd(-1){}

        /* 创建套接字 */
        bool Socket()
        {
            /*  AF_INET: IPv4地址族 
            *   SOCK_STREAM: 面向连接的流套接字，即TCP套接字 
            *   IPPROTO_TCP: 套接字的协议，TCP协议 */
            _sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if(_sockfd < 0)
            {
                perror("Created Error!");
                return false;
            }
            return true;
        }

        /* 为套接字绑定地址信息 */
        bool Bind(const string* ip, uint16_t port)
        {
            struct sockaddr_in addr;
            addr.sin_family = AF_INET; // 设置地址族IPv4
            addr.sin_port = htons(port); // 转换大段字节顺序
            addr.sin_addr.s_addr = inet_addr(ip->c_str()); // inet_addr(): 将ip转换为二进制格式 
            socklen_t len = sizeof(struct sockaddr_in);

            int ret = bind(_sockfd, (struct sockaddr*)&addr, len); // 绑定IP和端口至套接字
            if(ret < 0)
            {
                perror("Bind Error!");
                return false;
            }
            return true;
        }

        /* 开始监听 */
        bool Listen(int backlog = MAX_LISTEN)
        {
            int ret = listen(_sockfd, backlog);
            if(ret < 0)
            {
                perror("Listen Error!");
                return false;
            }
            return true;
        }

        /* 获取新建连接 */
        bool Accept(TcpSocket* new_sock, string* ip = NULL, uint16_t* port = NULL)
        {
            struct sockaddr_in addr;
            socklen_t len = sizeof(sockaddr_in);
            int new_fd = accept(_sockfd, (struct sockaddr*)&addr, &len);
            if(new_fd < 0)
            {
                perror("Accept Error!");
                return false;
            }
            new_sock->_sockfd = new_fd;
            if(ip != NULL)
            {
                *ip = inet_ntoa(addr.sin_addr);
            }
            if(port != NULL)
            {
                *port = ntohs(addr.sin_port);
            }
            return true;        
        }

        bool Receive(uint8_t* buf)
        {
            char temp[4096] = {0};
            /* recv(): 返回接收的字节数
            *  0: 默认接收，不做任何处理 */
            int ret = recv(_sockfd, temp, 4096, 0); // 默认阻塞，没有数据就会等待
            if(ret < 0)
            {
                perror("Receive Error!");
                return false;
            }
            else if(ret == 0) // 返回0表示连接断开
            {
                printf("Connection Broken\n");
                return false;
            }
            memcpy(buf, temp, ret); // 指定赋值的部分
            return true;
        }

        bool Send(const uint8_t* data, size_t len)
        {
            int ret = send(_sockfd, data, len, 0);
            if(ret < 0)
            {
                perror("Send Error!");
                return false;
            }
            return true;
        }
        
        /* 关闭套接字 */
        bool Close()
        {
            if(_sockfd > 0)
            {
                close(_sockfd);
                _sockfd = -1;
            }
            return true;
        }

        /* 建立连接 */
        bool Connect(const string& ip, uint16_t port)
        {
            struct sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_port = htons(port);
            addr.sin_addr.s_addr = inet_addr(ip.c_str());
            socklen_t len = sizeof(struct sockaddr_in);
            int ret = connect(_sockfd, (struct sockaddr*)&addr, len);
            if(ret < 0)
            {
                perror("Connect Error!");
                return false;
            }
            return true;
        }

    private:
        int _sockfd;

};