#include "host.h"

#include <ciso646>
#include <string.h>
#ifdef __unix__
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#else
#include <windows.h>
#endif
#include <fcntl.h>
#include <thread>
#include <chrono>
using namespace lime;

Controller::Controller() : socketFd(-1),mRunning(0)
{
#ifndef __unix__
    WSADATA wsaData;
    if( int err = WSAStartup(0x0202, &wsaData))
        printf("WSAStartup error %i\n", err);
#endif
    if(Listen(5000, false) != 0)
        Listen(5001, false);
}

Controller::~Controller()
{
    Halt();
#ifndef __unix__
    WSACleanup();
#endif
}

/** @brief Creates socket to listen for incomming connections
    @return 0-success
*/
int Controller::Listen(uint16_t port, bool localOnly)
{
    if(mRunning)
        return ALREADY_LISTENING;

    socketFd = socket(AF_INET, SOCK_STREAM, 0);
    if(socketFd < 0)
    {
        fprintf(stderr, "socket error %i\n", socketFd);
        return FAIL_CREATE_SOCKET;
    }
#ifdef __unix__
    fcntl(socketFd, F_SETFL, O_NONBLOCK);
#else
    u_long iMode = 0;
    ioctlsocket(socketFd, FIONBIO, &iMode);
#endif

    struct sockaddr_in host;
    memset((void *)&host, 0, sizeof(sockaddr_in));
    host.sin_family = AF_INET;
    host.sin_addr.s_addr = localOnly ? inet_addr("127.0.0.1") : INADDR_ANY;
    host.sin_port = htons(port);
    if(bind(socketFd, (struct sockaddr*)&host, sizeof(host)) < 0)
    {
        fprintf(stderr, "bind error on port %i\n", port);
        CloseSocket();
        return FAIL_BIND;
    }

    printf("Listening on port: %i\n", port);
    if(int error = listen(socketFd, SOMAXCONN) )
    {
#ifndef __unix__
        error =  WSAGetLastError();
#endif
        fprintf(stderr, "listen error %i\n", error);
        return FAIL_LISTEN;
    }

    workerThread = std::thread(&Controller::ProcessConnections, this);
    if(not workerThread.joinable())
    {
        fprintf(stderr, "Error creating listening thread\n");
        CloseSocket();
        return FAIL_THREAD_CREATE;
    }
    mRunning.store(true);
    return REMOTE_NO_ERROR;
}

void Controller::Halt()
{
    if(mRunning.load())
    {
        mRunning.store(false);
        CloseSocket();
        if(workerThread.joinable())
            workerThread.join();
    }
}

int Controller::ProcessConnections()
{
    printf("Started listening thread\n");
    struct sockaddr_in cli_addr;
#ifndef __unix__
    SOCKET clientFd;
    int clilen = sizeof(cli_addr);
#else
    int clientFd;
    unsigned int clilen = sizeof(cli_addr);
#endif // __unix__

    while(mRunning && socketFd >= 0)
    {
        memset(&cli_addr, 0, sizeof(cli_addr));
        clientFd = accept(socketFd, (struct sockaddr*)&cli_addr, &clilen);
        //if(clientFd == EAGAIN || clientFd == EWOULDBLOCK)
            //continue;
        if(clientFd < 0)
        {
            continue;
        }
        printf("Accepted\n");
        unsigned long ips = cli_addr.sin_addr.s_addr;

        bool connected = true;

        Packet message;
        const int msgSize = sizeof(message);;
        char* data = reinterpret_cast<char*>(&message);

        while(connected)
        {
            unsigned int r = 0;
            bool dataReceived = true;
            do
            {
                int brecv = recv(clientFd, data+r, sizeof(message)-r, 0);
                r += brecv;
                if(brecv == 0)
                {
                    connected = false;
                    printf("recv 0 %i/%i\n", r, msgSize);
                    dataReceived = false;
                    CloseSocket();
                    break;
                }
                else
                    printf("recv %i %i/%i\n", brecv, r, msgSize);
                if(brecv < 0 )
                {
#ifndef __unix__
                    brecv = WSAGetLastError();
#endif
                    printf("recv with error: %d\n", brecv);
                    CloseSocket();
                    connected = false;
                    clientFd = -1;
                    break;
                }
            }
            while(r < msgSize);

            if(not connected)
                break;

            if(not dataReceived)
                continue;

            Packet outbound;
            char* response = reinterpret_cast<char*>(&outbound);
            memset(response, 0, sizeof(Packet));

            if(spiOp)
                spiOp(message, &outbound);

            int bsent = 0;
            while(bsent < sizeof(response))
            {
                int r = send(clientFd, response+bsent, sizeof(outbound)-bsent, 0);
                if(r < 0)
                {
                    printf("response error %i\n", r);
                    CloseSocket();
                    connected = false;
                    break;
                }
                bsent += r;
            }
            printf("Sent response : %s\n", response);
        }
        if (clientFd >= 0)
            CloseSocket();
        printf("remote handling done\n");
    }
    CloseSocket();
    return 0;
}

void Controller::CloseSocket()
{
    if(socketFd > 0)
    {
#ifndef __unix__
    //shutdown(socketFd, SD_BOTH);
    closesocket(socketFd);
#else
    shutdown(socketFd, SHUT_RDWR);
    close(socketFd);
#endif
    }
    socketFd = -1;
}
