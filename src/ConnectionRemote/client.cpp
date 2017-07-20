#include "client.h"

#ifndef __unix__
#include <winsock2.h>
#include <conio.h>
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#endif

#include <sys/types.h>

#include <stdio.h>

RemoteClient::RemoteClient() : socketFd(0)
{
#ifndef __unix__
    WSADATA wsaData;
    if(int err = WSAStartup(0x0202, &wsaData) )
        printf("WSAStartup error %i\n", err);
#endif
}

RemoteClient::~RemoteClient()
{
    Close();
#ifndef __unix__
    WSACleanup();
#endif
}

int RemoteClient::Connect(const char* ip, uint16_t port)
{
    struct sockaddr_in clientService;
    clientService.sin_family = AF_INET;
    clientService.sin_addr.s_addr = inet_addr(ip);
    clientService.sin_port = htons(port);

    socketFd = socket(AF_INET, SOCK_STREAM, 0);
    if(socketFd < 0)
    {
#ifndef __unix__
        printf("socket failed with error: %d\n", WSAGetLastError());
#endif
        return FAIL_CREATE_SOCKET;
    }

    if(int result = connect(socketFd, (struct sockaddr*)&clientService, sizeof(clientService)) )
    {
#ifndef __unix__
        printf("connect failed with error: %d\n", WSAGetLastError());
        closesocket(params->socketFd);
#else
        close(socketFd);
#endif
        return FAIL_CONNECT;
    }
    printf("Connected to %s : %i\n", ip, port);
    return NO_ERROR;
}

int RemoteClient::Close()
{
    if(socketFd)
    {
        shutdown(socketFd, SHUT_RDWR);
#ifndef __unix__
        closesocket(socketFd);
#else
        close(socketFd);
#endif
    }
    socketFd = -1;
    return NO_ERROR;
}

int RemoteClient::TransferData(const uint8_t* data, uint32_t len, uint8_t* response)
{
    int bytesWritten = 0;
    while(bytesWritten < len)
    {
        int wrBytes = write(socketFd, data+bytesWritten, len-bytesWritten);
        if(wrBytes < 0)
            return FAIL_WRITE;
        bytesWritten += wrBytes;
    }
    int bytesRead = 0;
    while(bytesRead < len)
    {
        int rdBytes = read(socketFd, response+bytesRead, len-bytesRead);
        if(rdBytes < 0)
            return FAIL_READ;
        bytesRead += rdBytes;
    }
    return NO_ERROR;
}
