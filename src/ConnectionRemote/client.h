#ifndef REMOTE_CLIENT_H
#define REMOTE_CLIENT_H

#include <stdint.h>

class RemoteClient
{
public:
    enum
    {
        NO_ERROR = 0,
        ERROR,
        ALREADY_LISTENING,
        FAIL_CREATE_SOCKET,
        FAIL_BIND,
        FAIL_LISTEN,
        FAIL_THREAD_CREATE,
        FAIL_CONNECT,
        FAIL_WRITE,
        FAIL_READ,
    };

    RemoteClient();
    ~RemoteClient();

    int Connect(const char* ip, uint16_t port);
    int Close();

    int TransferData(const uint8_t* data, uint32_t len, uint8_t* response);
protected:
    int socketFd;
};

#endif // REMOTE_CLIENT_H
